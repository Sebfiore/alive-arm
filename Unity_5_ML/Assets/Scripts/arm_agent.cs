using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;

public class arm_agent : Agent
{
    [Header("Arm settings, to be applied to all joints")]
    public float stiffness = 10000f;
    public float damping = 2000f;
    public float forceLimit = 1000f;

    [Header("Arm status, only read")]
    [SerializeField]
    private float lastJointHeight = 0.0f;

    [Header("Robot Components")]
    public List<ArticulationBody> joints;
    public Transform HeightObject;
    public Renderer baseRenderer;

    [Header("Training Settings")]
    public float targetHeight = 2.0f;
    public float maxExpectedHeight = 3.0f;

    [Header("Movement Settings")]
    public float maxSpeed = 90f;
    public float maxDeltaPerStep = 5f;

    [Header("Joints position, used in heuristic mode")]
    [Range(-180f, 180f)] public float joint_1 = 0.0f;
    [Range(-180f, 180f)] public float joint_2 = 0.0f;
    [Range(-180f, 180f)] public float joint_3 = 0.0f;
    [Range(-180f, 180f)] public float joint_4 = 0.0f;
    [Range(-180f, 180f)] public float joint_5 = 0.0f;
    [Range(-180f, 180f)] public float joint_6 = 0.0f;

    private float[] minLimits;
    private float[] maxLimits;
    private float[] currentTargets;

    float[] homePosition = { 0f, 30f, -63f, 0f, 0f, 0f };

    public override void Initialize()
    {
        Time.timeScale = 1f;

        if (joints == null || joints.Count == 0)
        {
            joints = new List<ArticulationBody>();
            foreach (var ab in GetComponentsInChildren<ArticulationBody>())
            {
                if (ab.jointType != ArticulationJointType.FixedJoint)
                    joints.Add(ab);
            }
        }

        int n = joints.Count;
        minLimits = new float[n];
        maxLimits = new float[n];
        currentTargets = new float[n];

        for (int i = 0; i < n; i++)
        {
            minLimits[i] = joints[i].xDrive.lowerLimit;
            maxLimits[i] = joints[i].xDrive.upperLimit;
            currentTargets[i] = 0f;

            joints[i].mass = 0.01f;  // Set very low mass for each link, see if work better

            var drive = joints[i].xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            joints[i].xDrive = drive;
        }

        Debug.Log($"{gameObject.name} - Detected {n} joints");
    }

    public override void OnEpisodeBegin()
    {
        if (baseRenderer != null)
            baseRenderer.material.color = Color.white;

        for (int i = 0; i < joints.Count; i++)
        {
            float homePos = (i < homePosition.Length) ? homePosition[i] : 0f;
            homePos = Mathf.Clamp(homePos, minLimits[i], maxLimits[i]);
            currentTargets[i] = homePos;

            var drive = joints[i].xDrive;
            drive.target = homePos;
            joints[i].xDrive = drive;

            joints[i].linearVelocity = Vector3.zero;
            joints[i].angularVelocity = Vector3.zero;
        }

        if (TryGetComponent<Rigidbody>(out var rb))
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        foreach (var joint in joints)
        {
            sensor.AddObservation(joint.jointPosition[0]);
            sensor.AddObservation(joint.jointVelocity[0]);
        }

        lastJointHeight = HeightObject.transform.position.y;
        sensor.AddObservation(lastJointHeight);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var control = actions.ContinuousActions;

        for (int i = 0; i < joints.Count; i++)
        {
            if (joints[i].jointType != ArticulationJointType.RevoluteJoint)
                continue;

            float normalized = Mathf.Clamp(control[i], -1f, 1f);
            float desiredTarget = Mathf.Lerp(minLimits[i], maxLimits[i], (normalized + 1f) / 2f);

            float delta = desiredTarget - currentTargets[i];
            delta = Mathf.Clamp(delta, -maxDeltaPerStep, maxDeltaPerStep);

            currentTargets[i] += delta;
            currentTargets[i] = Mathf.Clamp(currentTargets[i], minLimits[i], maxLimits[i]);

            var drive = joints[i].xDrive;
            drive.target = currentTargets[i];
            joints[i].xDrive = drive;
        }

        float currentHeight = HeightObject.transform.position.y;
        float heightReward = Mathf.Clamp01(currentHeight / maxExpectedHeight);
        AddReward(heightReward);

        // Penalties
        AddReward(-0.001f); // time penalty
        foreach (var joint in joints)
            AddReward(-0.001f * Mathf.Abs(joint.jointVelocity[0]));

        // Failure condition
        if (currentHeight < 0.1f)
        {
            SetReward(-1f);
            if (baseRenderer != null) baseRenderer.material.color = Color.red;
            EndEpisode();
            return;
        }

        // Success condition
        if (currentHeight > targetHeight)
        {
            SetReward(1f);
            if (baseRenderer != null) baseRenderer.material.color = Color.green;
            EndEpisode();
        }

        lastJointHeight = currentHeight;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var actions = actionsOut.ContinuousActions;

        float[] rawAngles = { joint_1, joint_2, joint_3, joint_4, joint_5, joint_6 };

        for (int i = 0; i < joints.Count && i < rawAngles.Length; i++)
        {
            float min = minLimits[i];
            float max = maxLimits[i];
            float angle = Mathf.Clamp(rawAngles[i], min, max);
            float normalized = 2f * (angle - min) / (max - min) - 1f;
            actions[i] = Mathf.Clamp(normalized, -1f, 1f);
        }
    }
}
