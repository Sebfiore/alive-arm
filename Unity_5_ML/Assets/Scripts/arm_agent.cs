using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using Unity.VisualScripting;

public class arm_agent : Agent
{
    [Header("Arm settings, to be applied to all joints")]
    public float stiffness = 0f; //10000f;
    public float damping = 0f; // 2000f;
    public float forceLimit = 0f; // 1000f;

    [Header("Arm status, only read")]
    [SerializeField]
    private float lastJointHeight = 0.0f;

    [Header("Robot Components")]
    public List<ArticulationBody> joints;
    public Transform HeightObject;
    public Renderer baseRenderer;

    [Header("Training Settings")]
    public float targetHeight = 0.0f;//set in inspector
    //public float maxExpectedHeight = 0.0f;

    [Header("Movement Settings")]
    public float maxSpeed = 0f; // 90f;
    public float maxDeltaPerStep = 0f; // 5f;

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
        //Time.timeScale = 1f;

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
            maxLimits[i] = joints[i].xDrive.upperLimit;// popola i limiti min e max
            currentTargets[i] = 0f; //initialize current targets

            joints[i].mass = 0.5f;  // Set very low mass for each link, see if work better

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
        //if (baseRenderer != null)
        //    baseRenderer.material.color = Color.white;

        for (int i = 0; i < joints.Count; i++)
        {
            float homePos = (i < homePosition.Length) ? homePosition[i] : 0f;
            homePos = Mathf.Clamp(homePos, minLimits[i], maxLimits[i]);
            currentTargets[i] = homePos;        // current target is set to home position, i will be able to use it in heuristic mode

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

        // === Interpret action ===
        float normalized = Mathf.Clamp(control[i], -1f, 1f);
        float desiredTarget = Mathf.Lerp(minLimits[i], maxLimits[i], (normalized + 1f) / 2f);

        // Delta step
        float delta = desiredTarget - currentTargets[i];
        delta = Mathf.Clamp(delta, -maxDeltaPerStep, maxDeltaPerStep);

        // Apply with safety margin
        float safetyMargin = 2f; // degrees
        currentTargets[i] += delta;
        currentTargets[i] = Mathf.Clamp(
            currentTargets[i],
            minLimits[i] + safetyMargin,
            maxLimits[i] - safetyMargin
        );

        // === PD Controller ===
        var drive = joints[i].xDrive;

        float Kp = 20.0f;   // proportional gain (stiffness)
        float Kd = 0.5f;   // derivative gain (damping)

        float currentAngle = joints[i].jointPosition[0];   // current joint angle (radians)
        float currentVel   = joints[i].jointVelocity[0];   // current joint velocity (radians/sec)

        float error = currentTargets[i] - currentAngle;    // how far from target
        float derivative = -currentVel;                    // oppose fast motion

        float controlSignal = Kp * error + Kd * derivative;

        // Convert control signal into new drive target
        float newTarget = currentAngle + controlSignal * Time.fixedDeltaTime;

        // Clamp new target safely
        newTarget = Mathf.Clamp(
            newTarget,
            minLimits[i] + safetyMargin,
            maxLimits[i] - safetyMargin
        );

        drive.target = newTarget;
        joints[i].xDrive = drive;
    }

    // === Rewards ===
    float currentHeight = HeightObject.transform.position.y;
    float heightReward = Mathf.Clamp01(currentHeight / targetHeight);
    AddReward(heightReward * 0.1f);

    // Time + velocity penalties
    AddReward(-0.001f);
    foreach (var joint in joints)
        AddReward(-0.001f * Mathf.Abs(joint.jointVelocity[0]));

    // === Failure condition (RED) ===
    if (currentHeight < 0.5f)
    {
        Debug.Log("FAILURE: Arm collapsed, turning base RED");
        SetReward(-1f);
        if (baseRenderer != null)
            baseRenderer.material.color = Color.red;
        EndEpisode();
        return;
    }

    // === Success condition (GREEN) ===
    if (currentHeight > targetHeight)
    {
        Debug.Log("SUCCESS: Target reached, turning base GREEN");
        SetReward(+1f);
        if (baseRenderer != null)
            baseRenderer.material.color = Color.green;
        EndEpisode();
    }

    lastJointHeight = currentHeight;
}


    

    //testing a delta movement control
/*     public override void OnActionReceived(ActionBuffers actions)
{
    var control = actions.ContinuousActions;        // control is a vector of size joints.Count, each value in [-1, 1], 6 elements in total

    for (int i = 0; i < joints.Count; i++)
    {
        if (joints[i].jointType != ArticulationJointType.RevoluteJoint)
            continue;

        // Action is in [-1, 1], scale to delta change in degrees
        float delta = control[i] * maxDeltaPerStep;  

        // Update current target by delta
        currentTargets[i] += delta;
        currentTargets[i] = Mathf.Clamp(currentTargets[i], minLimits[i], maxLimits[i]);

        // Apply to articulation drive
        var drive = joints[i].xDrive;
        drive.target = currentTargets[i];
        joints[i].xDrive = drive;
    }

    // === Rewards ===
    float currentHeight = HeightObject.transform.position.y;

    // Dense shaping reward: ratio of current height to target height
    float heightReward = Mathf.Clamp01(currentHeight / targetHeight);
    AddReward(heightReward * 0.1f); // scale factor to balance with other terms

    // Small penalties
    AddReward(-0.0005f); // time penalty

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
        SetReward(+1f);
        if (baseRenderer != null) baseRenderer.material.color = Color.green;
        EndEpisode();
    }

    lastJointHeight = currentHeight;
} */

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
