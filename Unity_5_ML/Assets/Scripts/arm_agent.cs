using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;

public class arm_agent : Agent

{
    [Header("Arm settings, to be applied to all joints")]
    public float stiffness = 500;
    public float damping = 200;
    public float forceLimit = 300;

    [Header("Arm status, only read")]
    [SerializeField]
    private float lastJointHeight = 0.0f;       // the height of the last joint, used to determine the reward, debug purpose

    [Header("Robot Components")]
    public List<ArticulationBody> joints;

    //da assegnare manualmente
    public Transform HeightObject;        // this is the cylinder attached to the last joint, the one that will be used to determine the height of the arm

    [Header("Training Settings")]
    public float targetHeight = 2.0f;
    public float rewardThreshold = 0.05f;

    [Header("Joints position, used in heuristic mode")]
    [Range(-180f, 180f)] public float joint_1 = 0.0f;
    [Range(-180f, 180f)] public float joint_2 = 0.0f;
    [Range(-180f, 180f)] public float joint_3 = 0.0f;
    [Range(-180f, 180f)] public float joint_4 = 0.0f;
    [Range(-180f, 180f)] public float joint_5 = 0.0f;
    [Range(-180f, 180f)] public float joint_6 = 0.0f;

    private ArticulationBody lastJoint;

    private float[] minLimits;
    private float[] maxLimits;

    public override void Initialize()
    {
        // Auto-detect joints
        if (joints == null || joints.Count == 0)
        {
            joints = new List<ArticulationBody>();
            foreach (var ab in GetComponentsInChildren<ArticulationBody>())
            {
                if (ab.jointType != ArticulationJointType.FixedJoint)
                    joints.Add(ab);
            }
        }

        // Store min/max limits
        int n = joints.Count;
        minLimits = new float[n];
        maxLimits = new float[n];

        for (int i = 0; i < n; i++)
        {
            minLimits[i] = joints[i].xDrive.lowerLimit;
            maxLimits[i] = joints[i].xDrive.upperLimit;
            Debug.Log($"Joint {i}: Min = {minLimits[i]}, Max = {maxLimits[i]}");

            var drive = joints[i].xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            joints[i].xDrive = drive;
        }

        lastJoint = joints[n - 1];
        Debug.Log($"{gameObject.name} - Detected {n} joints");
    }

    public override void OnEpisodeBegin()
    {
        Debug.Log($"{gameObject.name} - Episode started");

        foreach (var joint in joints)
        {
            var drive = joint.xDrive;
            drive.target = 0f;
            joint.xDrive = drive;
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
        sensor.AddObservation(lastJointHeight);      //la posizione, in altezza, dell'ultimo joint, in pratica l'unica che mi serve
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var control = actions.ContinuousActions;

        Debug.Log($"OnActionReceived called with {control.Length} actions");

        for (int i = 0; i < joints.Count; i++)
        {
            if (joints[i].jointType != ArticulationJointType.RevoluteJoint)
                continue;

            var drive = joints[i].xDrive;

            // Denormalize [-1, 1] to actual joint limit range
            float normalized = Mathf.Clamp(control[i], -1f, 1f);
            float targetAngle = Mathf.Lerp(minLimits[i], maxLimits[i], (normalized + 1f) / 2f);

            Debug.Log($"Joint {i}: Action={normalized}, Setting target={targetAngle}");

            drive.target = targetAngle;
            joints[i].xDrive = drive;
        }

        float currentHeight = lastJoint.transform.position.y;
        float distance = Mathf.Abs(targetHeight - currentHeight);

        SetReward(1f - distance);

        if (distance < rewardThreshold)
        {
            SetReward(2f);
            EndEpisode();
        }

        if (currentHeight < 0.1f)
        {
            SetReward(-1f);
            EndEpisode();
        }
    }

    // public override void Heuristic(in ActionBuffers actionsOut)
    // {
    //     var actions = actionsOut.ContinuousActions;

    //     float[] rawAngles = { joint_1, joint_2, joint_3, joint_4, joint_5, joint_6 };

    //     for (int i = 0; i < joints.Count; i++)
    //     {
    //         // Normalize slider value to [-1, 1] range based on joint limits
    //         float min = minLimits[i];
    //         float max = maxLimits[i];
    //         float angle = Mathf.Clamp(rawAngles[i], min, max);
    //         float normalized = 2f * (angle - min) / (max - min) - 1f;
    //         actions[i] = Mathf.Clamp(normalized, -1f, 1f);
    //     }
    // }
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
            
            // Debug output
            Debug.Log($"Joint {i}: Raw={rawAngles[i]}, Normalized={normalized}, Target={angle}");
        }
    }
}
