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

    public Renderer baseRenderer;       //the object that changes color on episode begin and end

    [Header("Training Settings")]
    public float targetHeight = 2.0f;
    public float rewardThreshold = 0.05f;


    [Header("Movement Settings")]
    public float maxSpeed = 90f; // degrees per second
    public float smoothTime = 0.3f; // time to reach target (SmoothDamp parameter)

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
    
    // Simple smoothing variables
    private float[] currentTargets;
    private float[] targetVelocities;
    
    float[] homePosition = { 0f, 30f, -63f, 0f, 0f, 0f };

    public override void Initialize()
    {
        //time
        Time.timeScale = 15f;

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

        // Store min/max limits and initialize smoothing arrays
        int n = joints.Count;
        minLimits = new float[n];
        maxLimits = new float[n];
        currentTargets = new float[n];
        targetVelocities = new float[n];

        for (int i = 0; i < n; i++)
        {
            minLimits[i] = joints[i].xDrive.lowerLimit;
            maxLimits[i] = joints[i].xDrive.upperLimit;
            currentTargets[i] = 0f;
            targetVelocities[i] = 0f;

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

        // Reset color
        if (baseRenderer != null)
            baseRenderer.material.color = Color.white;
        
        float[] homePosition = { 0f, 30f, -63f, 0f, 0f, 0f };    

        for (int i = 0; i < joints.Count; i++)
        {
            // Use home position if defined, otherwise default to 0
            float homePos = (i < homePosition.Length) ? homePosition[i] : 0f;
            
            // Clamp to joint limits
            homePos = Mathf.Clamp(homePos, minLimits[i], maxLimits[i]);
            
            currentTargets[i] = homePos;
            targetVelocities[i] = 0f;
            
            var drive = joints[i].xDrive;
            drive.target = homePos;
            joints[i].xDrive = drive;
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

        for (int i = 0; i < joints.Count; i++)
        {
            if (joints[i].jointType != ArticulationJointType.RevoluteJoint)
                continue;

            // Denormalize [-1, 1] to actual joint limit range
            float normalized = Mathf.Clamp(control[i], -1f, 1f);
            float desiredTarget = Mathf.Lerp(minLimits[i], maxLimits[i], (normalized + 1f) / 2f);

            // Apply smooth movement with speed limit
            currentTargets[i] = Mathf.SmoothDamp(
                currentTargets[i],
                desiredTarget,
                ref targetVelocities[i],
                smoothTime,
                maxSpeed,
                Time.fixedDeltaTime
            );

            // Set the smoothed target to the joint
            var drive = joints[i].xDrive;
            drive.target = currentTargets[i];
            joints[i].xDrive = drive;

            Debug.Log($"Joint {i}: Desired={desiredTarget:F1}, Smooth={currentTargets[i]:F1}, Vel={targetVelocities[i]:F1}");
        }

        float currentHeight = HeightObject.transform.position.y;
        float scaledReward = Mathf.Clamp01(currentHeight / targetHeight);

        AddReward(scaledReward);  // Reward always between 0 and 1

        if (currentHeight < 0.1f)
        {
            SetReward(-1f);
            if (baseRenderer != null) baseRenderer.material.color = Color.red;          //color to indicate status
            EndEpisode();
        }

        if (currentHeight > targetHeight)
        {
            SetReward(1f);  // Bonus for achieving the goal
            if (baseRenderer != null) baseRenderer.material.color = Color.green;
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
