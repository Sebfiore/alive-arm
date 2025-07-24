using UnityEngine;
using System.Collections.Generic; // Required for lists

public class ArticulationBodyController : MonoBehaviour
{
    // Public variables to assign in the Inspector
    [Tooltip("List of Articulation Bodies to control.  Order should match the sliders.")]
    public List<ArticulationBody> articulationBodies = new List<ArticulationBody>();

    // Optional:  If you want to control the root
    [Tooltip("Optional: Articulation Body for the root of the robot.  This is usually Fixed, and you might not need to control it.")]
    public ArticulationBody rootArticulationBody;


    public bool set_mass = false;
    private float joint_mass = 10f;

    private float[] min_limits = new float[6] { -90f, -90f, -90f, -90f, -90f, -90f };
    private float[] max_limits = new float[6] { 90f, 90f, 90f, 90f, 90f, 90f }; // Initialized to 90

    private float[] joint_angles = new float[6];
    //angles
    /*
    [Range(min_limits[0], max_limits[0])] 
    public float angle_1 = 0.0f;

    [Range(min_limits[1], max_limits[1])]      //crea lo slider !!, usa questa idea per modificare anche quelli sotto
    public float angle_2 = 0.0f;

    [Range(min_limits[2], max_limits[2])]
    public float angle_3 = 0.0f;

    [Range(min_limits[3], max_limits[3])]      //crea lo slider !!
    public float angle_4 = 0.0f;
    
    [Range(min_limits[4], max_limits[4])] 
    public float angle_5 = 0.0f;

    [Range(min_limits[5], max_limits[5])]      //crea lo slider !!
    public float angle_6 = 0.0f;*/

    [Range(-90f, 90f)]
    public float angle_1 = 0.0f;

    [Range(-90f, 90f)]
    public float angle_2 = 0.0f;

    [Range(-140f, 90f)]
    public float angle_3 = 0.0f;

    [Range(-90f, 90f)]
    public float angle_4 = 0.0f;

    [Range(-90f, 90f)]
    public float angle_5 = 0.0f;

    [Range(-90f, 90f)]
    public float angle_6 = 0.0f;
    
 
    private void Start()
    {
        // Error checking: Ensure list is not empty
        if (articulationBodies.Count == 0)
        {
            Debug.LogError("Error: Articulation Bodies list is empty!  Disabling script.");
            this.enabled = false; // Disable the script to prevent errors
            return;
        }


        // Initialize Articulation Body settings
        for (int i = 0; i < articulationBodies.Count; i++)
        {
            ArticulationBody body = articulationBodies[i];

            // Get joint limits from ArticulationBody
            float minValue = GetMinJointValue(body);
            float maxValue = GetMaxJointValue(body);
            float currentValue = GetCurrentJointValue(body);

            if( set_mass )
            {
                body.mass = joint_mass;     //per la massa, automatico
            }

            joint_angles[i] = currentValue;

            //min_limits[i] = minValue;
            //max_limits[i] = maxValue;



            // Print the values, but only if it is a Revolute Joint
            if (body.jointType == ArticulationJointType.RevoluteJoint)
            {
                Debug.Log("Joint " + i + " (" + body.name + ") is Revolute:");
                Debug.Log("  Current Value: " + currentValue);
                Debug.Log("  Min Limit:     " + minValue);
                Debug.Log("  Max Limit:     " + maxValue);
            }
        }

        angle_1 = joint_angles[0];
        angle_2 = joint_angles[1];
        angle_3 = joint_angles[2];
        angle_4 = joint_angles[3];
        angle_5 = joint_angles[4];
        angle_6 = joint_angles[5];

    }


    private void FixedUpdate()
    {
        for (int i = 0; i < articulationBodies.Count; i++)
        {
            ArticulationBody body = articulationBodies[i];
            if (body != null && body.jointType == ArticulationJointType.RevoluteJoint)
            {
                ArticulationDrive drive = body.xDrive;
                float targetAngle = 0f; // <---- REMOVE 'private' HERE

                switch (i)
                {
                    case 0:
                        targetAngle = angle_1;
                        break;
                    case 1:
                        targetAngle = angle_2;
                        break;
                    case 2:
                        targetAngle = angle_3;
                        break;
                    case 3:
                        targetAngle = angle_4;
                        break;
                    case 4:
                        targetAngle = angle_5;
                        break;
                    case 5:
                        targetAngle = angle_6;
                        break;
                    default:
                        continue;
                }

                drive.target = targetAngle;
                body.xDrive = drive;
            }
        }
    }
 

    private float GetCurrentJointValue(ArticulationBody body)
    {
  
        return body.jointPosition[0]; // Get the rotation
        
    }

    private float GetMinJointValue(ArticulationBody body)
    {
    
        return body.xDrive.lowerLimit;
    
    }

    private float GetMaxJointValue(ArticulationBody body)
    {
      
        return body.xDrive.upperLimit;
        
    }
}
