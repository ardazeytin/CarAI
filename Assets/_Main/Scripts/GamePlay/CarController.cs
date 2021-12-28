using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.Events;

public class CarController : MonoBehaviour
{
    [Header("Required References")]
    public Transform path; //path to drive
    public WheelCollider wheelFL;
    public WheelCollider wheelFR;
    public WheelCollider wheelRL;
    public WheelCollider wheelRR;
    [SerializeField] private Transform steeringWheelTransform;

    [Header("Settings")]
    public AIType currentAIType = AIType.Player;
    public float maxSteerAngle = 45f;   
    public float maxMotorTorque = 80f;
    public float maxBrakeTorque = 150f;
    public float maxSpeed = 100f;
    public Vector3 centerOfMass;
    public bool IsAIActive { get; set; }

    [Header("Sensors")]
    [SerializeField] private bool autoFCenterSensorDistance = true;
    [SerializeField] private float fCenterSensorDistance = 3f;
    [SerializeField] private float fSideSensorDistance = 2f;
    [SerializeField] private float fSideSensorAngle = 30f;
    [SerializeField] private Transform fCenterSensorTransform;
    [SerializeField] private Transform fCenterLeftSensorTransform;
    [SerializeField] private Transform fCenterRightSensorTransform;
    [SerializeField] private Transform fSideLeftSensorTransform;
    [SerializeField] private Transform fSideRightSensorTransform;
    private float initialFCenterSensorDistance { get; set; }
    

    [Header("Events")]
    public UnityEvent OnAIActivated;
    public UnityEvent OnAIDeactivated;

    [Header("Debug"), Space(20)]
    [SerializeField] private bool isDebugModeEnabled = false;

    //Steering Wheel
    float steeringWheelSmoothTime = 0.3f;
    float steeringWheelZVelocity = 0.0f;

    //Car-Engine
    private bool isBraking = false;
    private float currentSpeed;

    //Path
    [HideInInspector] public UnityAction OnPathCompleted;
    public bool IsPathCompleted { get; private set; }
    private bool[] isPathClear = new bool[5]; //size = sensor count
    private List<Transform> nodes;
    private int currectNode = 0;
    private PathController currentPathController;

    //AI
    public enum AIType { Player, NPC}

    private void Awake()
    {
        IsAIActive = false;
        IsPathCompleted = false;
        currentPathController = path.GetComponent<PathController>();
        initialFCenterSensorDistance = fCenterSensorDistance;
        OnAIActivated.AddListener(LogAIState);
        OnAIDeactivated.AddListener(LogAIState);
    }

    private void Start()
    {        
        GetComponent<Rigidbody>().centerOfMass = centerOfMass;

        Transform[] pathTransforms = path.GetComponentsInChildren<Transform>();
        nodes = new List<Transform>();
        
        for (int i = 0; i < pathTransforms.Length; i++)
        {
            if (pathTransforms[i] != path.transform)
            {
                nodes.Add(pathTransforms[i]);
            }
        }

        if (isDebugModeEnabled == true)
        {
            Debug.Log($"CarController -> CurrentPathType = {currentPathController.pathType}");
        }

        //if (currentAIType == AIType.NPC)
        //{
        //    SetActiveAI(true);
        //}

        SetActiveAI(true);
    }

    private void Update()
    {
        if (isDebugModeEnabled)
        {
            //Activate or deactivate AI
            if (Input.GetKeyDown(KeyCode.Alpha0))
            {
                SetActiveAI(!IsAIActive);
            }
        }
    }

    private void FixedUpdate()
    {
        if (IsAIActive == true)
        {
            Sensors();
            ApplySteer();
            Drive();
            CheckWaypointDistance();
            Brake();
        }
        else
        {
            Brake(); //Apply brake and wait for activation
        }
    }

    public void SetActiveAI(bool isActive)
    {
        IsAIActive = isActive;
        isBraking = !isActive;

        if (IsAIActive)
        {
            OnAIActivated.Invoke();          
        }
        else
        {
            OnAIDeactivated.Invoke();
        }
    }

    /// <summary>
    /// 0 = Player, 1 = NPC
    /// </summary>
    /// <returns></returns>
    public int GetAIType()
    {
        return (int)currentAIType;
    }

    private void LogAIState()
    {
        Debug.Log("CarController -> Name: " + gameObject.name + " State: IsAIActive = " + IsAIActive);
    }
    
    #region Car/Engine Features

    private void Sensors()
    {
        RaycastHit hit;
        CalculateFCenterSensorDistance();

        //front center sensor
        if (Physics.Raycast(fCenterSensorTransform.position, transform.forward, out hit, fCenterSensorDistance))
        {
            Debug.DrawLine(fCenterSensorTransform.position, hit.point, Color.red);
            isPathClear[0] = false;
        }
        else
        {
            Debug.DrawRay(fCenterSensorTransform.position, transform.forward * fCenterSensorDistance, Color.green);
            isPathClear[0] = true;
        }

        //front right sensor      
        if (Physics.Raycast(fCenterRightSensorTransform.position, transform.forward, out hit, fCenterSensorDistance))
        {
            Debug.DrawLine(fCenterRightSensorTransform.position, hit.point, Color.red);
            isPathClear[1] = false;
        }
        else
        {
            Debug.DrawRay(fCenterRightSensorTransform.position, transform.forward * fCenterSensorDistance, Color.green);
            isPathClear[1] = true;
        }

        //front right angle sensor
        if (Physics.Raycast(fSideRightSensorTransform.position, Quaternion.AngleAxis(fSideSensorAngle, transform.up) * transform.forward, out hit, fSideSensorDistance))
        {
            Debug.DrawLine(fSideRightSensorTransform.position, hit.point, Color.red);
            isPathClear[2] = false;
        }
        else
        {
            Debug.DrawRay(fSideRightSensorTransform.position, Quaternion.AngleAxis(fSideSensorAngle, transform.up) * transform.forward * fSideSensorDistance, Color.green);
            isPathClear[2] = true;
        }

        //front left sensor
        if (Physics.Raycast(fCenterLeftSensorTransform.position, transform.forward, out hit, fCenterSensorDistance))
        {
            Debug.DrawLine(fCenterLeftSensorTransform.position, hit.point, Color.red);
            isPathClear[3] = false;
        }
        else
        {
            Debug.DrawRay(fCenterLeftSensorTransform.position, transform.forward * fCenterSensorDistance, Color.green);
            isPathClear[3] = true;
        }

        //front left angle sensor
        if (Physics.Raycast(fSideLeftSensorTransform.position, Quaternion.AngleAxis(-fSideSensorAngle, transform.up) * transform.forward, out hit, fSideSensorDistance))
        {
            Debug.DrawLine(fSideLeftSensorTransform.position, hit.point, Color.red);
            isPathClear[4] = false;
        }
        else
        {
            Debug.DrawRay(fSideLeftSensorTransform.position, Quaternion.AngleAxis(-fSideSensorAngle, transform.up) * transform.forward * fSideSensorDistance, Color.green);
            isPathClear[4] = true;
        }

        //check data from all sensors
        if (isPathClear[0] == true && isPathClear[1] == true && isPathClear[2] == true && isPathClear[3] == true && isPathClear[4] == true)
        {
            isBraking = false;
        }
        else
        {
            isBraking = true;
        }

    }

    public void SetFCenterSensorDistance(float distance)
    {
        fCenterSensorDistance = distance;
    }

    public void SetFSideSensorDistance(float distance)
    {
        fSideSensorDistance = distance;
    }

    private void CalculateFCenterSensorDistance()
    {
        if (autoFCenterSensorDistance)
        {
            fCenterSensorDistance = initialFCenterSensorDistance + (currentSpeed / 5);
        }               
    }
      
    private void CheckWaypointDistance()
    {
        if (Vector3.Distance(transform.position, nodes[currectNode].position) > 1.1f && Vector3.Distance(transform.position, nodes[currectNode].position) < 2f)
        {
            if (currentSpeed > maxSpeed * 0.5f)
            {
                isBraking = true;
                //Debug.Log($"CarController -> Apply brake before entering apex! CurrentSpeed = {currentSpeed}");
            }
            else
            {
                isBraking = false;
            }
        }
        else if (Vector3.Distance(transform.position, nodes[currectNode].position) < 1f)
        {
            if (currentPathController.pathType == PathController.PathType.Circuit)
            {
                if (currectNode == nodes.Count - 1)
                {
                    currectNode = 0;
                    IsPathCompleted = true;
                    OnPathCompleted?.Invoke();
                    Debug.Log($"CarController -> CurrentNode {currectNode}. START NEXT LAP!");
                }
                else
                {
                    currectNode++;
                    //Debug.Log($"CarController -> CurrentNode {currectNode}");
                }
            }
            else if (currentPathController.pathType == PathController.PathType.Sprint)
            {
                if (currectNode == nodes.Count - 1)
                {
                    currectNode = 0;
                    IsPathCompleted = true;
                    SetActiveAI(false);
                    OnPathCompleted?.Invoke();
                    Debug.Log($"CarController -> CurrentNode {currectNode}. SPRINT FINISHED!");
                }
                else
                {
                    currectNode++;
                    //Debug.Log($"CarController -> CurrentNode {currectNode}");
                }
                
            }
        }
    }

    private void Brake()
    {
        if (isBraking)
        {
            wheelRL.brakeTorque = maxBrakeTorque;
            wheelRR.brakeTorque = maxBrakeTorque;
        }
        else
        {
            wheelRL.brakeTorque = 0;
            wheelRR.brakeTorque = 0;
        }
    }

    private void ApplySteer()
    {
        Vector3 relativeVector = transform.InverseTransformPoint(nodes[currectNode].position);
        float newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngle;
        wheelFL.steerAngle = newSteer;
        wheelFR.steerAngle = newSteer;

        if (steeringWheelTransform != null)
        {
            float smoothZ = Mathf.SmoothDampAngle(steeringWheelTransform.localEulerAngles.z, wheelFL.steerAngle * -1.5f, ref steeringWheelZVelocity, steeringWheelSmoothTime);
            steeringWheelTransform.localEulerAngles = new Vector3(steeringWheelTransform.localEulerAngles.x, steeringWheelTransform.localEulerAngles.y, smoothZ);
        }
    }

    private void Drive()
    {
        currentSpeed = 2 * Mathf.PI * wheelFL.radius * wheelFL.rpm * 60 / 1000;
        if (currentSpeed < maxSpeed && isBraking == false)
        {
            wheelFL.motorTorque = maxMotorTorque;
            wheelFR.motorTorque = maxMotorTorque;
        }
        else
        {
            wheelFL.motorTorque = 0;
            wheelFR.motorTorque = 0;
        }
    }

    #endregion

    #region Collision

    private void OnCollisionEnter(Collision collision)
    {
        if (currentAIType != AIType.Player) return;

        if (collision.transform.GetComponentInParent<CarController>())
        {            
            SetActiveAI(false);
            Debug.Log($"{GetType().Name}: CRASH! -> {collision.gameObject.name}");
        }   
    }

    #endregion
}
