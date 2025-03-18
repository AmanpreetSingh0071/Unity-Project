using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;
using System.Linq;

public class RobotController : MonoBehaviour
{
    // naming constraints do not change
    [SerializeField] private WheelCollider FLC;
    [SerializeField] private WheelCollider FRC;
    [SerializeField] private WheelCollider RLC;
    [SerializeField] private WheelCollider RRC;

    [SerializeField] private Transform FLT;
    [SerializeField] private Transform FRT;
    [SerializeField] private Transform RLT;
    [SerializeField] private Transform RRT;

    [SerializeField] private Transform FRS;
    [SerializeField] private Transform L1S;
    [SerializeField] private Transform L2S;
    [SerializeField] private Transform L3S;
    [SerializeField] private Transform R1S;
    [SerializeField] private Transform R2S;
    [SerializeField] private Transform R3S;
    [SerializeField] private Transform ORS;

    private float motorForce = 1000f;
    private float maxSteerAngle = 30f;

    private float raycastThreshold = 35f; // Maximum distance for raycasts
    private float downwardRotateDegreeFar = 3f; // Adjust this value to control the downward angle
    private float downwardRotateDegreeNear = 9f; // Adjust this value to control the downward angle
    private float wingRotateDegreeFar = 10f; // Adjust this value to control the downward angle
    private float wingRotateDegreeNear = 20f; // Adjust this value to control the downward angle
    private float sensorOneRotate = 12f; // Adjust this value to control the downward angle
    private float sensorTwoRotate = 8f; // Adjust this value to control the downward angle
    private float sensorThreeRotate = 30f; // Adjust this value to control the downward angle

    private float currentSteerAngle;
    private float currentMotorForce;
    private Transform[] sensors;

    private readonly float[][] sensorAngles = new float[][] {
        new float[] { 15f, 10f, 9f, 0f, -9f, -10f, -15f },    // Very Near
        new float[] { 13f, 9f, 8f, 0f, -8f, -9f, -13f },    // Extremely Near
        new float[] { 12f, 8f, 7f, 0f, -7f, -8f, -12f },    // Super Near
        new float[] { 11f, 7f, 6f, 0f, -6f, -7f, -11f },      // Very Near
        new float[] { 9f, 8f, 7f, 0f, -7f, -8f, -9f },      // Near
        new float[] { 8f, 7f, 6f, 0f, -6f, -7f, -8f },      // Slightly Near
        new float[] { 6f, 5f, 4f, 0f, -4f, -5f, -6f },        // Medium
        new float[] { 5f, 4f, 4f, 0f, -3f, -4f, -5f },          // Slightly Far
        new float[] { 4f, 3f, 2f, 0f, -2f, -3f, -4f },          // Far
        new float[] { 1.5f, 1f, 0.5f, 0f, -0.5f, -1f, -1.5f }           // Very Far
    };

    private readonly float[] speeds = new float[] {
        30f,    // Very Near
        35f,    // Extremely Near
        40f,    // Super Near
        45f,    // Very Near
        65f,    // Near
        75f,    // Slightly Near
        80f,    // Medium
        85f,    // Slightly Far
        90f,    // Far
        110f    // Very Far
    };

    private readonly string[] distanceTypes = new string[] {
        "VeryNear",
        "ExtremelyNear",
        "SuperNear",
        "VeryNear",
        "Near",
        "SlightlyNear",
        "Medium",
        "SlightlyFar",
        "Far",
        "VeryFar"
    };

    private readonly Color[] colors = Enumerable.Range(0, 10).Select(i => Color.Lerp(Color.red, Color.blue, (float)i / 9f)).ToArray();



    private bool[] GetSensor(int distanceIndex)
    {
        float raycastDistance = Mathf.Lerp(10f, raycastThreshold, distanceIndex / 9f);
        float downwardRotate = Mathf.Lerp(downwardRotateDegreeNear, downwardRotateDegreeFar, distanceIndex / 9f);
        float wingRotate = Mathf.Lerp(wingRotateDegreeNear, wingRotateDegreeFar, distanceIndex / 9f);

        RotateSensorsDownward(sensors, downwardRotate, wingRotate);
        bool[] distances = new bool[sensors.Length];
        LayerMask mask = ~LayerMask.GetMask("Default");

        for (int i = 0; i < sensors.Length; i++)
        {
            RaycastHit hit;
            Transform sensor = sensors[i];
            if (Physics.Raycast(sensor.position, sensor.forward, out hit, raycastDistance, mask))
            {
                // DrawLine(sensor.position, sensor.forward, colors[distanceIndex], hit.distance, .3f);
                distances[i] = !BelongsToLayer(hit.transform.gameObject, "Road");
            }
            else
            {
                // DrawLine(sensor.position, sensor.forward, colors[distanceIndex], raycastDistance, 1f);
                distances[i] = true;
            }
        }

        // Debug.Log(distanceTypes[distanceIndex] + " " + string.Format("{0}{1}{2}{3}{4}{5}{6}", 
        //     distances[0]?1:0, distances[1]?1:0, distances[2]?1:0, distances[3]?1:0, 
        //     distances[4]?1:0, distances[5]?1:0, distances[6]?1:0));

        return distances;
    }

    public float GetAngle(bool[] sensorData, int distanceIndex)
    {
        float sum = 0f;
        for (int i = 0; i < sensorData.Length; i++)
        {
            sum += sensorData[i] ? sensorAngles[distanceIndex][i] : 0f;
        }
        return sum;
    }

    void RotateSensorsDownward(Transform[] sensors, float downwardRotate, float wingRotate)
    {
        for (int i = 0; i < sensors.Length; i++)
        {
            Transform sensor = sensors[i];
            float rotateDegree = (i == 0 || i == 6) ? wingRotate : downwardRotate;
            sensor.localRotation = Quaternion.Euler(rotateDegree, sensor.localRotation.eulerAngles.y, sensor.localRotation.eulerAngles.z);
        }
    }

    public void run()
    {
        bool foundObstacle = false;

        // Check from nerest to farthest
        for (int distanceIndex = 0; distanceIndex < distanceTypes.Length; distanceIndex++)
        {
            bool[] sensorData = GetSensor(distanceIndex);

            // Check for emergency brake condition at closest distance
            if (distanceIndex == 0 && sensorData.All(s => s))
            {
                Brake();
                return;
            }

            float angle = GetAngle(sensorData, distanceIndex);

            if (angle != 0)
            {
                ApplySteeringAngle(angle);
                float speed = speeds[distanceIndex];
                if (sensorData[3] == true)
                {
                    speed -= speeds[distanceIndex] * .1f;
                }
                if (sensorData[2] == true || sensorData[4] == true)
                {
                    speed -= speeds[distanceIndex] * .4f;
                }
                if (sensorData[1] == true || sensorData[5] == true)
                {
                    speed -= speeds[distanceIndex] * .3f;
                }
                MaintainSpeed(speed);
                foundObstacle = true;
                break;
            }
        }

        if (!foundObstacle)
        {
            MaintainSpeed(140f);
        }
    }

    private bool BelongsToLayer(GameObject gameObject, string layerName)
    {
        int layer = LayerMask.NameToLayer(layerName);
        return gameObject.layer == layer;
    }

    private void DrawLine(Vector3 startPosition, Vector3 vector, Color color, float distance, float duration = 0f)
    {
        Debug.DrawLine(startPosition, startPosition + vector * distance, color, duration);
    }

    public void MaintainSpeed(float velocity)
    {
        float currentVelocity = (RLC.rpm + RRC.rpm) / 2f;
        float error = velocity - currentVelocity;

        // Calculate time delta and update accumulator
        float deltaTime = Time.time - lastUpdateTime;
        lastUpdateTime = Time.time;

        // Debug.Log("rotation: " + FindUpRotationAngle(ORS));
        // Calculate torque with proportional term
        float proportionalTerm = error * (Mathf.Abs(error) > 10f ? 10f : Mathf.Lerp(10f, 1f, Mathf.InverseLerp(10f, 0f, Mathf.Abs(error))));
        // float proportionalTerm = error * 10f;
        float torque = proportionalTerm + (FindUpRotationAngle(ORS) * 100f);

        ApplyTorque(torque);
    }

    private float errorAccumulator = 0f;
    private float lastUpdateTime = 0f;
    private const float MAX_ACCUMULATED_ERROR = 300f;  // Limit the maximum accumulated error

    private float targetSteerAngle = 0f;
    public void ApplySteeringAngle(float angle)
    {
        targetSteerAngle = angle;
    }

    private void Update()
    {
        float step = 70f * Time.deltaTime;
        FLC.steerAngle = Mathf.MoveTowards(FLC.steerAngle, targetSteerAngle, step);
        FRC.steerAngle = Mathf.MoveTowards(FRC.steerAngle, targetSteerAngle, step);
    }

    public void ApplyTorque(float torque)
    {
        // Debug.Log("Torque: " + torque);
        RLC.motorTorque = torque;
        RRC.motorTorque = torque;
        FLC.motorTorque = torque;
        FRC.motorTorque = torque;
    }

    public void Brake()
    {
        // Apply a large negative torque to quickly stop the car
        float brakeTorque = -motorForce * 2f; // Using double the motor force for strong braking
        ApplyTorque(brakeTorque);
    }

    public static float FindUpRotationAngle(Transform sensorTransform, float smoothTime = 2f)
    {
        Vector3 upNormal = Vector3.up;
        Vector3 surfaceNormal = sensorTransform.up;
        float angle = Vector3.Angle(upNormal, surfaceNormal);
        if (Vector3.Dot(upNormal, surfaceNormal) < 0f)
        {
            angle = -angle;
        }
        angle = Mathf.SmoothDampAngle(upRotationAngle, angle, ref upRotationVelocity, smoothTime);
        upRotationAngle = angle;
        return angle;
    }
    private static float upRotationAngle = 0f;
    private static float upRotationVelocity = 0f;

    private void Start()
    {
        sensors = new Transform[] { L3S, L2S, L1S, FRS, R1S, R2S, R3S };
        foreach (var sensor in sensors)
        {
            sensor.localRotation = Quaternion.identity;
        }
        L2S.localRotation = Quaternion.Euler(L2S.localRotation.eulerAngles.x, L2S.localRotation.eulerAngles.y - sensorOneRotate, L2S.localRotation.eulerAngles.z);
        R2S.localRotation = Quaternion.Euler(R2S.localRotation.eulerAngles.x, R2S.localRotation.eulerAngles.y + sensorOneRotate, R2S.localRotation.eulerAngles.z);
        L1S.localRotation = Quaternion.Euler(L1S.localRotation.eulerAngles.x, L1S.localRotation.eulerAngles.y - sensorTwoRotate, L1S.localRotation.eulerAngles.z);
        R1S.localRotation = Quaternion.Euler(R1S.localRotation.eulerAngles.x, R1S.localRotation.eulerAngles.y + sensorTwoRotate, R1S.localRotation.eulerAngles.z);
        L3S.localRotation = Quaternion.Euler(L3S.localRotation.eulerAngles.x, L3S.localRotation.eulerAngles.y - sensorThreeRotate, L3S.localRotation.eulerAngles.z);
        R3S.localRotation = Quaternion.Euler(R3S.localRotation.eulerAngles.x, R3S.localRotation.eulerAngles.y + sensorThreeRotate, R3S.localRotation.eulerAngles.z);
    }

    private void FixedUpdate()
    {
        run();
        HandleMotor();
        HandleSteering();
        UpdateWheelPoses();
    }

    private void HandleMotor()
    {
        currentMotorForce = Input.GetAxis("Vertical") * motorForce;
        // ApplyTorque(currentMotorForce);
    }

    private void HandleSteering()
    {
        currentSteerAngle = maxSteerAngle * Input.GetAxis("Horizontal");
        // ApplySteeringAngle(currentSteerAngle);
    }

    private void UpdateWheelPoses()
    {
        UpdateWheelPose(FLC, FLT);
        UpdateWheelPose(FRC, FRT);
        UpdateWheelPose(RLC, RLT);
        UpdateWheelPose(RRC, RRT);
    }

    private void UpdateWheelPose(WheelCollider collider, Transform transform)
    {
        Vector3 pos;
        Quaternion rot;
        collider.GetWorldPose(out pos, out rot);
        transform.position = pos;
        transform.rotation = rot;
    }

    public float GetWheelSpeed(WheelCollider wheel)
    {
        return wheel.rpm * wheel.radius * 2f * Mathf.PI / 60f; // Convert RPM to m/s
    }
}
