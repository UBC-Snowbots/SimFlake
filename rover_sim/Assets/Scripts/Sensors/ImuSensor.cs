using SimFlake.Ros;
using UnityEngine;

/// <summary>
/// Simulated IMU. Put it on the rover's base link (or any link: it uses that object's pose).
/// Publishes sensor_msgs/Imu on /imu/data:
///   orientation          body (FLU) -> world (ENU, Unity +z = north). Yaw 0 = facing east.
///   angular_velocity     body frame, rad/s
///   linear_acceleration  body frame, m/s^2, includes gravity like a real accelerometer
///                        (reads about +9.81 on z when sitting still and level).
/// </summary>
public class ImuSensor : MonoBehaviour
{
    public string topic = "/imu/data";
    public string frameId = "imu_link";
    public float publishRateHz = 50f;

    [Header("Noise (1 sigma). 0 = perfect")]
    public float gyroNoise = 0f;   // rad/s
    public float accelNoise = 0f;  // m/s^2

    RosPublisher<sensor_msgs.msg.Imu> pub;
    ArticulationBody articulation;
    Rigidbody body;

    Vector3 lastPos, lastVel;
    Quaternion lastRot;
    Vector3 accelSum, gyroSum;
    float sampleTime;
    int samples;
    float timer;
    bool primed;

    void Start()
    {
        articulation = GetComponentInParent<ArticulationBody>();
        body = articulation == null ? GetComponentInParent<Rigidbody>() : null;
        pub = Ros.Advertise<sensor_msgs.msg.Imu>(topic, 2);
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        Vector3 pos = transform.position;
        Quaternion rot = transform.rotation;

        // Finite differences of this object's own pose work for any link and need no physics access.
        Vector3 vel = (pos - lastPos) / dt;
        if (!primed)
        {
            lastPos = pos; lastRot = rot; lastVel = Vector3.zero;
            primed = true;
            return;
        }
        Vector3 accelWorld = (vel - lastVel) / dt;

        // Angular velocity in world frame from the rotation delta.
        Quaternion dq = rot * Quaternion.Inverse(lastRot);
        dq.ToAngleAxis(out float angleDeg, out Vector3 axis);
        if (angleDeg > 180f) angleDeg -= 360f;
        Vector3 omegaWorld = float.IsNaN(axis.x) ? Vector3.zero : axis * (angleDeg * Mathf.Deg2Rad / dt);
        if (articulation != null) omegaWorld = articulation.angularVelocity;
        else if (body != null) omegaWorld = body.angularVelocity;

        // Accelerometers measure specific force: a - g.
        Vector3 specificForce = accelWorld - Physics.gravity;
        accelSum += specificForce * dt;
        gyroSum += omegaWorld * dt;
        sampleTime += dt;
        samples++;

        lastPos = pos; lastRot = rot; lastVel = vel;

        timer += dt;
        if (timer + 1e-4f >= 1f / publishRateHz && samples > 0)
        {
            timer = 0;
            Publish(accelSum / sampleTime, gyroSum / sampleTime);
            accelSum = gyroSum = Vector3.zero;
            sampleTime = 0;
            samples = 0;
        }
    }

    void Publish(Vector3 specificForceWorld, Vector3 omegaWorld)
    {
        Quaternion inv = Quaternion.Inverse(transform.rotation);
        // Unity's angular velocity is left-handed (positive = clockwise); converting to a right-handed
        // frame flips its sign on top of the axis swap.
        Vector3 gyro = -RosFrames.UnityToFlu(inv * omegaWorld);
        Vector3 acc = RosFrames.UnityToFlu(inv * specificForceWorld);
        if (gyroNoise > 0) gyro += Noise(gyroNoise);
        if (accelNoise > 0) acc += Noise(accelNoise);

        var msg = new sensor_msgs.msg.Imu
        {
            header = RosTime.Header(frameId),
            orientation = RosFrames.UnityRotationToEnu(transform.rotation),
            angular_velocity = RosFrames.ToMsg(gyro),
            linear_acceleration = RosFrames.ToMsg(acc),
        };
        double g2 = Mathf.Max(gyroNoise * gyroNoise, 1e-6f);
        double a2 = Mathf.Max(accelNoise * accelNoise, 1e-4f);
        for (int i = 0; i < 3; i++)
        {
            msg.orientation_covariance[i * 4] = 1e-4;
            msg.angular_velocity_covariance[i * 4] = g2;
            msg.linear_acceleration_covariance[i * 4] = a2;
        }
        pub.Publish(msg);
    }

    static Vector3 Noise(float sigma) =>
        new Vector3(Gauss(), Gauss(), Gauss()) * sigma;

    static float Gauss()
    {
        float u1 = 1f - Random.value, u2 = Random.value;
        return Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
    }
}
