using System;
using UnityEngine;

/// <summary>
/// One driven wheel: motor + gearhead + speed controller, plus what the wheel feels from the ground.
/// Sits on the wheel's ArticulationBody. <see cref="MotorController"/> sets <see cref="TargetSpeed"/>.
///
/// How the torque is applied: the joint keeps a PhysX velocity drive (implicit, so it is stable at any gain) but every
/// physics step its force limit is set to the torque the motor can actually deliver right now:
///   - voltage limit: DC motor curve, stall torque at 0 rpm falling to 0 at no-load speed (back-EMF). So the wheel
///     can't exceed the no-load speed, and it has less torque the faster it spins;
///   - current limit of the controller (derated when the winding gets hot);
///   - gearhead efficiency and torque rating.
/// The speed setpoint ramps at the controller's acceleration limit. The speed loop is P (optionally PI): the drive's
/// damping is the P gain; an integral term would shift the drive's target velocity (torque = Kp (target + I/Kp - speed)). The drive type must be Force: Unity's "Velocity"/"Target" drive types
/// compute their own (near-infinite) gains and ignore damping, which made the wheels bang between ±max torque.
///
/// Ground side, every step (from the wheel's collisions):
///   - normal load, contact point/normal, ground collider and its <see cref="GroundSpec"/>;
///   - longitudinal slip ratio and slip angle;
///   - rolling resistance: a torque against the wheel's rotation = coefficient x load x radius.
/// Friction itself is still PhysX Coulomb friction from the PhysicMaterials.
///
/// Next tiers hook in here: a wheel-soil force model (Bekker/Wong) would set the tread's friction to 0 and apply its
/// own traction/lateral/sinkage forces in <see cref="ApplyGroundForces"/> from <see cref="Contact"/>; ruts can
/// subscribe to <see cref="Stepped"/> (contact point, load and slip every physics step).
/// </summary>
[RequireComponent(typeof(ArticulationBody))]
public class DriveWheel : MonoBehaviour
{
    public DriveMotorSpec motor;
    public WheelSpec wheel;
    [Tooltip("°C")]
    public float ambientTemperature = 25f;
    public bool thermalModel = true;
    public bool rollingResistance = true;
    [Tooltip("Balance the wheel like a real one: centre of mass on the axle and the inertia's symmetry axis along the axle. " +
             "Imported URDF inertials can be off/tilted, which makes the wheel wobble: it shakes the rover and hops at speed.")]
    public bool balanceWheel = true;

    /// <summary>Speed target from the drive controller, rad/s in joint space.</summary>
    [NonSerialized] public float TargetSpeed;

    // ---- state (read-only, for the HUD / odometry / ROS) ----
    public float SetpointSpeed { get; private set; }     // rad/s, after the acceleration ramp
    public float Speed { get; private set; }             // rad/s, measured joint speed
    public float OutputTorque { get; private set; }      // N·m at the wheel (joint space sign), applied last step
    public float TorqueAvailable { get; private set; }   // N·m the motor could give this step
    public float MotorCurrent { get; private set; }      // A
    public float MotorVoltage { get; private set; }      // V
    public float WindingTemperature { get; private set; } = 25f;
    public float CurrentLimitNow { get; private set; }   // A, after thermal derating
    public bool Saturated { get; private set; }          // torque-limited this step
    public float Radius { get; private set; }            // m
    public float RollingResistanceTorque { get; private set; }
    public WheelContact Contact;

    /// <summary>After each physics-step update of a wheel (contact data is fresh). For ruts, logging, wheel-soil models.</summary>
    public static event Action<DriveWheel> Stepped;

    public ArticulationBody Body => body;
    public Vector3 AxisWorld => transform.rotation * axisLocal;
    public Vector3 HubWorld => transform.TransformPoint(body.anchorPosition);

    ArticulationBody body, parentBody;
    Transform robotRoot;
    Vector3 axisLocal;
    float integral;   // N·m, PI integral term
    Vector3 lastHub; float lastHubTime = -1; Vector3 hubVelocity;
    float housingTemperature;
    bool warnedNoMotor;

    // contact accumulation (filled by the collision callbacks after each step)
    float contactStamp = -1;
    Vector3 cPointSum, cNormalSum;
    float cImpulse;
    int cCount;
    Collider cGround;
    Rigidbody cGroundRb;
    ArticulationBody cGroundAb;

    void Awake()
    {
        body = GetComponent<ArticulationBody>();
        parentBody = transform.parent != null ? transform.parent.GetComponentInParent<ArticulationBody>() : null;
        robotRoot = transform;
        for (var b = parentBody; b != null; b = b.transform.parent != null ? b.transform.parent.GetComponentInParent<ArticulationBody>() : null)
            robotRoot = b.transform;   // the articulation root (moving body): its forward is the rover's forward
        axisLocal = (body.anchorRotation * Vector3.right).normalized;
        WindingTemperature = housingTemperature = ambientTemperature;
    }

    void Start()
    {
        Radius = wheel != null && wheel.radius > 0 ? wheel.radius : MeasureRadius();
        body.jointFriction = 0;          // losses are modelled explicitly
        body.angularDamping = 0.05f;
        if (balanceWheel) Balance();
        if (motor != null)
        {
            body.maxAngularVelocity = Mathf.Max(body.maxAngularVelocity, motor.NoLoadWheelSpeed * 2f + 10f);
            AddReflectedInertia(motor.ReflectedInertia);
        }
    }

    void Balance()
    {
        Vector3 com = body.centerOfMass, pivot = body.anchorPosition;
        Vector3 off = Vector3.ProjectOnPlane(com - pivot, axisLocal);
        if (off.magnitude > 1e-4f)
        {
            body.centerOfMass = com - off;
            if (off.magnitude > 0.002f)
                Debug.Log($"[DriveWheel] {name}: centre of mass was {off.magnitude * 1000:F0} mm off the axle; moved onto it");
        }
        // Principal axis closest to the axle = the wheel's spin axis. Rebuild the tensor symmetric about the axle.
        Vector3 t = body.inertiaTensor;
        Quaternion r = body.inertiaTensorRotation;
        int k = 0; float best = -1;
        for (int i = 0; i < 3; i++)
        {
            float d = Mathf.Abs(Vector3.Dot(r * (i == 0 ? Vector3.right : i == 1 ? Vector3.up : Vector3.forward), axisLocal));
            if (d > best) { best = d; k = i; }
        }
        float tilt = Mathf.Acos(Mathf.Clamp01(best)) * Mathf.Rad2Deg;
        float axial = t[k], radial = 0.5f * (t[(k + 1) % 3] + t[(k + 2) % 3]);
        if (tilt > 0.5f || Mathf.Abs(t[(k + 1) % 3] - t[(k + 2) % 3]) > 0.02f * radial)
            Debug.Log($"[DriveWheel] {name}: inertia axis was {tilt:F1} deg off the axle; aligned it");
        Vector3 perp = Vector3.Cross(axisLocal, Mathf.Abs(axisLocal.y) < 0.9f ? Vector3.up : Vector3.right).normalized;
        body.inertiaTensorRotation = Quaternion.LookRotation(axisLocal, perp);   // local z = axle
        body.inertiaTensor = new Vector3(radial, radial, axial);
    }

    float MeasureRadius()
    {
        Vector3 hub = HubWorld, axis = AxisWorld;
        float r = 0;
        foreach (var c in GetComponentsInChildren<Collider>())
        {
            if (c is MeshCollider mc && mc.sharedMesh != null)
            {
                foreach (var v in mc.sharedMesh.vertices)
                    r = Mathf.Max(r, Vector3.ProjectOnPlane(mc.transform.TransformPoint(v) - hub, axis).magnitude);
            }
            else if (c is SphereCollider sc) r = Mathf.Max(r, sc.radius * sc.transform.lossyScale.x);
        }
        if (r <= 0) foreach (var c in GetComponentsInChildren<Collider>()) r = Mathf.Max(r, c.bounds.extents.y);
        return r > 0 ? r : 0.1f;
    }

    // Motor rotor + gears spin N times faster than the wheel: they add J x N² to the wheel's inertia about its axle.
    void AddReflectedInertia(float j)
    {
        if (j <= 0) return;
        Vector3 t = body.inertiaTensor;
        Quaternion r = body.inertiaTensorRotation;
        for (int k = 0; k < 3; k++)
        {
            Vector3 e = r * (k == 0 ? Vector3.right : k == 1 ? Vector3.up : Vector3.forward);
            if (Mathf.Abs(Vector3.Dot(e, axisLocal)) > 0.98f)
            {
                t[k] += j;
                body.inertiaTensor = t;
                return;
            }
        }
        Debug.LogWarning($"[DriveWheel] {name}: no principal inertia axis along the axle; motor inertia not added");
    }

    void FixedUpdate()
    {
        if (body.jointVelocity.dofCount == 0) return;
        float dt = Time.fixedDeltaTime;
        Speed = body.jointVelocity[0];
        if (body.driveForce.dofCount > 0) OutputTorque = body.driveForce[0];

        // Hub velocity from positions: ArticulationBody linear velocities read several % high with the TGS solver
        // (measured: 1.05 m/s reported at 0.99 m/s true). Angular velocities are fine.
        Vector3 hub = HubWorld;
        hubVelocity = lastHubTime >= 0 && Time.fixedTime > lastHubTime ? (hub - lastHub) / (Time.fixedTime - lastHubTime) : Vector3.zero;
        lastHub = hub; lastHubTime = Time.fixedTime;
        UpdateContact(dt);

        if (motor == null)
        {
            if (!warnedNoMotor) { warnedNoMotor = true; Debug.LogWarning($"[DriveWheel] {name}: no DriveMotorSpec, wheel is unpowered"); }
            SetDrive(0, 0, 0);
            return;
        }

        UpdateElectrical(dt);

        // Controller: ramp the setpoint, never beyond what the motor can reach.
        float maxStep = motor.maxWheelAcceleration * dt;
        float target = Mathf.Clamp(TargetSpeed, -motor.NoLoadWheelSpeed, motor.NoLoadWheelSpeed);
        SetpointSpeed += Mathf.Clamp(target - SetpointSpeed, -maxStep, maxStep);

        // Torque the motor can deliver in the direction the controller needs.
        float err = SetpointSpeed - Speed;
        float dir = Mathf.Abs(err) > 1e-4f ? Mathf.Sign(err) : (SetpointSpeed != 0 ? Mathf.Sign(SetpointSpeed) : 1f);
        float n = motor.gearRatio;
        float wm = Speed * n;
        float voltLimited = motor.StallMotorTorque * (1f - dir * wm / motor.NoLoadMotorSpeedRad);
        float currentLimited = motor.torqueConstant * CurrentLimitNow;
        float tm = Mathf.Max(0, Mathf.Min(voltLimited, currentLimited));
        bool driving = dir * Speed >= 0;   // torque along the motion: the motor drives the gears; else the load does
        float tOut = driving ? tm * n * motor.gearEfficiency : tm * n / motor.gearEfficiency;
        tOut = Mathf.Min(tOut, motor.gearheadMaxTorque);
        TorqueAvailable = tOut;
        float kp = Mathf.Max(motor.speedLoopGain, 1e-3f);
        if (motor.speedLoopIntegralTime > 0)
        {
            integral += kp / motor.speedLoopIntegralTime * err * dt;
            integral = Mathf.Clamp(integral, -tOut, tOut);                       // anti-windup
        }
        else integral = 0;
        Saturated = Mathf.Abs(kp * err + integral) > tOut * 0.98f;

        SetDrive(SetpointSpeed + integral / kp, kp, tOut);
        ApplyGroundForces(dt);
        Stepped?.Invoke(this);
    }

    void SetDrive(float targetRad, float gainPerRad, float limit)
    {
        var d = body.xDrive;
        d.driveType = ArticulationDriveType.Force;
        d.stiffness = 0;
        d.damping = gainPerRad;                  // N·m per rad/s (measured: Unity passes damping to PhysX as is,
                                                 // only the target velocity is in deg/s)
        d.forceLimit = limit;
        d.targetVelocity = targetRad * Mathf.Rad2Deg;
        body.xDrive = d;
    }

    // Current, voltage and winding temperature from last step's torque.
    void UpdateElectrical(float dt)
    {
        float n = motor.gearRatio;
        bool driving = OutputTorque * Speed >= 0;
        float tMotor = Mathf.Abs(OutputTorque) / n * (driving ? 1f / motor.gearEfficiency : motor.gearEfficiency);
        MotorCurrent = tMotor / motor.torqueConstant + (Mathf.Abs(Speed) > 1e-3f ? motor.noLoadCurrent : 0);
        float backEmf = motor.torqueConstant * Speed * n;
        MotorVoltage = Mathf.Clamp(Mathf.Sign(OutputTorque) * MotorCurrent * motor.terminalResistance + backEmf, -motor.supplyVoltage, motor.supplyVoltage);

        CurrentLimitNow = motor.currentLimit;
        if (!thermalModel || motor.thermalResistanceWinding <= 0 || motor.windingTimeConstant <= 0) return;
        // Two-node model: winding -> housing -> ambient. Copper resistance rises ~0.39 %/K.
        float rw = motor.thermalResistanceWinding, rh = motor.thermalResistanceHousing;
        float cw = motor.windingTimeConstant / rw, ch = motor.motorTimeConstant / Mathf.Max(rh, 1e-3f);
        float loss = MotorCurrent * MotorCurrent * motor.terminalResistance * (1f + 0.0039f * (WindingTemperature - 25f));
        float qwh = (WindingTemperature - housingTemperature) / rw;
        float qha = (housingTemperature - ambientTemperature) / Mathf.Max(rh, 1e-3f);
        WindingTemperature += (loss - qwh) / cw * dt;
        housingTemperature += (qwh - qha) / ch * dt;
        // Controllers protect the motor: derate to the continuous current over the last 20 K.
        float hot = Mathf.InverseLerp(motor.maxWindingTemperature - 20f, motor.maxWindingTemperature, WindingTemperature);
        CurrentLimitNow = Mathf.Lerp(motor.currentLimit, Mathf.Min(motor.nominalCurrent, motor.currentLimit), hot);
    }

    // ---------------- ground ----------------

    void OnCollisionEnter(Collision c) => Accumulate(c);
    void OnCollisionStay(Collision c) => Accumulate(c);

    void Accumulate(Collision c)
    {
        int n = c.contactCount;
        if (n == 0) return;
        if (contactStamp != Time.fixedTime)
        {
            contactStamp = Time.fixedTime;
            cPointSum = cNormalSum = Vector3.zero; cImpulse = 0; cCount = 0; cGround = null;
        }
        Vector3 hub = HubWorld;
        for (int i = 0; i < n; i++)
        {
            var p = c.GetContact(i);
            Vector3 nrm = p.normal;
            if (Vector3.Dot(nrm, hub - p.point) < 0) nrm = -nrm;   // from the ground towards the wheel
            cPointSum += p.point; cNormalSum += nrm; cCount++;
        }
        float imp = c.impulse.magnitude;
        if (cGround == null || imp > 0) { cGround = c.collider; cGroundRb = c.rigidbody; cGroundAb = c.articulationBody; }
        cImpulse += imp;
    }

    void UpdateContact(float dt)
    {
        var k = new WheelContact();
        if (cCount > 0 && Time.fixedTime - contactStamp <= dt * 1.5f)
        {
            k.grounded = true;
            k.point = cPointSum / cCount;
            k.normal = cNormalSum.normalized;
            k.normalForce = cImpulse / dt;
            k.ground = cGround;
            k.groundSpec = cGround != null ? GroundSpecTable.Lookup(cGround.sharedMaterial) : null;

            Vector3 axis = AxisWorld, hub = HubWorld;
            Vector3 groundVel = cGroundRb != null ? cGroundRb.GetPointVelocity(k.point)
                              : cGroundAb != null ? cGroundAb.GetPointVelocity(k.point) : Vector3.zero;
            Vector3 hubVel = hubVelocity - groundVel;
            Vector3 omegaRel = body.angularVelocity - (parentBody != null ? parentBody.angularVelocity : Vector3.zero);
            Vector3 fwd = Vector3.Cross(axis, k.normal).normalized;
            if (Vector3.Dot(fwd, robotRoot.forward) < 0) fwd = -fwd;
            Vector3 side = Vector3.Cross(k.normal, fwd);
            k.forward = fwd;
            k.longitudinalVelocity = Vector3.Dot(hubVel, fwd);
            k.lateralVelocity = Vector3.Dot(hubVel, side);
            // Tread speed relative to the hub (ω x r points backwards at the contact when rolling forwards).
            k.surfaceSpeed = -Vector3.Dot(Vector3.Cross(omegaRel, k.point - hub), fwd);
            float denom = Mathf.Max(Mathf.Max(Mathf.Abs(k.surfaceSpeed), Mathf.Abs(k.longitudinalVelocity)), 0.05f);
            k.slipRatio = (k.surfaceSpeed - k.longitudinalVelocity) / denom;   // + = spinning (traction), - = skidding (braking)
            k.slipAngle = Mathf.Atan2(k.lateralVelocity, Mathf.Max(Mathf.Abs(k.longitudinalVelocity), 0.05f)) * Mathf.Rad2Deg;
        }
        Contact = k;
        cCount = 0;
    }

    void ApplyGroundForces(float dt)
    {
        RollingResistanceTorque = 0;
        if (!rollingResistance || !Contact.grounded || Contact.groundSpec == null) return;
        // Rolling resistance acts against the wheel's rotation relative to its carrier; fades in over ~5 cm/s so a
        // parked wheel doesn't jitter.
        Vector3 axis = AxisWorld;
        Vector3 rel = body.angularVelocity - (parentBody != null ? parentBody.angularVelocity : Vector3.zero);
        float spin = Vector3.Dot(rel, axis);
        float scale = wheel != null ? wheel.rollingResistanceScale : 1f;
        float tau = Contact.groundSpec.rollingResistance * scale * Contact.normalForce * Radius
                    * Mathf.Clamp01(Mathf.Abs(spin) * Radius / 0.05f);
        RollingResistanceTorque = tau;
        if (tau > 0) body.AddTorque(-Mathf.Sign(spin) * tau * axis);
    }
}

[Serializable]
public struct WheelContact
{
    public bool grounded;
    public Vector3 point, normal, forward;
    public float normalForce;          // N
    public Collider ground;
    public GroundSpec groundSpec;
    public float longitudinalVelocity; // m/s of the hub along the ground, wheel-forward
    public float lateralVelocity;      // m/s sideways (scrub during skid turns)
    public float surfaceSpeed;         // m/s of the tread relative to the hub (ω r)
    public float slipRatio;            // (ωr - v) / max(|ωr|, |v|)
    public float slipAngle;            // deg
}
