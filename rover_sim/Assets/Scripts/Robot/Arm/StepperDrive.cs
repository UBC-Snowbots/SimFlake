using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// One stepper motor (<see cref="StepperMotorSpec"/>) driving one or more joints through a flexible transmission:
/// one revolute arm joint, or both gripper fingers (one motor, rack/lead screw, fingers coupled).
///
/// Every physics step:
///  1. The driver advances the field angle by the step rate (ramped, capped at max speed).
///  2. The rotor (massless, overdamped) settles where the stepper torque <c>T sin(N (field - rotor))</c> balances the flex
///     springs to the joints. When the load needs more than the motor has, the rotor falls back a pole: steps are lost
///     (<see cref="lostSteps"/>), the field keeps going, the motor buzzes at its limit. That is the stall.
///  3. Each joint gets the flex spring's force through its PhysX drive. The drive's own (implicit) stiffness only
///     resists the change within one step (target = predicted position + force / stiffness), which keeps stiff gearboxes
///     stable at 5 ms. Damping is relative to the rotor.
/// Stopping the command leaves the field where it is, so a gripper closed on something keeps squeezing it with
/// whatever force the flex had built up (up to the stall force), without anything being commanded.
/// </summary>
public class StepperDrive
{
    public class Output
    {
        public ArticulationBody body;
        public float sign = 1;      // joint coordinate = sign * ratio * output angle
        public float ratio = 1;     // 1 for a revolute joint (rad/rad), metres per radian for a finger
        public float stiffness;     // flex: N·m/rad (revolute) or N/m (finger)
        public float force;         // last flex force applied (N·m or N, + = along the joint's positive direction)
        internal ArticulationBody[] subtree;
        public bool Prismatic => body.jointType == ArticulationJointType.PrismaticJoint;
        public double Q => body.jointPosition.dofCount > 0 ? body.jointPosition[0] : 0;
        public double Qd => body.jointVelocity.dofCount > 0 ? body.jointVelocity[0] : 0;
    }

    public readonly string name;
    public StepperMotorSpec spec;
    public readonly List<Output> outputs = new List<Output>();

    public double field;        // commanded (field) output angle, rad, axis frame
    public double fieldVel;     // rad/s
    public double rotor;        // rotor angle at the output, rad
    public double rotorVel;
    public long lostSteps;
    public bool powered = true;
    /// <summary>Torque at the output now (N·m) and the most it could give now.</summary>
    public float torque, available;
    public float lastSlipTime = -100f;
    public bool Slipping => Time.time - lastSlipTime < 0.25f;

    float stillTime;
    bool started;

    public StepperDrive(string name, StepperMotorSpec spec) { this.name = name; this.spec = spec; }

    public Output Add(ArticulationBody body, float sign, float ratio, float stiffness)
    {
        var o = new Output { body = body, sign = sign, ratio = ratio, stiffness = stiffness };
        outputs.Add(o);
        RefreshSubtree(o);
        return o;
    }

    public void RefreshSubtree()
    {
        foreach (var o in outputs) RefreshSubtree(o);
    }

    static void RefreshSubtree(Output o)
    {
        var all = o.body.GetComponentsInChildren<ArticulationBody>();
        o.subtree = Array.FindAll(all, b => b.isActiveAndEnabled);
    }

    /// <summary>Output angle (rad, axis frame) from the joints: the mean over the outputs.</summary>
    public double Position
    {
        get
        {
            if (outputs.Count == 0) return 0;
            double s = 0;
            foreach (var o in outputs) s += o.sign * o.Q / o.ratio;
            return s / outputs.Count;
        }
    }

    /// <summary>Field and rotor to where the joints are, no speed (start, re-power).</summary>
    public void Reset()
    {
        field = rotor = Position;
        fieldVel = rotorVel = 0;
        started = true;
    }

    /// <summary>One physics step. <paramref name="desiredVel"/>: output speed wanted (rad/s, axis frame).</summary>
    public void Step(double desiredVel, float dt)
    {
        if (spec == null || outputs.Count == 0 || dt <= 0) return;
        if (!started) Reset();

        float G = Mathf.Max(spec.gearRatio, 1e-3f);
        double rpmToOut = 2 * Math.PI / 60.0 / G;
        double maxV = spec.maxSpeedRpm * rpmToOut;
        double acc = spec.accelerationRpmPerSec * rpmToOut;
        if (!powered || double.IsNaN(desiredVel)) desiredVel = 0;
        desiredVel = Math.Max(-maxV, Math.Min(maxV, desiredVel));
        double dv = desiredVel - fieldVel;
        fieldVel += Math.Max(-acc * dt, Math.Min(acc * dt, dv));
        field += fieldVel * dt;

        stillTime = Math.Abs(fieldVel) < 1e-6 ? stillTime + dt : 0;
        float current = !powered ? 0 : stillTime > 0.5f ? spec.holdCurrent : spec.runCurrent;
        float motorRpm = (float)(Math.Abs(fieldVel) / rpmToOut);
        float T = spec.holdingTorque * current * spec.SpeedFactor(motorRpm) * G * spec.gearEfficiency;
        available = T;
        double N = spec.ElectricalPerOutputRad;

        // Flex springs seen from the rotor: torque = Ksum * rotor - Q.
        double Ksum = 0, Q = 0;
        foreach (var o in outputs)
        {
            double r = o.ratio;
            Ksum += r * r * o.stiffness;
            Q += r * o.stiffness * o.sign * o.Q;
        }

        double prevRotor = rotor;
        if (T > 0 && Ksum > 0)
        {
            // Overdamped rotor: relax to the stable balance nearest to where it was (steps never exceed the max slope).
            double L = T * N + Ksum, phi = rotor + fieldVel * dt;
            for (int it = 0; it < 40; it++)
            {
                double h = T * Math.Sin(N * (field - phi)) - (Ksum * phi - Q);
                phi += h / L;
                if (Math.Abs(h) < 1e-6 * (T + 1)) break;
            }
            rotor = phi;
        }
        else rotor = Ksum > 0 ? Q / Ksum : Position; // driver off: nothing holds the rotor

        // Lost steps: the rotor sits more than half an electrical period from the field. Fold the field back (the torque
        // curve is periodic) and count them: 4 full steps per period.
        double e = N * (field - rotor);
        long p = (long)Math.Round(e / (2 * Math.PI));
        if (p != 0 && T > 0)
        {
            field -= p * 2 * Math.PI / N;
            lostSteps += 4 * Math.Abs(p);
            lastSlipTime = Time.time;
        }
        torque = T > 0 ? (float)(T * Math.Sin(N * (field - rotor))) : 0;
        rotorVel = Mathf.Lerp((float)rotorVel, (float)((rotor - prevRotor) / dt), 0.5f);

        foreach (var o in outputs) Apply(o, T, N, dt);
    }

    void Apply(Output o, float T, double N, float dt)
    {
        var b = o.body;
        if (b == null || b.jointPosition.dofCount == 0) return;
        double r = o.ratio, q = o.Q, qd = o.Qd;
        float J = Inertia(o);
        var d = b.xDrive;
        d.driveType = ArticulationDriveType.Force;
        double f, kd, limit;
        if (T > 0)
        {
            f = o.stiffness * (o.sign * r * rotor - q);
            double kMotor = T * N / (r * r);       // motor stiffness seen at this joint
            kd = 1.0 / (1.0 / Math.Max(o.stiffness, 1e-6) + 1.0 / kMotor);
            limit = T / r;
        }
        else
        {
            // Unpowered: detent torque acts like a brake, nothing drives it.
            f = 0; kd = 0;
            limit = spec.detentTorque * spec.gearRatio / r;
        }
        f = Math.Max(-limit, Math.Min(limit, f));
        o.force = (float)f;
        double c = 2 * spec.dampingRatio * Math.Sqrt(Math.Max(kd, o.stiffness * 0.01) * J);
        double target = q + qd * dt + (kd > 0 ? f / kd : 0);
        double tv = T > 0 ? o.sign * r * rotorVel : 0;
        bool rev = !o.Prismatic;
        d.stiffness = (float)kd;
        d.damping = (float)c;
        d.target = (float)(rev ? target * Mathf.Rad2Deg : target);
        d.targetVelocity = (float)(rev ? tv * Mathf.Rad2Deg : tv);
        d.forceLimit = (float)limit;
        b.xDrive = d;
    }

    /// <summary>What the joint moves: subtree inertia about a revolute axis, or subtree mass for a finger.</summary>
    static float Inertia(Output o)
    {
        var b = o.body;
        if (o.subtree == null) return Mathf.Max(b.mass, 1e-3f);
        if (o.Prismatic)
        {
            float m = 0;
            foreach (var s in o.subtree) if (s != null) m += s.mass;
            return Mathf.Max(m, 1e-3f);
        }
        Vector3 p = b.transform.TransformPoint(b.anchorPosition);
        Vector3 axis = (b.transform.rotation * b.anchorRotation * Vector3.right).normalized;
        float J = 0;
        foreach (var s in o.subtree)
        {
            if (s == null) continue;
            Quaternion R = s.transform.rotation * s.inertiaTensorRotation;
            Vector3 a = Quaternion.Inverse(R) * axis;
            Vector3 I = s.inertiaTensor;
            J += a.x * a.x * I.x + a.y * a.y * I.y + a.z * a.z * I.z;
            Vector3 rc = s.worldCenterOfMass - p;
            J += s.mass * (rc - Vector3.Dot(rc, axis) * axis).sqrMagnitude;
        }
        return Mathf.Max(J, 1e-5f);
    }
}
