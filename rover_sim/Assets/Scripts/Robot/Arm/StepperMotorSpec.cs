using UnityEngine;

/// <summary>
/// A stepper motor + driver + gearbox driving one arm axis (or the gripper). Used by <see cref="StepperDrive"/>.
/// Simple model, enough for "does it hold, does it stall, does it skip":
///  - Torque vs lag is the stepper's sine curve: <c>T sin(Nr (field - rotor))</c>, Nr = 90 deg / step angle (50 for
///    1.8 deg). Past half an electrical period the rotor falls into the next pole: steps are lost and the joint does not
///    get them back (the driver keeps counting). Overloaded, it buzzes in place at about its holding torque.
///  - Available torque falls with step rate: flat up to the corner speed, then straight down to
///    <see cref="torqueAtMaxSpeed"/> at <see cref="maxSpeedRpm"/> (pull-out curve, roughly).
///  - Gearbox: ratio and efficiency. Gearbox + link flex is a spring in series with the motor (<see cref="outputStiffness"/>).
/// Edit in the inspector, or live from Esc > ADVANCED > ARM (not saved).
/// </summary>
[CreateAssetMenu(menuName = "SimFlake/Stepper Motor Spec", fileName = "Stepper")]
public class StepperMotorSpec : ScriptableObject
{
    [Header("Motor")]
    [Tooltip("Holding torque at rated current (N·m, motor shaft). NEMA 17: 0.3-0.6, NEMA 23: 0.9-3.")]
    public float holdingTorque = 0.45f;
    [Tooltip("Full-step angle (deg). 1.8 for almost every hybrid stepper, 0.9 for high-resolution ones.")]
    public float stepAngle = 1.8f;
    [Tooltip("Step rate (motor rpm) up to which the full torque is available.")]
    public float cornerSpeedRpm = 150f;
    [Tooltip("Fastest the driver steps the motor (motor rpm). Commands above it are capped.")]
    public float maxSpeedRpm = 600f;
    [Tooltip("Fraction of the holding torque still available at max speed (pull-out torque).")]
    [Range(0f, 1f)] public float torqueAtMaxSpeed = 0.3f;
    [Tooltip("Detent torque (N·m, motor shaft): what holds the joint with the driver off (e.g. a broken-off piece).")]
    public float detentTorque = 0.02f;

    [Header("Driver")]
    [Tooltip("Run current as a fraction of the rated current (torque scales with it).")]
    [Range(0f, 1.5f)] public float runCurrent = 1f;
    [Tooltip("Current while standing still (fraction of rated). Many drivers drop to ~50 % after 0.5 s; 1 = no reduction.")]
    [Range(0f, 1.5f)] public float holdCurrent = 1f;
    [Tooltip("Step-rate ramp of the driver (motor rpm per second).")]
    public float accelerationRpmPerSec = 1500f;

    [Header("Gearbox and structure")]
    [Tooltip("Motor turns per output turn.")]
    public float gearRatio = 50f;
    [Tooltip("Gearbox efficiency (planetary 0.7-0.9, worm / lead screw 0.3-0.5).")]
    [Range(0.05f, 1f)] public float gearEfficiency = 0.8f;
    [Tooltip("Stiffness of gearbox + link at the output (N·m per degree), in series with the motor. Lower = the arm flexes " +
             "more under load. For the gripper the finger flex (ArmController) is used instead.")]
    public float outputStiffness = 60f;
    [Tooltip("Damping of the drive as a fraction of critical (motor + structure).")]
    [Range(0f, 2f)] public float dampingRatio = 0.6f;

    [TextArea(2, 6)] public string notes;

    // ---- derived (output side) ----

    /// <summary>Electrical radians per output radian.</summary>
    public float ElectricalPerOutputRad => 90f / Mathf.Max(stepAngle, 1e-3f) * Mathf.Max(gearRatio, 1e-3f);
    /// <summary>Holding torque at the output with run current (N·m).</summary>
    public float OutputHoldingTorque => holdingTorque * runCurrent * gearRatio * gearEfficiency;
    /// <summary>Top output speed (deg/s).</summary>
    public float MaxOutputDegPerSec => maxSpeedRpm * 6f / Mathf.Max(gearRatio, 1e-3f);
    /// <summary>Output angle of one full step (deg).</summary>
    public float OutputStepDeg => stepAngle / Mathf.Max(gearRatio, 1e-3f);

    /// <summary>Torque fraction available at a motor speed (rpm), 0..1, before current scaling.</summary>
    public float SpeedFactor(float motorRpm)
    {
        motorRpm = Mathf.Abs(motorRpm);
        if (motorRpm <= cornerSpeedRpm || maxSpeedRpm <= cornerSpeedRpm) return 1f;
        float t = Mathf.Clamp01((motorRpm - cornerSpeedRpm) / (maxSpeedRpm - cornerSpeedRpm));
        return Mathf.Lerp(1f, torqueAtMaxSpeed, t);
    }
}
