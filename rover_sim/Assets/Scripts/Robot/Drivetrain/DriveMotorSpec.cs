using UnityEngine;

/// <summary>
/// One drive motor + gearhead + motor controller, as data. Used by <see cref="DriveWheel"/>.
///
/// Motor values are at the MOTOR SHAFT (as in a maxon-style datasheet). The gearhead turns them into wheel torque/speed.
/// Model: a DC motor (brushed or BLDC with a speed controller):
///   - at full voltage the torque falls linearly from stall torque (0 rpm) to 0 at no-load speed;
///   - the controller caps the current at <see cref="currentLimit"/>, which caps the torque at kt x I;
///   - the gearhead multiplies torque by ratio x efficiency and caps it at its own rating.
/// The controller follows a speed target with an acceleration ramp (like the real drivers do).
/// Create new ones with Assets > Create > SimFlake > Drive Motor.
/// </summary>
[CreateAssetMenu(menuName = "SimFlake/Drive Motor", fileName = "DriveMotor")]
public class DriveMotorSpec : ScriptableObject
{
    [TextArea(2, 6)] public string notes;

    [Header("Motor (at the motor shaft, nominal voltage)")]
    public float nominalVoltage = 24f;
    [Tooltip("rpm with no load at nominal voltage")]
    public float noLoadSpeedRpm = 8050f;
    [Tooltip("A, drawn with no load (friction inside the motor)")]
    public float noLoadCurrent = 0.363f;
    [Tooltip("N·m at 0 rpm and nominal voltage (without the current limit)")]
    public float stallTorque = 1.11f;
    [Tooltip("N·m/A")]
    public float torqueConstant = 0.0282f;
    [Tooltip("Ohm, terminal resistance (phase-to-phase for BLDC)")]
    public float terminalResistance = 0.233f;
    [Tooltip("A, max continuous current (datasheet 'nominal current'). Shown in the HUD as the thermal reference.")]
    public float nominalCurrent = 7.31f;
    [Tooltip("g·cm², rotor inertia")]
    public float rotorInertia = 86.2f;

    [Header("Gearhead")]
    public float gearRatio = 26f;
    [Range(0.3f, 1f)] public float gearEfficiency = 0.81f;
    [Tooltip("N·m at the output; the controller never asks for more (the gearhead's intermittent rating)")]
    public float gearheadMaxTorque = 11.3f;
    [Tooltip("g·cm², gearhead inertia at its input")]
    public float gearheadInertia = 15f;

    [Header("Thermal (0 = no thermal model)")]
    [Tooltip("K/W winding to housing")]
    public float thermalResistanceWinding = 3.04f;
    [Tooltip("K/W housing to ambient")]
    public float thermalResistanceHousing = 1.66f;
    [Tooltip("s, thermal time constant of the winding")]
    public float windingTimeConstant = 20.2f;
    [Tooltip("s, thermal time constant of the motor")]
    public float motorTimeConstant = 92.3f;
    [Tooltip("°C. The controller derates the current to the nominal current as the winding approaches this.")]
    public float maxWindingTemperature = 155f;

    [Header("Motor controller")]
    [Tooltip("A, the controller's peak current limit")]
    public float currentLimit = 20f;
    [Tooltip("V, supply (battery) voltage available to the controller")]
    public float supplyVoltage = 24f;
    [Tooltip("rad/s² at the WHEEL. The controller ramps its speed setpoint this fast.")]
    public float maxWheelAcceleration = 50f;
    [Tooltip("N·m per rad/s at the WHEEL: proportional gain of the speed loop. Higher = stiffer; too high makes the wheels " +
             "chatter against the ground. A response time of ~10-20 ms: gain ≈ (wheel + reflected + share of rover inertia) / 0.015 s.")]
    public float speedLoopGain = 40f;
    [Tooltip("s, integral time of the speed loop (PI). 0 = P only (default). Real controllers are PI, but PhysX contacts have " +
             "no tyre compliance: six integrators on rigidly coupled wheels wind up against each other (measured on chassis_2024: " +
             "3-4 N·m per wheel of internal fighting on flat ground, vs 1.4 P-only). With P only the rover still holds on 30 deg.")]
    public float speedLoopIntegralTime = 0f;

    // ---- derived (all at the wheel unless noted) ----
    public float NoLoadMotorSpeedRad => noLoadSpeedRpm * Mathf.PI / 30f * (supplyVoltage / Mathf.Max(nominalVoltage, 1e-3f));
    public float NoLoadWheelSpeed => NoLoadMotorSpeedRad / gearRatio;                                                  // rad/s
    public float StallMotorTorque => stallTorque * (supplyVoltage / Mathf.Max(nominalVoltage, 1e-3f));                  // N·m at motor
    public float ReflectedInertia => (rotorInertia + gearheadInertia) * 1e-7f * gearRatio * gearRatio;                  // kg·m² at wheel
    public float PeakWheelTorque => Mathf.Min(Mathf.Min(StallMotorTorque, torqueConstant * currentLimit) * gearRatio * gearEfficiency, gearheadMaxTorque);
    public float ContinuousWheelTorque => Mathf.Min(torqueConstant * nominalCurrent * gearRatio * gearEfficiency, gearheadMaxTorque);

    public string Summary =>
        $"no-load {NoLoadWheelSpeed:F1} rad/s ({NoLoadWheelSpeed * 30 / Mathf.PI:F0} rpm) at the wheel, peak {PeakWheelTorque:F2} N·m, " +
        $"continuous {ContinuousWheelTorque:F2} N·m, reflected inertia {ReflectedInertia * 1000:F2} g·m²";

    void OnValidate()
    {
        gearRatio = Mathf.Max(gearRatio, 1e-3f);
        torqueConstant = Mathf.Max(torqueConstant, 1e-6f);
        terminalResistance = Mathf.Max(terminalResistance, 1e-6f);
    }
}
