using System;
using System.IO;
using UnityEditor;
using UnityEngine;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// Sets up the arm: stepper motor specs (Assets/Robots/Arm/Motors), the gripper pad material, the Arm prefab
    /// (gravity on, sane masses for the URDF's placeholder links, motors, breakage), and the arm mount + payload on
    /// chassis_2024. Existing spec assets are kept (tune them in the inspector; delete one to get these values back).
    /// Menu: SimFlake > Arm > Set Up Arm, or -executeMethod SimFlake.EditorTools.ArmSetup.SetUpFromCommandLine.
    /// </summary>
    public static class ArmSetup
    {
        const string Root = "Assets/Robots/Arm";
        const string ArmPrefab = Root + "/Arm.prefab";
        const string ChassisPrefab = "Assets/Robots/Chassis/chassis_2024.prefab";
        /// <summary>Arm base (link_0) on chassis_link: on the deck (top at y = 0.01), centred, ahead of the middle.</summary>
        public static readonly Vector3 MountPosition = new Vector3(0f, 0.01f, 0.10f);

        [MenuItem("SimFlake/Arm/Set Up Arm (motors, gripper, breakage, chassis_2024 mount)", priority = 260)]
        public static void SetUpMenu() { if (SetUp()) Debug.Log("[ArmSetup] done"); }

        public static void SetUpFromCommandLine()
        {
            bool ok = false;
            try { ok = SetUp(); }
            catch (Exception e) { Debug.LogException(e); }
            EditorApplication.Exit(ok ? 0 : 1);
        }

        struct Motor
        {
            public string file, notes;
            public float torque, corner, max, atMax, ratio, eff, stiff, detent;
        }

        // Estimates: the old arm's steppers and gearboxes are not documented here. Sized so the output torques match
        // typical NEMA 17 / 23 + planetary builds and carry the URDF masses with margin. Edit to match the real arm.
        static readonly Motor[] Motors =
        {
            new Motor { file = "Stepper_Base_NEMA23_50to1", torque = 1.26f, corner = 300, max = 375, atMax = 0.6f, ratio = 50, eff = 0.8f, stiff = 60, detent = 0.04f,
                        notes = "Axis 1 (base yaw). Estimate: NEMA 23 (1.26 N·m) + 50:1 planetary -> ~50 N·m, 45 deg/s." },
            new Motor { file = "Stepper_Shoulder_NEMA23_100to1", torque = 1.9f, corner = 300, max = 750, atMax = 0.4f, ratio = 100, eff = 0.8f, stiff = 100, detent = 0.06f,
                        notes = "Axis 2 (shoulder). Estimate: NEMA 23 (1.9 N·m) + 100:1 -> ~150 N·m (arm alone needs ~20 N·m held straight out), 45 deg/s." },
            new Motor { file = "Stepper_Elbow_NEMA23_50to1", torque = 1.26f, corner = 300, max = 375, atMax = 0.6f, ratio = 50, eff = 0.8f, stiff = 50, detent = 0.04f,
                        notes = "Axis 3 (elbow). Estimate: NEMA 23 (1.26 N·m) + 50:1 -> ~50 N·m (forearm needs ~7 N·m), 45 deg/s." },
            new Motor { file = "Stepper_ForearmRoll_NEMA17_27to1", torque = 0.45f, corner = 300, max = 202, atMax = 0.6f, ratio = 26.85f, eff = 0.8f, stiff = 15, detent = 0.015f,
                        notes = "Axis 4 (forearm roll). Estimate: NEMA 17 (0.45 N·m) + 26.85:1 planetary -> ~10 N·m, 45 deg/s." },
            new Motor { file = "Stepper_Wrist_NEMA17_50to1", torque = 0.45f, corner = 300, max = 375, atMax = 0.5f, ratio = 50, eff = 0.8f, stiff = 15, detent = 0.015f,
                        notes = "Axis 5 (wrist pitch). Estimate: NEMA 17 (0.45 N·m) + 50:1 -> ~18 N·m, 45 deg/s." },
            new Motor { file = "Stepper_WristRoll_NEMA17_14to1", torque = 0.45f, corner = 300, max = 105, atMax = 0.8f, ratio = 13.73f, eff = 0.85f, stiff = 8, detent = 0.015f,
                        notes = "Axis 6 (wrist roll). Estimate: NEMA 17 (0.45 N·m) + 13.73:1 -> ~5 N·m, 45 deg/s." },
            new Motor { file = "Stepper_Gripper_NEMA17_10to1", torque = 0.45f, corner = 300, max = 360, atMax = 0.5f, ratio = 10, eff = 0.5f, stiff = 15, detent = 0.015f,
                        notes = "End effector. Estimate: NEMA 17 (0.45 N·m) + 10:1 into the finger drive (0.5 mm of finger travel per output degree, " +
                                "efficiency 0.5 for the screw/rack) -> ~40 N squeeze per finger at stall, full stroke in ~1 s. " +
                                "Output stiffness is not used here: the finger flex is ArmController.fingerStiffness." },
        };

        public static bool SetUp()
        {
            var specs = EnsureMotors();
            var pad = EnsurePadMaterial();
            bool ok = SetUpArmPrefab(specs, pad);
            ok &= SetUpChassisMount();
            AssetDatabase.SaveAssets();
            return ok;
        }

        static StepperMotorSpec[] EnsureMotors()
        {
            string dir = Root + "/Motors";
            if (!AssetDatabase.IsValidFolder(dir)) AssetDatabase.CreateFolder(Root, "Motors");
            var specs = new StepperMotorSpec[Motors.Length];
            for (int i = 0; i < Motors.Length; i++)
            {
                var m = Motors[i];
                string path = $"{dir}/{m.file}.asset";
                var s = AssetDatabase.LoadAssetAtPath<StepperMotorSpec>(path);
                if (s == null)
                {
                    s = ScriptableObject.CreateInstance<StepperMotorSpec>();
                    s.holdingTorque = m.torque; s.cornerSpeedRpm = m.corner; s.maxSpeedRpm = m.max; s.torqueAtMaxSpeed = m.atMax;
                    s.gearRatio = m.ratio; s.gearEfficiency = m.eff; s.outputStiffness = m.stiff; s.detentTorque = m.detent;
                    s.notes = m.notes;
                    AssetDatabase.CreateAsset(s, path);
                    Debug.Log($"[ArmSetup] created {path}: {s.OutputHoldingTorque:0.0} N·m, {s.MaxOutputDegPerSec:0} deg/s at the output");
                }
                specs[i] = s;
            }
            return specs;
        }

        static PhysicMaterial EnsurePadMaterial()
        {
            string path = Root + "/GripperPad.physicMaterial";
            var pad = AssetDatabase.LoadAssetAtPath<PhysicMaterial>(path);
            if (pad != null) return pad;
            // Rubber-faced printed fingers. Maximum combine: the pad's grip counts whatever the object is made of.
            pad = new PhysicMaterial("GripperPad")
            {
                staticFriction = 1.0f, dynamicFriction = 0.9f, bounciness = 0f,
                frictionCombine = PhysicMaterialCombine.Maximum, bounceCombine = PhysicMaterialCombine.Minimum,
            };
            AssetDatabase.CreateAsset(pad, path);
            return pad;
        }

        static void SetInertia(ArticulationBody b, float mass, Vector3 inertia)
        {
            b.mass = mass;
            b.automaticCenterOfMass = false;
            b.centerOfMass = Vector3.zero;
            var auto = typeof(ArticulationBody).GetProperty("automaticInertiaTensor");
            if (auto != null && auto.CanWrite) auto.SetValue(b, false);
            b.inertiaTensor = inertia;
            b.inertiaTensorRotation = Quaternion.identity;
        }

        static bool SetUpArmPrefab(StepperMotorSpec[] specs, PhysicMaterial pad)
        {
            var root = PrefabUtility.LoadPrefabContents(ArmPrefab);
            try
            {
                var ac = root.GetComponent<ArmController>();
                if (ac == null) { Debug.LogError("[ArmSetup] Arm.prefab has no ArmController"); return false; }
                ac.motors = specs;
                foreach (var b in root.GetComponentsInChildren<ArticulationBody>(true))
                {
                    b.useGravity = true;
                    b.angularDamping = 0.05f;
                    b.linearDamping = 0.05f;
                    b.jointFriction = 0.05f;
                    var d = b.xDrive;
                    if (b.jointType == ArticulationJointType.RevoluteJoint || b.jointType == ArticulationJointType.PrismaticJoint)
                    {
                        d.driveType = ArticulationDriveType.Force; d.stiffness = 0; d.damping = 0; d.targetVelocity = 0;
                        b.xDrive = d;
                    }
                    // The URDF has no inertial for these two fixed frames: Unity gave them 1 kg and 1 kg·m², which made
                    // the wrist ~5000x harder to turn than its own parts.
                    if (b.name == "ee_base_link") SetInertia(b, 0.05f, new Vector3(2e-5f, 2e-5f, 2e-5f));
                    if (b.name == "link_tt") SetInertia(b, 0.005f, new Vector3(1e-6f, 1e-6f, 1e-6f));
                }
                foreach (var f in new[] { "finger_left", "finger_right" })
                {
                    var t = Find(root.transform, f);
                    if (t == null) { Debug.LogError($"[ArmSetup] {f} not found"); return false; }
                    foreach (var c in t.GetComponentsInChildren<Collider>(true))
                        if (c.attachedArticulationBody == t.GetComponent<ArticulationBody>()) c.sharedMaterial = pad;
                }
                var brk = root.GetComponent<ArmBreakage>();
                if (brk == null) brk = root.AddComponent<ArmBreakage>();
                // Strengths live in ArmBreakage.DefaultLinks (measured, see there); tune live in Esc > ADVANCED > ARM.
                brk.links = ArmBreakage.DefaultLinks();
                brk.filterTime = 0.1f;
                int layer = ArmController.ArmLayer;
                foreach (var t in root.GetComponentsInChildren<Transform>(true)) t.gameObject.layer = layer;
                Physics.IgnoreLayerCollision(layer, layer, true);
                PrefabUtility.SaveAsPrefabAsset(root, ArmPrefab);
                Debug.Log("[ArmSetup] Arm.prefab: stepper motors, gravity on, gripper pads, breakage");
                return true;
            }
            finally { PrefabUtility.UnloadPrefabContents(root); }
        }

        static bool SetUpChassisMount()
        {
            var arm = AssetDatabase.LoadAssetAtPath<GameObject>(ArmPrefab);
            var root = PrefabUtility.LoadPrefabContents(ChassisPrefab);
            try
            {
                var chassis = Find(root.transform, "chassis_link");
                if (chassis == null) { Debug.LogError("[ArmSetup] chassis_link not found in chassis_2024"); return false; }
                var mount = chassis.Find("ArmMount");
                if (mount == null)
                {
                    mount = new GameObject("ArmMount").transform;
                    mount.SetParent(chassis, false);
                    mount.localPosition = MountPosition;
                    mount.localRotation = Quaternion.identity;
                }
                var p = root.GetComponent<RoverPayload>();
                if (p == null) p = root.AddComponent<RoverPayload>();
                p.armMount = mount;
                p.armPrefab = arm;
                PrefabUtility.SaveAsPrefabAsset(root, ChassisPrefab);
                Debug.Log($"[ArmSetup] chassis_2024: ArmMount at {mount.localPosition} on chassis_link, payload = {p.configuration}");
                return true;
            }
            finally { PrefabUtility.UnloadPrefabContents(root); }
        }

        static Transform Find(Transform root, string name)
        {
            foreach (var t in root.GetComponentsInChildren<Transform>(true)) if (t.name == name) return t;
            return null;
        }
    }
}
