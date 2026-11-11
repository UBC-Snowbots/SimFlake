using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// ONE-OFF MIGRATION (already run). Don't use it to add new robots; see "Adding a robot" in CLAUDE.md.
    /// Turns the rover and arm that were copied into each scene into shared prefabs.
    ///   -executeMethod SimFlake.EditorTools.RobotPrefabs.MakeChassisPrefab [-dryrun]
    ///   -executeMethod SimFlake.EditorTools.RobotPrefabs.MakeArmPrefab [-dryrun]
    /// The chassis prefab is made from WaypointTask's rover. In the other scenes the old rover is replaced by an
    /// instance; every setting that differed is kept as a prefab override and logged ("[Prefab] keep ...").
    /// </summary>
    public static class RobotPrefabs
    {
        public const string ChassisPrefabPath = "Assets/Robots/Chassis/chassis_2024.prefab";
        public const string ArmPrefabPath = "Assets/Robots/Arm/Arm.prefab";
        const string ChassisSourceScene = "Assets/Scenes/TaskWorlds/WaypointTask.unity";
        static readonly string[] ChassisOtherScenes = { "Assets/Scenes/ChassisPlayground.unity", "Assets/Scenes/SampleScene.unity" };
        const string ArmScene = "Assets/Scenes/ArmScene.unity";

        // Sensors that should exist once per rover. A copy outside the rover publishing the same topic gets disabled.
        static readonly Type[] SingleSensors = {
            typeof(HorizontalLidarScanner), typeof(RGBCameraCapture), typeof(CompressedCameraCapture),
            typeof(ImuSensor), typeof(LocationStuff), typeof(MotorController),
        };

        static bool DryRun => Environment.GetCommandLineArgs().Contains("-dryrun");

        public static void MakeChassisPrefab()
        {
            bool ok = true;
            try
            {
                // 1. Source scene: the rover becomes the prefab.
                var scene = EditorSceneManager.OpenScene(ChassisSourceScene);
                var rover = FindRoot(scene, "chassis_24");
                Debug.Log($"[Prefab] === {ChassisSourceScene}");
                if (PrefabUtility.IsPartOfPrefabInstance(rover))
                    throw new Exception("rover in the source scene is already a prefab instance; nothing to do");
                AdoptGps(scene, rover);
                // Every scene's rover root carries an empty component whose script no longer exists (unknown GUID,
                // no fields). Prefabs can't be saved with missing scripts, so drop it.
                foreach (var t in rover.GetComponentsInChildren<Transform>(true))
                {
                    int removed = GameObjectUtility.RemoveMonoBehavioursWithMissingScript(t.gameObject);
                    if (removed > 0) Debug.Log($"[Prefab] removed {removed} missing script(s) from {GetPath(t)}");
                }
                if (!DryRun)
                {
                    Directory.CreateDirectory(Path.GetDirectoryName(ChassisPrefabPath));
                    PrefabUtility.SaveAsPrefabAssetAndConnect(rover, ChassisPrefabPath, InteractionMode.AutomatedAction, out bool saved);
                    if (!saved) throw new Exception("SaveAsPrefabAssetAndConnect failed");
                    EditorSceneManager.SaveScene(scene);
                    Debug.Log($"[Prefab] saved {ChassisPrefabPath}");
                }

                // 2. Other scenes: swap the copy for an instance.
                foreach (var path in ChassisOtherScenes)
                {
                    scene = EditorSceneManager.OpenScene(path);
                    Debug.Log($"[Prefab] === {path}");
                    var old = FindRoot(scene, "chassis_24");
                    if (PrefabUtility.IsPartOfPrefabInstance(old)) { Debug.Log("[Prefab] already an instance, skipped"); continue; }
                    if (DryRun) { Debug.Log("[Prefab] dry run: would replace " + old.name); continue; }
                    ReplaceWithInstance(scene, old, AssetDatabase.LoadAssetAtPath<GameObject>(ChassisPrefabPath));
                    EditorSceneManager.SaveScene(scene);
                }
            }
            catch (Exception e) { ok = false; Debug.LogError("[Prefab] FAILED: " + e); }
            if (Application.isBatchMode) EditorApplication.Exit(ok ? 0 : 1);
        }

        public static void MakeArmPrefab()
        {
            bool ok = true;
            try
            {
                var scene = EditorSceneManager.OpenScene(ArmScene);
                var arm = FindRoot(scene, "dev_arm");
                if (PrefabUtility.IsPartOfPrefabInstance(arm)) throw new Exception("dev_arm is already a prefab instance");
                Debug.Log($"[Prefab] arm components: {string.Join(", ", arm.GetComponents<MonoBehaviour>().Select(m => m.GetType().Name))}");
                if (!DryRun)
                {
                    Directory.CreateDirectory(Path.GetDirectoryName(ArmPrefabPath));
                    PrefabUtility.SaveAsPrefabAssetAndConnect(arm, ArmPrefabPath, InteractionMode.AutomatedAction, out bool saved);
                    if (!saved) throw new Exception("SaveAsPrefabAssetAndConnect failed");
                    EditorSceneManager.SaveScene(scene);
                    Debug.Log($"[Prefab] saved {ArmPrefabPath}");
                }
            }
            catch (Exception e) { ok = false; Debug.LogError("[Prefab] FAILED: " + e); }
            if (Application.isBatchMode) EditorApplication.Exit(ok ? 0 : 1);
        }

        static GameObject FindRoot(Scene scene, string name)
        {
            var found = scene.GetRootGameObjects().Where(g => g.name == name).ToArray();
            if (found.Length != 1) throw new Exception($"expected one root '{name}' in {scene.path}, found {found.Length}");
            return found[0];
        }

        /// <summary>The moving body: the root ArticulationBody (chassis_link).</summary>
        static Transform Body(GameObject rover) =>
            rover.GetComponentsInChildren<ArticulationBody>(true)
                 .First(a => a.transform.parent == null || a.transform.parent.GetComponentInParent<ArticulationBody>() == null).transform;

        /// <summary>
        /// GPS becomes part of the rover: a "GPS" child of chassis_link. A GPS component elsewhere in the scene
        /// (the old LocationManager) hands over its settings (origin, noise, debug text) and is removed.
        /// </summary>
        static void AdoptGps(Scene scene, GameObject rover)
        {
            var body = Body(rover);
            var gps = rover.GetComponentsInChildren<LocationStuff>(true).FirstOrDefault();
            if (gps == null)
            {
                var go = new GameObject("GPS");
                go.transform.SetParent(body, false);
                gps = go.AddComponent<LocationStuff>();
                Debug.Log($"[Prefab] added GPS under {GetPath(body)}");
            }
            foreach (var ext in AllInScene<LocationStuff>(scene).Where(g => !g.transform.IsChildOf(rover.transform)))
            {
                Debug.Log($"[Prefab] GPS settings taken from {GetPath(ext.transform)} (origin {ext.startLatitude}, {ext.startLongitude}); component removed");
                EditorUtility.CopySerialized(ext, gps);
                Object.DestroyImmediate(ext);
            }
            gps.object_to_track = body;
            EditorUtility.SetDirty(gps);
        }

        static void ReplaceWithInstance(Scene scene, GameObject old, GameObject prefab)
        {
            var inst = (GameObject)PrefabUtility.InstantiatePrefab(prefab, scene);
            inst.name = old.name;
            inst.transform.SetParent(old.transform.parent, false);
            inst.transform.SetSiblingIndex(old.transform.GetSiblingIndex());
            inst.transform.localPosition = old.transform.localPosition;
            inst.transform.localRotation = old.transform.localRotation;
            inst.transform.localScale = old.transform.localScale;
            inst.SetActive(old.activeSelf);

            var map = BuildMap(old, inst);
            int kept = 0;
            foreach (var oc in old.GetComponentsInChildren<Component>(true))
            {
                if (oc == null) { Debug.LogWarning("[Prefab] old rover has a missing script; skipped"); continue; }
                if (oc.transform == old.transform && oc is Transform) continue; // root pose set above
                if (!map.TryGetValue(oc, out var nc)) { Debug.LogWarning($"[Prefab] NOT IN PREFAB: {oc.GetType().Name} on {GetPath(oc.transform)}"); continue; }
                kept += CopyDifferences(oc, (Component)nc, map);
            }
            foreach (var nc in inst.GetComponentsInChildren<MonoBehaviour>(true).Where(c => !map.ContainsValue(c)))
                Debug.Log($"[Prefab] new in this scene: {nc.GetType().Name} on {GetPath(nc.transform)}");
            Debug.Log($"[Prefab] {kept} settings kept as overrides");

            // Anything in the scene that pointed into the old rover now points into the instance.
            foreach (var c in AllInScene<Component>(scene))
            {
                if (c == null || c.transform.IsChildOf(old.transform) || c.transform.IsChildOf(inst.transform)) continue;
                var so = new SerializedObject(c);
                var it = so.GetIterator();
                bool changed = false;
                while (it.Next(true))
                {
                    if (it.propertyType != SerializedPropertyType.ObjectReference || it.objectReferenceValue == null) continue;
                    if (map.TryGetValue(it.objectReferenceValue, out var to))
                    {
                        Debug.Log($"[Prefab] relinked {c.GetType().Name}.{it.propertyPath} on {GetPath(c.transform)}");
                        it.objectReferenceValue = to; changed = true;
                    }
                    else if (Owner(it.objectReferenceValue) is Transform t && t.IsChildOf(old.transform))
                        Debug.LogWarning($"[Prefab] could not relink {c.GetType().Name}.{it.propertyPath} on {GetPath(c.transform)}");
                }
                if (changed) so.ApplyModifiedPropertiesWithoutUndo();
            }

            AdoptGps(scene, inst);

            // Old scenes had their own sensors outside the rover; the prefab now brings them, so silence the copies.
            foreach (var type in SingleSensors)
            {
                var mine = inst.GetComponentsInChildren(type, true).Cast<MonoBehaviour>().ToArray();
                foreach (var ext in AllInScene<MonoBehaviour>(scene).Where(m => m != null && m.GetType() == type && m.enabled
                                                                                && !m.transform.IsChildOf(inst.transform) && !m.transform.IsChildOf(old.transform)))
                {
                    var topics = Topics(ext);
                    if (mine.Any(m => Topics(m).Overlaps(topics)))
                    {
                        Debug.LogWarning($"[Prefab] disabled duplicate {type.Name} on {GetPath(ext.transform)} ({string.Join(", ", topics)})");
                        ext.enabled = false;
                        EditorUtility.SetDirty(ext);
                    }
                }
            }

            Object.DestroyImmediate(old);
        }

        /// <summary>Old component/GameObject -> matching one in the instance (same path, type and index on the object).</summary>
        static Dictionary<Object, Object> BuildMap(GameObject from, GameObject to)
        {
            var map = new Dictionary<Object, Object>();
            var targets = to.GetComponentsInChildren<Transform>(true).GroupBy(t => RelPath(to.transform, t))
                            .ToDictionary(g => g.Key, g => g.First());
            foreach (var t in from.GetComponentsInChildren<Transform>(true))
            {
                if (!targets.TryGetValue(RelPath(from.transform, t), out var nt)) continue;
                map[t.gameObject] = nt.gameObject;
                var oldComps = t.GetComponents<Component>().Where(c => c != null).ToArray();
                var newComps = nt.GetComponents<Component>().Where(c => c != null).ToArray();
                foreach (var g in oldComps.GroupBy(c => c.GetType()))
                {
                    var same = newComps.Where(c => c.GetType() == g.Key).ToArray();
                    int i = 0;
                    foreach (var c in g) { if (i < same.Length) map[c] = same[i]; i++; }
                }
            }
            return map;
        }

        static readonly HashSet<string> SkipProps = new HashSet<string> {
            "m_Script", "m_GameObject", "m_Father", "m_Children", "m_PrefabInstance", "m_PrefabAsset",
            "m_CorrespondingSourceObject", "m_ObjectHideFlags", "m_EditorHideFlags", "m_EditorClassIdentifier",
        };

        /// <summary>Copies every setting of `from` that differs from `to`, remapping references into the rover. Returns the count.</summary>
        static int CopyDifferences(Component from, Component to, Dictionary<Object, Object> map)
        {
            // Rover physics (masses, joint limits, drives) come from the prefab in every scene, so it's one rover.
            if (from is ArticulationBody)
            {
                var diffs = Differences(from, to).ToArray();
                if (diffs.Length > 0) Debug.Log($"[Prefab] physics from prefab on {GetPath(from.transform)}, dropped: {string.Join("; ", diffs)}");
                return 0;
            }
            var src = new SerializedObject(from);
            var dst = new SerializedObject(to);
            var it = src.GetIterator();
            int n = 0;
            bool enter = true;
            while (it.Next(enter))
            {
                enter = false;
                if (SkipProps.Contains(it.name)) continue;
                var d = dst.FindProperty(it.propertyPath);
                if (d == null) continue;
                if (it.propertyType == SerializedPropertyType.ObjectReference)
                {
                    var v = it.objectReferenceValue;
                    if (v != null && map.TryGetValue(v, out var mapped)) v = mapped;
                    if (d.objectReferenceValue != v) { Log(from, it, "ref -> " + (v ? v.name : "None")); d.objectReferenceValue = v; n++; }
                    continue;
                }
                if (it.isArray && it.propertyType == SerializedPropertyType.Generic)
                {
                    if (d.arraySize != it.arraySize) { Log(from, it, $"size {d.arraySize} -> {it.arraySize}"); d.arraySize = it.arraySize; n++; }
                    enter = true;
                    continue;
                }
                if (it.propertyType == SerializedPropertyType.Generic) { enter = true; continue; }
                if (!SerializedProperty.DataEquals(it, d))
                {
                    Log(from, it, Value(d) + " -> " + Value(it));
                    dst.CopyFromSerializedProperty(it);
                    n++;
                }
            }
            dst.ApplyModifiedPropertiesWithoutUndo();
            return n;
        }

        static IEnumerable<string> Differences(Component from, Component to)
        {
            var d = new SerializedObject(to);
            var it = new SerializedObject(from).GetIterator();
            bool enter = true;
            while (it.Next(enter))
            {
                enter = it.propertyType == SerializedPropertyType.Generic;
                if (SkipProps.Contains(it.name) || enter) continue;
                var p = d.FindProperty(it.propertyPath);
                // Anchor positions differ by float noise between imports; not worth reporting.
                if (p != null && !SerializedProperty.DataEquals(it, p) && !it.name.Contains("Anchor"))
                    yield return $"{it.propertyPath} {Value(it)} (prefab {Value(p)})";
            }
        }

        static void Log(Component c, SerializedProperty p, string what) =>
            Debug.Log($"[Prefab] keep {GetPath(c.transform)} {c.GetType().Name}.{p.propertyPath}: {what}");

        static string Value(SerializedProperty p)
        {
            switch (p.propertyType)
            {
                case SerializedPropertyType.Float: return p.doubleValue.ToString("G6");
                case SerializedPropertyType.Integer: return p.longValue.ToString();
                case SerializedPropertyType.Boolean: return p.boolValue.ToString();
                case SerializedPropertyType.String: return "\"" + p.stringValue + "\"";
                case SerializedPropertyType.Enum: return p.enumValueIndex.ToString();
                case SerializedPropertyType.Vector3: return p.vector3Value.ToString("G4");
                case SerializedPropertyType.Quaternion: return p.quaternionValue.eulerAngles.ToString("G4");
                default: return p.propertyType.ToString();
            }
        }

        static HashSet<string> Topics(MonoBehaviour m)
        {
            var set = new HashSet<string>();
            var it = new SerializedObject(m).GetIterator();
            while (it.NextVisible(true))
                if (it.propertyType == SerializedPropertyType.String && it.stringValue.StartsWith("/")) set.Add(it.stringValue);
            return set;
        }

        static IEnumerable<T> AllInScene<T>(Scene scene) where T : Component =>
            scene.GetRootGameObjects().SelectMany(g => g.GetComponentsInChildren<T>(true));

        static Transform Owner(Object o) => o is Component c ? c.transform : o is GameObject g ? g.transform : null;

        static string RelPath(Transform root, Transform t) => t == root ? "" : RelPath(root, t.parent) + "/" + t.name;

        static string GetPath(Transform t) => t.parent == null ? t.name : GetPath(t.parent) + "/" + t.name;
    }
}
