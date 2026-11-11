using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;

namespace Lab4.EditorTools
{
    /// <summary>
    /// 根据 ModelData（由 STEP 模型转换得到的网格与关节/支架测量数据）自动生成仿真场景：
    /// 网格与材质资源、运动层级（关节、铰链、托盘坐标系）以及全部控制组件。
    /// 菜单：Lab4 / 生成仿真场景
    /// </summary>
    public static class SceneBuilder
    {
        const string ScenePath = "Assets/Scenes/Lab4.unity";
        const string GenDir = "Assets/Generated";

        [System.Serializable] class V3 { public float x, y, z; public Vector3 V => new Vector3(x, y, z); }
        [System.Serializable] class Group { public string name; public string[] parts; }
        [System.Serializable] class PartInfo { public string id, name; }
        [System.Serializable] class JointInfo { public V3 point, axis; }
        [System.Serializable]
        class SceneData
        {
            public Group[] groups; public PartInfo[] parts; public JointInfo[] joints;
            public V3 tcp, approach, jaw; public float openGap, maxStroke;
            public V3 columnCenter, columnPadOrigin, columnPadUp, columnPadRight;
            public V3 tiltCenter, tiltPadOrigin, tiltPadUp, tiltPadRight;
            public V3 hingePoint, hingeAxis; public float modelTiltDeg;
            public V3 graspInPad;
        }

        static string DataDir => Path.GetFullPath(Path.Combine(Application.dataPath, "..", "ModelData"));
        static readonly Dictionary<string, Material> materials = new Dictionary<string, Material>();

        /// <summary>场景尚未生成时先生成（仓库中不包含生成的网格与场景）。</summary>
        public static void EnsureScene()
        {
            if (!File.Exists(ScenePath)) Build();
        }

        [MenuItem("Lab4/生成仿真场景")]
        public static void Build()
        {
            ConfigureProject();
            var data = JsonUtility.FromJson<SceneData>(File.ReadAllText(Path.Combine(DataDir, "scene_unity.json")));
            var names = new Dictionary<string, string>();
            foreach (var p in data.parts) names[p.id] = p.name;
            var groups = new Dictionary<string, string[]>();
            foreach (var g in data.groups) groups[g.name] = g.parts;

            Directory.CreateDirectory(GenDir + "/Meshes");
            Directory.CreateDirectory(GenDir + "/Materials");
            Directory.CreateDirectory("Assets/Scenes");
            materials.Clear();

            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // ---------------------------------------------------------------- 环境
            var lightGo = new GameObject("Directional Light");
            var light = lightGo.AddComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = 1.1f;
            light.shadows = LightShadows.Soft;
            lightGo.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
            var fill = new GameObject("Fill Light").AddComponent<Light>();
            fill.type = LightType.Directional;
            fill.intensity = 0.35f;
            fill.shadows = LightShadows.None;
            fill.transform.rotation = Quaternion.Euler(30f, 150f, 0f);
            RenderSettings.ambientMode = AmbientMode.Trilight;
            RenderSettings.ambientSkyColor = new Color(0.62f, 0.66f, 0.72f);
            RenderSettings.ambientEquatorColor = new Color(0.45f, 0.46f, 0.48f);
            RenderSettings.ambientGroundColor = new Color(0.25f, 0.25f, 0.25f);

            var floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
            floor.name = "Floor";
            floor.transform.position = new Vector3(0f, -0.75f, 0f);
            floor.transform.localScale = Vector3.one * 0.6f;
            floor.GetComponent<Renderer>().sharedMaterial = Mat("c_8c9196");
            Object.DestroyImmediate(floor.GetComponent<Collider>());
            var legMat = Mat("c_5a5e63");
            foreach (var c in new[] { new Vector2(-0.46f, -0.46f), new Vector2(0.46f, -0.46f), new Vector2(-0.46f, 0.46f), new Vector2(0.46f, 0.46f) })
            {
                var leg = GameObject.CreatePrimitive(PrimitiveType.Cube);
                leg.name = "TableLeg";
                leg.transform.position = new Vector3(c.x, -0.39f, c.y);
                leg.transform.localScale = new Vector3(0.04f, 0.72f, 0.04f);
                leg.GetComponent<Renderer>().sharedMaterial = legMat;
                Object.DestroyImmediate(leg.GetComponent<Collider>());
            }

            var world = new GameObject("Lab4 World").transform;
            var table = new GameObject("Table").transform;
            table.SetParent(world, false);
            AddParts(table, groups["Table"], names);

            // ---------------------------------------------------------------- 机器人
            var robotRoot = new GameObject("Robot").transform;
            robotRoot.SetParent(world, false);
            robotRoot.position = data.joints[0].point.V - Vector3.up * data.joints[0].point.y;
            AddParts(Node("Base", robotRoot), groups["RobotBase"], names);
            var jointNodes = new Transform[data.joints.Length];
            Transform parent = robotRoot;
            for (int i = 0; i < data.joints.Length; i++)
            {
                var j = new GameObject($"J{i + 1}").transform;
                j.SetParent(parent, true);
                j.position = data.joints[i].point.V;
                j.rotation = Quaternion.identity;
                AddParts(Node($"Link{i + 1}", j), groups[$"Link{i + 1}"], names);
                jointNodes[i] = j;
                parent = j;
            }
            var j6 = jointNodes[jointNodes.Length - 1];
            var tcp = new GameObject("TCP").transform;
            tcp.SetParent(j6, true);
            tcp.position = data.tcp.V;
            tcp.rotation = Quaternion.LookRotation(data.approach.V, Vector3.Cross(data.approach.V, data.jaw.V));
            var jawA = Node("JawA", j6);
            AddParts(jawA, groups["JawA"], names);
            var jawB = Node("JawB", j6);
            AddParts(jawB, groups["JawB"], names);

            var arm = robotRoot.gameObject.AddComponent<RobotArm>();
            arm.joints = jointNodes;
            arm.axes = new Vector3[jointNodes.Length];
            arm.limitsDeg = new Vector2[jointNodes.Length];
            for (int i = 0; i < jointNodes.Length; i++)
            {
                arm.axes[i] = data.joints[i].axis.V;
                arm.limitsDeg[i] = i == 2 ? new Vector2(-165f, 165f) : new Vector2(-180f, 180f);
            }
            arm.tcp = tcp;
            var grip = robotRoot.gameObject.AddComponent<Gripper>();
            grip.jawA = jawA; grip.jawB = jawB; grip.tcp = tcp;
            grip.jawAxisLocal = data.jaw.V;
            grip.openGap = data.openGap; grip.maxStroke = data.maxStroke;

            // ---------------------------------------------------------------- 立柱支撑架
            float tableTop = data.columnCenter.y;
            var column = new GameObject("ColumnStand").transform;
            column.SetParent(world, false);
            column.position = data.columnCenter.V;
            AddParts(Node("Meshes", column), groups["ColumnSupport"], names);
            var columnPad = PadFrame("Pad", column, data.columnPadOrigin.V, data.columnPadUp.V, data.columnPadRight.V);
            var colStand = column.gameObject.AddComponent<SupportStand>();
            colStand.displayName = "立柱支撑架";
            colStand.pad = columnPad;
            colStand.tableHeight = tableTop;
            colStand.positionXZ = new Vector2(column.position.x, column.position.z);

            // ---------------------------------------------------------------- 斜面支撑架
            var tilt = new GameObject("TiltStand").transform;
            tilt.SetParent(world, false);
            tilt.position = data.tiltCenter.V;
            AddParts(Node("BaseMeshes", tilt), groups["TiltSupportBase"], names);
            var hinge = new GameObject("Hinge").transform;
            hinge.SetParent(tilt, true);
            hinge.position = data.hingePoint.V;
            AddParts(Node("TopMeshes", hinge), groups["TiltSupportTop"], names);
            var tiltPad = PadFrame("Pad", hinge, data.tiltPadOrigin.V, data.tiltPadUp.V, data.tiltPadRight.V);
            var tStand = tilt.gameObject.AddComponent<SupportStand>();
            tStand.displayName = "斜面支撑架";
            tStand.pad = tiltPad;
            tStand.canTilt = true;
            tStand.hinge = hinge;
            tStand.hingeAxisLocal = data.hingeAxis.V;
            tStand.modelTiltDeg = data.modelTiltDeg;
            tStand.tableHeight = tableTop;
            tStand.positionXZ = new Vector2(tilt.position.x, tilt.position.z);
            // 倾角为 0 时托盘水平：确定“倾角增大”对应的铰链旋转方向
            float level = Vector3.SignedAngle(data.tiltPadUp.V, Vector3.up, data.hingeAxis.V);
            tStand.tiltSign = -Mathf.Sign(level);
            tStand.tiltDeg = data.modelTiltDeg;

            // ---------------------------------------------------------------- 无人机
            var drone = new GameObject("Drone").transform;
            drone.SetPositionAndRotation(tiltPad.position, tiltPad.rotation);
            AddParts(Node("Meshes", drone), groups["Drone"], names);
            var gp = new GameObject("GraspPoint").transform;
            gp.SetParent(drone, false);
            gp.localPosition = data.graspInPad.V;
            gp.localRotation = Quaternion.LookRotation(Vector3.down, Vector3.left); // 接近方向朝下，夹持方向沿托盘 z
            var graspable = drone.gameObject.AddComponent<Graspable>();
            graspable.graspPoint = gp;
            drone.SetParent(tiltPad, true);

            // ---------------------------------------------------------------- 任务与界面
            var ctrl = new GameObject("Lab4 Controller");
            var task = ctrl.AddComponent<PickPlaceTask>();
            task.robot = arm; task.gripper = grip; task.drone = graspable;
            task.columnStand = colStand; task.tiltStand = tStand;
            task.custom = new TaskSetup
            {
                name = "自定场景",
                column = new StandPose(-180, 160, 30),
                tilt = new StandPose(160, 170, -20, 25),
                startOnTilt = true,
            };
            task.teacher = new TaskSetup
            {
                name = "教师指定场景",
                column = new StandPose(Mathf.Round(colStand.positionXZ.x * 1000), Mathf.Round(colStand.positionXZ.y * 1000), 0),
                tilt = new StandPose(Mathf.Round(tStand.positionXZ.x * 1000), Mathf.Round(tStand.positionXZ.y * 1000), 0, Mathf.Round(data.modelTiltDeg * 10) / 10),
                startOnTilt = true,
            };
            ctrl.AddComponent<ControlPanel>().task = task;

            var camGo = new GameObject("Main Camera");
            camGo.tag = "MainCamera";
            var cam = camGo.AddComponent<Camera>();
            cam.nearClipPlane = 0.01f;
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0.86f, 0.88f, 0.91f);
            camGo.AddComponent<AudioListener>();
            camGo.AddComponent<OrbitCamera>().follow = tcp;

            EditorSceneManager.SaveScene(scene, ScenePath);
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };
            AssetDatabase.SaveAssets();
            Debug.Log("[Lab4] 场景已生成：" + ScenePath);
        }

        static Transform Node(string name, Transform parent)
        {
            var t = new GameObject(name).transform;
            t.SetParent(parent, false);
            t.position = Vector3.zero;   // 子网格顶点为世界坐标，节点放在原点，父节点运动时随之运动
            t.rotation = Quaternion.identity;
            return t;
        }

        static Transform PadFrame(string name, Transform parent, Vector3 origin, Vector3 up, Vector3 right)
        {
            var t = new GameObject(name).transform;
            t.SetParent(parent, true);
            t.SetPositionAndRotation(origin, Quaternion.LookRotation(Vector3.Cross(right, up), up));
            return t;
        }

        static void AddParts(Transform parent, string[] ids, Dictionary<string, string> names)
        {
            foreach (var id in ids)
            {
                var go = new GameObject($"{id} {names[id]}");
                go.transform.SetParent(parent, false);
                go.transform.position = Vector3.zero;
                go.transform.rotation = Quaternion.identity;
                // 顶点为世界坐标，父节点链在参考位形下为恒等变换（Node 置于原点）
                go.transform.SetParent(parent, true);
                var mesh = LoadMesh(id, out var mats);
                go.AddComponent<MeshFilter>().sharedMesh = mesh;
                go.AddComponent<MeshRenderer>().sharedMaterials = mats;
            }
        }

        static Mesh LoadMesh(string id, out Material[] mats)
        {
            string asset = $"{GenDir}/Meshes/{id}.asset";
            var verts = new List<Vector3>();
            var subs = new List<List<int>>();
            var matNames = new List<string>();
            List<int> cur = null;
            foreach (var line in File.ReadLines(Path.Combine(DataDir, "meshes", id + ".obj")))
            {
                if (line.Length < 2) continue;
                var t = line.Split(' ');
                if (t[0] == "v")
                    verts.Add(new Vector3(F(t[1]), F(t[2]), F(t[3])));
                else if (t[0] == "usemtl")
                {
                    int k = matNames.IndexOf(t[1]);
                    if (k < 0) { matNames.Add(t[1]); subs.Add(new List<int>()); k = subs.Count - 1; }
                    cur = subs[k];
                }
                else if (t[0] == "f")
                {
                    cur.Add(int.Parse(t[1]) - 1); cur.Add(int.Parse(t[2]) - 1); cur.Add(int.Parse(t[3]) - 1);
                }
            }
            var mesh = new Mesh { name = id, indexFormat = IndexFormat.UInt32 };
            mesh.SetVertices(verts);
            mesh.subMeshCount = subs.Count;
            for (int i = 0; i < subs.Count; i++) mesh.SetTriangles(subs[i], i);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            AssetDatabase.DeleteAsset(asset);
            AssetDatabase.CreateAsset(mesh, asset);
            mats = new Material[matNames.Count];
            for (int i = 0; i < mats.Length; i++) mats[i] = Mat(matNames[i]);
            return mesh;
        }

        static float F(string s) => float.Parse(s, CultureInfo.InvariantCulture);

        static Material Mat(string name)
        {
            if (materials.TryGetValue(name, out var m)) return m;
            string path = $"{GenDir}/Materials/{name}.mat";
            m = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (m == null)
            {
                var shader = Shader.Find("Standard") ?? Shader.Find("Universal Render Pipeline/Lit");
                m = new Material(shader);
                AssetDatabase.CreateAsset(m, path);
            }
            ColorUtility.TryParseHtmlString("#" + name.Substring(2), out var c);
            m.color = c;
            m.SetFloat("_Glossiness", 0.35f);
            m.SetFloat("_Smoothness", 0.35f);
            m.enableInstancing = true;
            EditorUtility.SetDirty(m);
            materials[name] = m;
            return m;
        }

        /// <summary>使用旧版输入管理器（控制面板与相机使用 UnityEngine.Input）。</summary>
        static void ConfigureProject()
        {
            var ps = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/ProjectSettings.asset");
            if (ps.Length == 0) return;
            var so = new SerializedObject(ps[0]);
            var p = so.FindProperty("activeInputHandler");
            if (p != null && p.intValue == 1) { p.intValue = 2; so.ApplyModifiedPropertiesWithoutUndo(); }
            PlayerSettings.productName = "Lab4 DroneSim";
            PlayerSettings.colorSpace = ColorSpace.Linear;
        }
    }
}
