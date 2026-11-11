using System.Collections.Generic;
using UnityEngine;

namespace Lab4
{
    /// <summary>
    /// 碰撞（穿模）检测：每个零件用凸包碰撞体近似，检查
    ///  · 机器人连杆/夹爪/被夹持的无人机 与 桌面、支撑架、未夹持的无人机、机器人底座；
    ///  · 机器人不相邻连杆之间的自碰撞。
    /// 夹爪手指与无人机、无人机落座时与托盘的接触属于正常接触，不计入。
    /// 运行时把发生穿插的零件标红，供路径规划和批处理验证调用 Check()。
    /// </summary>
    public class CollisionMonitor : MonoBehaviour
    {
        public RobotArm robot;
        public Gripper gripper;
        public Graspable drone;
        public SupportStand[] stands;
        public Transform table;
        [Tooltip("穿插深度超过此值才判为碰撞（米），用于容忍凸包近似误差")] public float tolerance = 0.002f;
        [Tooltip("运行时逐帧检测（调试用，默认关闭；路径规划的碰撞检查不受影响）")] public bool runtimeCheck = false;
        [Tooltip("逐帧检测时把穿插的零件标红")] public bool highlight = false;

        class Body
        {
            public string name;
            public int link;            // 机器人：0 底座，1..6 连杆，7 夹爪手指；环境：-1
            public SupportStand stand;  // 属于哪个支撑架（托盘接触判定用）
            public readonly List<Collider> cols = new List<Collider>();
            public readonly List<Renderer> rends = new List<Renderer>();
        }

        readonly List<Body> robotBodies = new List<Body>();
        readonly List<Body> envBodies = new List<Body>();
        Body droneBody;
        readonly HashSet<(Collider, Collider)> ignoredSelf = new HashSet<(Collider, Collider)>();
        readonly Dictionary<Renderer, Color[]> baseColors = new Dictionary<Renderer, Color[]>();
        readonly HashSet<Renderer> tinted = new HashSet<Renderer>();
        bool ready;

        public void Init()
        {
            if (ready) return;
            robotBodies.Add(MakeBody("机器人底座", 0, robot.transform.Find("Base")));
            for (int i = 0; i < robot.joints.Length; i++)
                robotBodies.Add(MakeBody($"连杆{i + 1}", i + 1, robot.joints[i].Find($"Link{i + 1}")));
            var jaws = MakeBody("夹爪手指", 7, gripper.jawA);
            AddColliders(jaws, gripper.jawB);
            robotBodies.Add(jaws);
            if (table == null) table = GameObject.Find("Table")?.transform;
            if (table != null) envBodies.Add(MakeBody("桌面", -1, table));
            foreach (var s in stands)
            {
                var b = MakeBody(s.displayName, -1, s.transform);
                b.stand = s;
                envBodies.Add(b);
            }
            droneBody = MakeBody("无人机", -1, drone.transform);

            // 参考位形下就已重叠的连杆零件对（凸包近似造成的假阳性）不参与自碰撞检测
            var saved = robot.Angles;
            robot.SetAngles(new float[robot.Dof]);
            Physics.SyncTransforms();
            foreach (var (a, b) in SelfPairs())
                foreach (var ca in a.cols)
                    foreach (var cb in b.cols)
                        if (Penetration(ca, cb) > 0f) ignoredSelf.Add((ca, cb));
            robot.SetAngles(saved);
            ready = true;
        }

        Body MakeBody(string name, int link, Transform root)
        {
            var b = new Body { name = name, link = link };
            AddColliders(b, root);
            return b;
        }

        readonly Dictionary<Collider, Renderer> colRenderer = new Dictionary<Collider, Renderer>();

        void AddColliders(Body b, Transform root)
        {
            if (root == null) return;
            foreach (var mf in root.GetComponentsInChildren<MeshFilter>(true))
            {
                if (mf.name.StartsWith("col")) continue;
                // 支撑架、机器人等子树里可能挂着无人机，跳过它
                if (drone != null && b.name != "无人机" && mf.transform.IsChildOf(drone.transform)) continue;
                var r = mf.GetComponent<Renderer>();
                if (r != null) b.rends.Add(r);
                foreach (var mc in ConvexPieces(mf))
                {
                    b.cols.Add(mc);
                    colRenderer[mc] = r;
                }
            }
        }

        /// <summary>
        /// 零件的凸包碰撞体。较大的零件（如弯曲的连杆）先按三角形位置做 k 均值聚类拆成几块，
        /// 每块单独求凸包，避免一个大凸包把弯头内侧的空隙也包进去。
        /// </summary>
        static List<MeshCollider> ConvexPieces(MeshFilter mf)
        {
            var list = new List<MeshCollider>();
            foreach (Transform c in mf.transform)
                if (c.name.StartsWith("col") && c.TryGetComponent<MeshCollider>(out var existing)) list.Add(existing);
            if (list.Count > 0) return list;

            Mesh mesh = mf.sharedMesh;
            Vector3 size = mesh.bounds.size;
            float extent = Mathf.Max(size.x, Mathf.Max(size.y, size.z));
            int k = Mathf.Clamp(Mathf.CeilToInt(extent / 0.05f), 1, 8);
            var verts = mesh.vertices;
            var tris = mesh.triangles;
            int nt = tris.Length / 3;
            var cent = new Vector3[nt];
            for (int t = 0; t < nt; t++) cent[t] = (verts[tris[3 * t]] + verts[tris[3 * t + 1]] + verts[tris[3 * t + 2]]) / 3f;

            var label = new int[nt];
            if (k > 1)
            {
                // 最远点初始化 + 8 轮 k 均值
                var centers = new Vector3[k];
                centers[0] = cent[0];
                for (int c = 1; c < k; c++)
                {
                    float best = -1; int bi = 0;
                    for (int t = 0; t < nt; t += 7)
                    {
                        float d = float.MaxValue;
                        for (int j = 0; j < c; j++) d = Mathf.Min(d, (cent[t] - centers[j]).sqrMagnitude);
                        if (d > best) { best = d; bi = t; }
                    }
                    centers[c] = cent[bi];
                }
                for (int it = 0; it < 8; it++)
                {
                    var sum = new Vector3[k]; var cnt = new int[k];
                    for (int t = 0; t < nt; t++)
                    {
                        int bj = 0; float bd = float.MaxValue;
                        for (int j = 0; j < k; j++) { float d = (cent[t] - centers[j]).sqrMagnitude; if (d < bd) { bd = d; bj = j; } }
                        label[t] = bj; sum[bj] += cent[t]; cnt[bj]++;
                    }
                    for (int j = 0; j < k; j++) if (cnt[j] > 0) centers[j] = sum[j] / cnt[j];
                }
            }

            for (int j = 0; j < k; j++)
            {
                var pts = new List<Vector3>();
                var idx = new List<int>();
                var map = new Dictionary<int, int>();
                for (int t = 0; t < nt; t++)
                {
                    if (label[t] != j) continue;
                    for (int e = 0; e < 3; e++)
                    {
                        int vi = tris[3 * t + e];
                        if (!map.TryGetValue(vi, out int ni)) { ni = pts.Count; map[vi] = ni; pts.Add(verts[vi]); }
                        idx.Add(ni);
                    }
                }
                if (idx.Count < 12) continue;
                var b = new Bounds(pts[0], Vector3.zero);
                foreach (var p in pts) b.Encapsulate(p);
                if (Mathf.Min(b.size.x, Mathf.Min(b.size.y, b.size.z)) < 0.0005f) continue;   // 近似平面，无法求凸包
                var piece = new Mesh { name = mesh.name + "_col" + j, indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
                piece.SetVertices(pts);
                piece.SetTriangles(idx, 0);
                var go = new GameObject("col" + j) { hideFlags = HideFlags.DontSave };
                go.transform.SetParent(mf.transform, false);
                var mc = go.AddComponent<MeshCollider>();
                mc.sharedMesh = piece;
                mc.convex = true;
                mc.isTrigger = true;
                list.Add(mc);
            }
            return list;
        }

        IEnumerable<(Body, Body)> SelfPairs()
        {
            foreach (var a in robotBodies)
                foreach (var b in robotBodies)
                {
                    if (a.link >= b.link) continue;
                    bool adjacent = b.link - a.link < 2 || (a.link == 6 && b.link == 7);
                    if (!adjacent) yield return (a, b);
                }
        }

        static float Penetration(Collider a, Collider b)
        {
            if (!a.bounds.Intersects(b.bounds)) return 0f;
            return Physics.ComputePenetration(a, a.transform.position, a.transform.rotation,
                                              b, b.transform.position, b.transform.rotation,
                                              out _, out float d) ? d : 0f;
        }

        /// <summary>检查当前姿态，返回发生穿插的零件对（“A ↔ B”）。</summary>
        public List<string> Check(bool tint = false)
        {
            Init();
            Physics.SyncTransforms();
            var hits = new List<string>();
            var hitRends = new HashSet<Renderer>();
            bool held = gripper.Held == drone.transform;

            void Test(Body a, Body b, System.Func<Collider, Collider, bool> skip = null)
            {
                foreach (var ca in a.cols)
                    foreach (var cb in b.cols)
                    {
                        if (skip != null && skip(ca, cb)) continue;
                        if (Penetration(ca, cb) <= tolerance) continue;
                        string s = $"{a.name}（{Part(ca)}）↔ {b.name}（{Part(cb)}）";
                        if (!hits.Contains(s)) hits.Add(s);
                        if (colRenderer.TryGetValue(ca, out var ra)) hitRends.Add(ra);
                        if (colRenderer.TryGetValue(cb, out var rb)) hitRends.Add(rb);
                    }
            }

            // 机器人自碰撞
            foreach (var (a, b) in SelfPairs())
                Test(a, b, (ca, cb) => ignoredSelf.Contains((ca, cb)));

            // 机器人（不含底座）与环境；底座本来就放在桌上
            foreach (var r in robotBodies)
            {
                if (r.link == 0) continue;
                foreach (var e in envBodies) Test(r, e);
                if (!held && r.link != 7) Test(r, droneBody);   // 手指与无人机的接触是夹持
            }

            // 被夹持的无人机与环境、机器人底座及前段连杆；落座接触不算
            if (held)
            {
                foreach (var e in envBodies)
                {
                    if (e.stand != null && Vector3.Distance(drone.transform.position, e.stand.pad.position) < 0.03f) continue;
                    Test(droneBody, e);
                }
                foreach (var r in robotBodies)
                    if (r.link <= 5) Test(droneBody, r);
            }

            if (tint) Tint(hitRends);
            return hits;
        }

        static string Part(Collider c)
        {
            string n = c.transform.parent != null ? c.transform.parent.name : c.gameObject.name;
            int sp = n.IndexOf(' ');
            return sp > 0 ? n.Substring(sp + 1) : n;
        }

        void Tint(HashSet<Renderer> hit)
        {
            foreach (var r in tinted)
                if (r != null && !hit.Contains(r) && baseColors.TryGetValue(r, out var cs))
                {
                    var mats = r.materials;
                    for (int i = 0; i < mats.Length; i++) mats[i].color = cs[i];
                }
            tinted.RemoveWhere(r => r == null || !hit.Contains(r));
            foreach (var r in hit)
            {
                if (r == null || tinted.Contains(r)) continue;
                var mats = r.materials;
                if (!baseColors.ContainsKey(r))
                {
                    var cs = new Color[mats.Length];
                    for (int i = 0; i < mats.Length; i++) cs[i] = mats[i].color;
                    baseColors[r] = cs;
                }
                foreach (var m in mats) m.color = new Color(0.95f, 0.15f, 0.15f);
                tinted.Add(r);
            }
        }

        /// <summary>最近一次运行时逐帧检测的结果。</summary>
        public List<string> LastHits { get; private set; } = new List<string>();

        // 运行时逐帧检测并标红
        void LateUpdate()
        {
            if (!Application.isPlaying || robot == null || !runtimeCheck) return;
            LastHits = Check(highlight);
        }
    }
}
