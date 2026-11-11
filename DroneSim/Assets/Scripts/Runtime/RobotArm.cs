using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Lab4
{
    /// <summary>
    /// 6 轴机器人：关节控制、正运动学（指数积 POE）与数值逆运动学（阻尼最小二乘）。
    /// 关节轴与零位全部来自 CAD 模型：CAD 中的姿态即所有关节角为 0 的参考位形。
    /// </summary>
    public class RobotArm : MonoBehaviour
    {
        [Tooltip("J1..J6 关节节点（父子嵌套），参考位形下旋转均为单位旋转")]
        public Transform[] joints = new Transform[6];
        [Tooltip("各关节在其父节点坐标系中的旋转轴")]
        public Vector3[] axes = new Vector3[6];
        public Vector2[] limitsDeg = new Vector2[6];
        [Tooltip("工具中心点 TCP：forward=接近方向，right=夹爪开合方向")]
        public Transform tcp;
        [Tooltip("关节最大角速度（度/秒）")]
        public float jointSpeed = 60f;
        [Tooltip("直线运动速度（米/秒）")]
        public float linearSpeed = 0.08f;
        [Tooltip("IK 结构化初值均失败时的随机重启次数")]
        public int randomRestarts = 40;

        [SerializeField] float[] angles = new float[6];

        // 参考位形下（机器人根坐标系）的关节点、轴与 TCP 位姿
        Vector3[] q0, w0;
        Pose tcp0;
        bool cached;

        public int Dof => joints.Length;
        public float[] Angles => (float[])angles.Clone();
        public bool IsMoving { get; private set; }

        void Awake() => Cache();

        void Cache()
        {
            if (cached) return;
            int n = joints.Length;
            q0 = new Vector3[n];
            w0 = new Vector3[n];
            var saved = (float[])angles.Clone();
            ApplyAngles(new float[n]);
            for (int i = 0; i < n; i++)
            {
                q0[i] = transform.InverseTransformPoint(joints[i].position);
                w0[i] = transform.InverseTransformDirection(joints[i].parent.TransformDirection(axes[i])).normalized;
            }
            tcp0 = new Pose(transform.InverseTransformPoint(tcp.position),
                            Quaternion.Inverse(transform.rotation) * tcp.rotation);
            ApplyAngles(saved);
            cached = true;
        }

        // ------------------------------------------------------------ 关节控制

        /// <summary>直接设置全部关节角（度），超限部分会被截断。</summary>
        public void SetAngles(float[] deg)
        {
            for (int i = 0; i < angles.Length; i++)
                angles[i] = Mathf.Clamp(deg[i], limitsDeg[i].x, limitsDeg[i].y);
            ApplyAngles(angles);
        }

        public void SetJoint(int index, float deg)
        {
            var a = Angles;
            a[index] = deg;
            SetAngles(a);
        }

        void ApplyAngles(float[] deg)
        {
            for (int i = 0; i < joints.Length; i++)
                joints[i].localRotation = Quaternion.AngleAxis(deg[i], axes[i]);
        }

        /// <summary>关节空间插补运动（各关节同时起止，S 形速度曲线）。</summary>
        public IEnumerator MoveJ(float[] target, float speedScale = 1f)
        {
            IsMoving = true;
            var start = Angles;
            float maxDelta = 0f;
            for (int i = 0; i < start.Length; i++) maxDelta = Mathf.Max(maxDelta, Mathf.Abs(target[i] - start[i]));
            float duration = Mathf.Max(0.3f, maxDelta / (jointSpeed * speedScale));
            var cur = new float[start.Length];
            for (float t = 0; t < 1f; t += SimClock.Dt / duration)
            {
                float s = Mathf.SmoothStep(0f, 1f, t);
                for (int i = 0; i < cur.Length; i++) cur[i] = Mathf.Lerp(start[i], target[i], s);
                SetAngles(cur);
                yield return null;
            }
            SetAngles(target);
            IsMoving = false;
        }

        /// <summary>
        /// 笛卡尔直线运动：TCP 沿直线移动到目标位姿，每一小步都用 IK 求解。
        /// 返回 false 表示途中某点不可达（机器人会停在最后一个可达点）。
        /// </summary>
        public IEnumerator MoveL(Pose target, System.Action<bool> done = null)
        {
            IsMoving = true;
            Pose start = TcpPose();
            float dist = Vector3.Distance(start.position, target.position);
            float duration = Mathf.Max(0.4f, dist / linearSpeed);
            bool ok = true;
            for (float t = 0; t < 1f; t += SimClock.Dt / duration)
            {
                float s = Mathf.SmoothStep(0f, 1f, t);
                var p = new Pose(Vector3.Lerp(start.position, target.position, s),
                                 Quaternion.Slerp(start.rotation, target.rotation, s));
                if (!SolveIK(p, Angles, out var sol)) { ok = false; break; }
                SetAngles(sol);
                yield return null;
            }
            if (ok && SolveIK(target, Angles, out var fin)) SetAngles(fin); else ok = false;
            IsMoving = false;
            done?.Invoke(ok);
        }

        /// <summary>按预先规划好的关节序列运动（序列由路径规划器生成并已做碰撞检查）。</summary>
        public IEnumerator PlayPath(List<float[]> path, float duration)
        {
            if (path == null || path.Count == 0) yield break;
            IsMoving = true;
            duration = Mathf.Max(0.3f, duration);
            var cur = new float[Dof];
            for (float t = 0; t < 1f; t += SimClock.Dt / duration)
            {
                float s = Mathf.SmoothStep(0f, 1f, t) * (path.Count - 1);
                int i = Mathf.Min((int)s, path.Count - 2);
                if (path.Count == 1) { SetAngles(path[0]); yield return null; continue; }
                float f = s - i;
                for (int j = 0; j < Dof; j++) cur[j] = Mathf.Lerp(path[i][j], path[i + 1][j], f);
                SetAngles(cur);
                yield return null;
            }
            SetAngles(path[path.Count - 1]);
            IsMoving = false;
        }

        // ------------------------------------------------------------ 正运动学

        /// <summary>当前 TCP 的世界位姿。</summary>
        public Pose TcpPose() => new Pose(tcp.position, tcp.rotation);

        /// <summary>给定关节角（度）计算 TCP 世界位姿（不改变场景）。</summary>
        public Pose ForwardKinematics(float[] deg)
        {
            Cache();
            Quaternion r = Quaternion.identity;
            Vector3 p = Vector3.zero;
            for (int i = 0; i < joints.Length; i++)
            {
                // exp(ξ_i θ_i)：绕过点 q、方向 w 的轴旋转
                Quaternion ri = Quaternion.AngleAxis(deg[i], w0[i]);
                Vector3 pi = q0[i] - ri * q0[i];
                p = p + r * pi;
                r = r * ri;
            }
            Vector3 pt = p + r * tcp0.position;
            Quaternion rt = r * tcp0.rotation;
            return new Pose(transform.TransformPoint(pt), transform.rotation * rt);
        }

        // ------------------------------------------------------------ 逆运动学

        static readonly float[][] SeedOffsets =
        {
            new float[] { 0, 0, 0, 0, 0, 0 },
            new float[] { 0, -40, 80, -40, 0, 0 },
            new float[] { 0, 40, -80, 40, 0, 0 },
            new float[] { 0, 0, 0, 0, 90, 0 },
            new float[] { 0, 0, 0, 0, -90, 0 },
            new float[] { 0, -30, 60, 60, 90, 0 },
            new float[] { 0, 30, -60, -60, -90, 0 },
            new float[] { 0, -60, 100, -130, 90, 0 },
            new float[] { 0, 60, -100, 130, -90, 0 },
        };

        /// <summary>
        /// 求解使 TCP 到达目标位姿的关节角。从 seed 及多组附加初值出发迭代，
        /// 在满足精度与关节限位的解中选取离 seed 最近的一个。
        /// </summary>
        public bool SolveIK(Pose target, float[] seed, out float[] best, float posTol = 0.0005f, float rotTolDeg = 0.2f)
        {
            Cache();
            best = null;
            float bestCost = float.MaxValue;
            var seeds = new List<float[]> { seed };
            // 以“J1 转向目标”的角度为基础，补充几组典型构型作为初值
            Vector3 axis = transform.TransformDirection(w0[0]);
            Vector3 basePt = transform.TransformPoint(q0[0]);
            Vector3 from = Vector3.ProjectOnPlane(ForwardKinematics(seed).position - basePt, axis);
            Vector3 to = Vector3.ProjectOnPlane(target.position - basePt, axis);
            float turn = Vector3.SignedAngle(from, to, axis);
            foreach (float j1 in new[] { turn, 0f })
                foreach (var off in SeedOffsets)
                {
                    var s = new float[joints.Length];
                    for (int i = 0; i < s.Length; i++) s[i] = seed[i] + off[i];
                    s[0] += j1;
                    seeds.Add(s);
                }
            // 结构化初值都失败时，再用若干随机初值重启（固定随机种子，结果可复现）
            var rng = new System.Random(7);
            for (int k = 0; k < randomRestarts; k++)
            {
                var s = new float[joints.Length];
                for (int i = 0; i < s.Length; i++)
                    s[i] = Mathf.Lerp(limitsDeg[i].x, limitsDeg[i].y, (float)rng.NextDouble());
                seeds.Add(s);
            }
            int structured = seeds.Count - randomRestarts;
            for (int si = 0; si < seeds.Count; si++)
            {
                var s = seeds[si];
                if (si >= structured && best != null) break;
                if (!Iterate(target, s, out var sol, posTol, rotTolDeg)) continue;
                float cost = 0;
                for (int i = 0; i < sol.Length; i++) cost += Mathf.Abs(sol[i] - seed[i]) * (i < 3 ? 1.5f : 1f);
                if (cost < bestCost) { bestCost = cost; best = sol; }
                if (s == seed && cost < 5f) break;   // 连续运动时首个初值通常已足够
            }
            return best != null;
        }

        /// <summary>
        /// 求目标位姿的多组不同 IK 解（肘部上/下、手腕翻转等不同构型），按离 seed 由近到远排序。
        /// 路径规划器会逐个检查，选出能无碰撞到达的那一组。
        /// </summary>
        public List<float[]> SolveIKAll(Pose target, float[] seed, int max = 6)
        {
            var sols = new List<float[]>();
            foreach (var s in IKSeeds(seed)) AddSolution(target, s, sols);
            return SortSolutions(sols, seed, max);
        }

        /// <summary>多解求解的初值集合：典型构型偏移 × J1 翻转 + 少量随机初值。</summary>
        public List<float[]> IKSeeds(float[] seed)
        {
            Cache();
            var seeds = new List<float[]> { seed };
            foreach (var off in SeedOffsets)
                foreach (float j1 in new[] { 0f, 180f, -180f })
                {
                    var s = new float[Dof];
                    for (int i = 0; i < Dof; i++) s[i] = seed[i] + off[i];
                    s[0] += j1;
                    seeds.Add(s);
                }
            var rng = new System.Random(11);
            for (int k = 0; k < Mathf.Min(randomRestarts, 10); k++)
            {
                var s = new float[Dof];
                for (int i = 0; i < Dof; i++) s[i] = Mathf.Lerp(limitsDeg[i].x, limitsDeg[i].y, (float)rng.NextDouble());
                seeds.Add(s);
            }
            return seeds;
        }

        /// <summary>从一个初值迭代求解，收敛且与已有解不重复（任一关节差 ≥ 5°）时加入列表。</summary>
        public void AddSolution(Pose target, float[] seed, List<float[]> sols)
        {
            if (!Iterate(target, seed, out var sol, 0.0005f, 0.2f)) return;
            foreach (var o in sols)
            {
                float d = 0;
                for (int i = 0; i < Dof; i++) d = Mathf.Max(d, Mathf.Abs(Mathf.DeltaAngle(o[i], sol[i])));
                if (d < 5f) return;
            }
            sols.Add(sol);
        }

        public static List<float[]> SortSolutions(List<float[]> sols, float[] seed, int max)
        {
            sols.Sort((a, b) => Distance(a, seed).CompareTo(Distance(b, seed)));
            if (sols.Count > max) sols.RemoveRange(max, sols.Count - max);
            return sols;
        }

        public static float Distance(float[] a, float[] b)
        {
            float d = 0;
            for (int i = 0; i < a.Length; i++) d += Mathf.Abs(a[i] - b[i]) * (i < 3 ? 1.5f : 1f);
            return d;
        }

        bool Iterate(Pose target, float[] seed, out float[] th, float posTol, float rotTolDeg)
        {
            int n = joints.Length;
            th = (float[])seed.Clone();
            var J = new float[6, n];
            var e = new float[6];
            const float lambda = 0.05f, rotWeight = 0.25f, eps = 0.01f; // eps: 数值微分步长（度）
            for (int it = 0; it < 150; it++)
            {
                Pose cur = ForwardKinematics(th);
                Error(cur, target, e, rotWeight);
                float pe = Mathf.Sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
                float re = Mathf.Sqrt(e[3] * e[3] + e[4] * e[4] + e[5] * e[5]) / rotWeight * Mathf.Rad2Deg;
                if (pe < posTol && re < rotTolDeg)
                {
                    for (int i = 0; i < n; i++)
                    {
                        th[i] = Wrap(th[i], limitsDeg[i]);
                        if (th[i] < limitsDeg[i].x || th[i] > limitsDeg[i].y) return false;
                    }
                    return true;
                }
                // 数值雅可比（每列：关节 i 微小转动引起的 TCP 位姿变化）
                var de = new float[6];
                for (int i = 0; i < n; i++)
                {
                    th[i] += eps;
                    Error(ForwardKinematics(th), target, de, rotWeight);
                    th[i] -= eps;
                    for (int k = 0; k < 6; k++) J[k, i] = (e[k] - de[k]) / (eps * Mathf.Deg2Rad);
                }
                // Δθ = Jᵀ (J Jᵀ + λ² I)⁻¹ e
                var A = new float[6, 6];
                for (int r = 0; r < 6; r++)
                    for (int c = 0; c < 6; c++)
                    {
                        float s = 0;
                        for (int i = 0; i < n; i++) s += J[r, i] * J[c, i];
                        A[r, c] = s + (r == c ? lambda * lambda : 0f);
                    }
                var y = Solve6(A, e);
                if (y == null) return false;
                float maxStep = 0;
                var d = new float[n];
                for (int i = 0; i < n; i++)
                {
                    float s = 0;
                    for (int k = 0; k < 6; k++) s += J[k, i] * y[k];
                    d[i] = s * Mathf.Rad2Deg;
                    maxStep = Mathf.Max(maxStep, Mathf.Abs(d[i]));
                }
                float scale = maxStep > 15f ? 15f / maxStep : 1f;
                for (int i = 0; i < n; i++) th[i] += d[i] * scale;
            }
            return false;
        }

        static void Error(Pose cur, Pose target, float[] e, float rotWeight)
        {
            Vector3 dp = target.position - cur.position;
            Quaternion dq = target.rotation * Quaternion.Inverse(cur.rotation);
            dq.ToAngleAxis(out float ang, out Vector3 ax);
            if (ang > 180f) ang -= 360f;
            if (float.IsNaN(ax.x) || float.IsInfinity(ax.x)) { ax = Vector3.zero; ang = 0; }
            Vector3 dr = ax * (ang * Mathf.Deg2Rad * rotWeight);
            e[0] = dp.x; e[1] = dp.y; e[2] = dp.z;
            e[3] = dr.x; e[4] = dr.y; e[5] = dr.z;
        }

        static float Wrap(float a, Vector2 lim)
        {
            while (a > lim.y) a -= 360f;
            while (a < lim.x) a += 360f;
            return a;
        }

        static float[] Solve6(float[,] A, float[] b)
        {
            int n = 6;
            var M = new float[n, n + 1];
            for (int r = 0; r < n; r++) { for (int c = 0; c < n; c++) M[r, c] = A[r, c]; M[r, n] = b[r]; }
            for (int c = 0; c < n; c++)
            {
                int piv = c;
                for (int r = c + 1; r < n; r++) if (Mathf.Abs(M[r, c]) > Mathf.Abs(M[piv, c])) piv = r;
                if (Mathf.Abs(M[piv, c]) < 1e-9f) return null;
                for (int k = 0; k <= n; k++) { var t = M[c, k]; M[c, k] = M[piv, k]; M[piv, k] = t; }
                for (int r = 0; r < n; r++)
                {
                    if (r == c) continue;
                    float f = M[r, c] / M[c, c];
                    for (int k = c; k <= n; k++) M[r, k] -= f * M[c, k];
                }
            }
            var x = new float[n];
            for (int r = 0; r < n; r++) x[r] = M[r, n] / M[r, r];
            return x;
        }
    }
}
