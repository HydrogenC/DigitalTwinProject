using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

namespace Lab4
{
    /// <summary>
    /// 带碰撞检查的路径规划：所有候选路径先在虚拟状态下逐点检查（设置关节角 → 碰撞检测），
    /// 只有整段无碰撞才会被采用。
    /// 规划以协程方式分帧执行：每帧计算不超过 frameBudgetMs，超时就把机器人恢复到真实姿态并让出一帧，
    /// 避免画面卡顿。
    /// </summary>
    public class PathPlanner
    {
        readonly RobotArm robot;
        readonly CollisionMonitor monitor;
        readonly float[] upright;             // 收拢姿态（零位的 J2..J6）：手臂竖直、最紧凑
        public string LastHit { get; private set; }

        public float jointStepDeg = 2f;       // 关节插补检查步长
        public float lineStepM = 0.004f;      // 直线运动检查步长
        public float frameBudgetMs = 8f;      // 每帧规划计算时间上限
        public int rrtIterations = 600;
        public float rrtStepDeg = 10f;

        /// <summary>让出一帧前需要恢复的显示状态（真实姿态）；规划期间由调用方设置。</summary>
        public float[] displayPose;
        /// <summary>让出一帧前/恢复后的额外处理（如虚拟夹持时临时放回无人机）。</summary>
        public System.Action beforeYield, afterYield;

        readonly Stopwatch sw = new Stopwatch();

        public PathPlanner(RobotArm robot, CollisionMonitor monitor, float[] upright)
        {
            this.robot = robot; this.monitor = monitor; this.upright = upright;
            sw.Start();
        }

        /// <summary>一段运动：关节序列 + 是否为直线运动（直线段按距离计时）。</summary>
        public class Segment
        {
            public List<float[]> path;
            public bool linear;
            public float length;               // 直线段：TCP 移动距离（米）
        }

        /// <summary>协程结果容器。</summary>
        public class Result<T> { public bool ok; public T value; }

        // ---------------------------------------------------------------- 分帧

        /// <summary>本帧计算是否已超时（超时则调用方应 yield return Yield()）。</summary>
        public bool OverBudget => sw.Elapsed.TotalMilliseconds > frameBudgetMs;

        /// <summary>恢复真实姿态后让出一帧，下一帧重新开始计时。</summary>
        public IEnumerator Yield()
        {
            beforeYield?.Invoke();
            if (displayPose != null) robot.SetAngles(displayPose);
            yield return null;
            afterYield?.Invoke();
            sw.Restart();
        }

        public void ResetBudget() => sw.Restart();

        /// <summary>分帧版的多解 IK（见 RobotArm.SolveIKAll），结果按离 seed 由近到远排序。</summary>
        public IEnumerator SolveAll(Pose target, float[] seed, int max, Result<List<float[]>> r)
        {
            var sols = new List<float[]>();
            foreach (var s in robot.IKSeeds(seed))
            {
                robot.AddSolution(target, s, sols);
                if (OverBudget) yield return Yield();
            }
            r.ok = sols.Count > 0;
            r.value = RobotArm.SortSolutions(sols, seed, max);
        }

        // ---------------------------------------------------------------- 检查

        public bool Clear(float[] q)
        {
            robot.SetAngles(q);
            var hits = monitor.Check();
            if (hits.Count > 0) { LastHit = hits[0]; return false; }
            return true;
        }

        /// <summary>关节空间直线插补 a→b 是否全程无碰撞。</summary>
        public IEnumerator JointClear(float[] a, float[] b, Result<bool> r)
        {
            r.ok = false;
            float max = 0;
            for (int i = 0; i < a.Length; i++) max = Mathf.Max(max, Mathf.Abs(b[i] - a[i]));
            int n = Mathf.Max(1, Mathf.CeilToInt(max / jointStepDeg));
            var q = new float[a.Length];
            // 先查终点和中点，碰撞多发生在这些位置，可以尽早排除
            foreach (float t in new[] { 1f, 0.5f })
            {
                for (int i = 0; i < q.Length; i++) q[i] = Mathf.Lerp(a[i], b[i], t);
                if (!Clear(q)) yield break;
            }
            for (int k = 1; k < n; k++)
            {
                float t = (float)k / n;
                for (int i = 0; i < q.Length; i++) q[i] = Mathf.Lerp(a[i], b[i], t);
                if (!Clear(q)) yield break;
                if (OverBudget) yield return Yield();
            }
            r.ok = true;
        }

        /// <summary>
        /// 从关节角 qa 出发，TCP 沿直线移动到 target，逐点求 IK（以上一点为初值，保持构型连续）并检查碰撞。
        /// </summary>
        public IEnumerator Line(float[] qa, Pose target, Result<Segment> r)
        {
            r.ok = false;
            var seg = new Segment { linear = true, path = new List<float[]> { qa } };
            Pose start = robot.ForwardKinematics(qa);
            float dist = Vector3.Distance(start.position, target.position);
            float ang = Quaternion.Angle(start.rotation, target.rotation);
            int n = Mathf.Max(1, Mathf.CeilToInt(Mathf.Max(dist / lineStepM, ang / jointStepDeg)));
            seg.length = dist;
            var q = qa;
            for (int k = 1; k <= n; k++)
            {
                float t = (float)k / n;
                var p = new Pose(Vector3.Lerp(start.position, target.position, t),
                                 Quaternion.Slerp(start.rotation, target.rotation, t));
                if (!robot.SolveIK(p, q, out var next)) { LastHit = "直线路径上存在不可达点"; yield break; }
                // 构型突变（相邻两点关节角跳变过大）说明经过奇异点，视为不可行
                for (int i = 0; i < next.Length; i++)
                    if (Mathf.Abs(next[i] - q[i]) > 20f) { LastHit = "直线路径经过奇异位形"; yield break; }
                if (!Clear(next)) yield break;
                seg.path.Add(next);
                q = next;
                if (OverBudget) yield return Yield();
            }
            r.ok = true;
            r.value = seg;
        }

        /// <summary>
        /// 关节空间转移 qa→qb：依次尝试
        ///  1) 直接插补；
        ///  2) 先在原位调整手臂/手腕，再转底座 J1；
        ///  3) 先转底座 J1，再调整手臂/手腕；
        ///  4) 收拢到竖直姿态 → 转底座 → 展开到目标（最保守）；
        ///  5) allowRrt 时用 RRT-Connect 随机搜索。
        /// 结果为途经点序列（不含起点）。
        /// </summary>
        public IEnumerator Transit(float[] qa, float[] qb, bool allowRrt, Result<List<float[]>> r)
        {
            r.ok = false;
            var options = new List<List<float[]>>
            {
                new List<float[]> { qb },
                new List<float[]> { With(qa[0], qb), qb },
                new List<float[]> { With(qb[0], qa), qb },
                new List<float[]> { With(qa[0], upright), With(qb[0], upright), qb },
            };
            var seg = new Result<bool>();
            foreach (var opt in options)
            {
                var prev = qa;
                bool ok = true;
                foreach (var w in opt)
                {
                    yield return JointClear(prev, w, seg);
                    if (!seg.ok) { ok = false; break; }
                    prev = w;
                }
                if (ok) { r.ok = true; r.value = opt; yield break; }
            }
            if (allowRrt) yield return RrtConnect(qa, qb, r);
        }

        static float[] With(float j1, float[] rest)
        {
            var q = (float[])rest.Clone();
            q[0] = j1;
            return q;
        }

        public static float JointLength(List<float[]> pts, float[] from)
        {
            float d = 0;
            var prev = from;
            foreach (var p in pts) { d += RobotArm.Distance(prev, p); prev = p; }
            return d;
        }

        // ---------------------------------------------------------------- RRT-Connect

        class Node { public float[] q; public Node parent; }

        /// <summary>
        /// 双向快速随机树：起点树与终点树交替向随机采样点扩展，并尝试互相连接；
        /// 找到路径后做捷径化简（能直接连通的途经点之间去掉中间点）。
        /// </summary>
        public IEnumerator RrtConnect(float[] qa, float[] qb, Result<List<float[]>> r)
        {
            r.ok = false;
            if (!Clear(qb)) yield break;
            var rng = new System.Random(12345);
            var ta = new List<Node> { new Node { q = qa } };
            var tb = new List<Node> { new Node { q = qb } };
            bool swapped = false;
            var edge = new Result<bool>();
            for (int it = 0; it < rrtIterations; it++)
            {
                var sample = new float[qa.Length];
                for (int i = 0; i < sample.Length; i++)
                    sample[i] = Mathf.Lerp(robot.limitsDeg[i].x, robot.limitsDeg[i].y, (float)rng.NextDouble());

                // 扩展：树 ta 向采样点走一步
                var near = Nearest(ta, sample);
                var qn = Step(near.q, sample);
                yield return JointClear(near.q, qn, edge);
                if (edge.ok)
                {
                    var na = new Node { q = qn, parent = near };
                    ta.Add(na);
                    // 连接：树 tb 朝新节点一直走，直到相接或被挡住
                    var cur = Nearest(tb, na.q);
                    for (int k = 0; k < 60; k++)
                    {
                        var qs = Step(cur.q, na.q);
                        yield return JointClear(cur.q, qs, edge);
                        if (!edge.ok) break;
                        var nb = new Node { q = qs, parent = cur };
                        tb.Add(nb);
                        cur = nb;
                        if (MaxDiff(qs, na.q) < 1e-3f)
                        {
                            var path = Trace(swapped ? nb : na, swapped ? na : nb);
                            var pts = new List<float[]> { qa };
                            pts.AddRange(path);
                            yield return Shortcut(pts, rng);
                            pts.RemoveAt(0);
                            r.ok = true; r.value = pts;
                            yield break;
                        }
                    }
                }
                (ta, tb) = (tb, ta);
                swapped = !swapped;
            }
            LastHit = "关节空间搜索未找到无碰撞路径";
        }

        Node Nearest(List<Node> tree, float[] q)
        {
            Node best = null; float bd = float.MaxValue;
            foreach (var n in tree) { float d = MaxDiff(n.q, q); if (d < bd) { bd = d; best = n; } }
            return best;
        }

        static float MaxDiff(float[] a, float[] b)
        {
            float d = 0;
            for (int i = 0; i < a.Length; i++) d = Mathf.Max(d, Mathf.Abs(a[i] - b[i]));
            return d;
        }

        float[] Step(float[] from, float[] to)
        {
            float d = MaxDiff(from, to);
            var q = (float[])to.Clone();
            if (d > rrtStepDeg)
                for (int i = 0; i < q.Length; i++) q[i] = from[i] + (to[i] - from[i]) * rrtStepDeg / d;
            return q;
        }

        /// <summary>起点树节点 a 与终点树节点 b 相接：a→起点反向 + b→终点。返回不含起点的途经点。</summary>
        static List<float[]> Trace(Node a, Node b)
        {
            var head = new List<float[]>();
            for (var n = a; n != null; n = n.parent) head.Add(n.q);
            head.Reverse();
            head.RemoveAt(0);
            for (var n = b.parent; n != null; n = n.parent) head.Add(n.q);
            return head;
        }

        IEnumerator Shortcut(List<float[]> pts, System.Random rng)
        {
            var edge = new Result<bool>();
            for (int k = 0; k < 40 && pts.Count > 2; k++)
            {
                int i = rng.Next(pts.Count - 2), j = rng.Next(i + 2, pts.Count);
                yield return JointClear(pts[i], pts[j], edge);
                if (edge.ok) pts.RemoveRange(i + 1, j - i - 1);
            }
        }
    }
}
