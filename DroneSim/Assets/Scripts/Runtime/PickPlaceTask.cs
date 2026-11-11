using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Lab4
{
    [System.Serializable]
    public class StandPose
    {
        [Tooltip("位置 x（毫米，桌面中心为原点）")] public float xMm;
        [Tooltip("位置 z（毫米，桌面中心为原点）")] public float zMm;
        [Tooltip("朝向角（度，绕竖直轴）")] public float yawDeg;
        [Tooltip("倾斜角（度，仅斜面支撑架）")] public float tiltDeg;

        public StandPose(float x, float z, float yaw, float tilt = 0f) { xMm = x; zMm = z; yawDeg = yaw; tiltDeg = tilt; }
        public StandPose Clone() => new StandPose(xMm, zMm, yawDeg, tiltDeg);
    }

    [System.Serializable]
    public class TaskSetup
    {
        public string name = "自定场景";
        public StandPose column = new StandPose(-139, 194, 0);
        public StandPose tilt = new StandPose(132, 199, 0, 17.9f);
        [Tooltip("true：无人机从斜面支撑架搬到立柱支撑架；false：反向")]
        public bool startOnTilt = true;

        public TaskSetup Clone() => new TaskSetup
        { name = name, column = column.Clone(), tilt = tilt.Clone(), startOnTilt = startOnTilt };
    }

    /// <summary>
    /// 自动抓取-搬运-放置流程：
    /// 到达预抓取点 → 沿托盘法线直线接近 → 夹爪闭合抓住无人机 → 抬起 → 移到目标托盘上方
    /// → 沿法线直线下放 → 松开夹爪 → 退回 → 回零位。
    /// </summary>
    public class PickPlaceTask : MonoBehaviour
    {
        public RobotArm robot;
        public Gripper gripper;
        public Graspable drone;
        public SupportStand columnStand;
        public SupportStand tiltStand;

        [Header("场景参数")]
        public TaskSetup custom = new TaskSetup();
        public TaskSetup teacher = new TaskSetup { name = "教师指定场景" };

        [Header("运动参数")]
        [Tooltip("预抓取/预放置点沿托盘法线的退让距离（米），远处不可达时依次尝试更短的距离；" +
                 "最短 35 mm 时指尖仍高于无人机顶面")]
        public float[] approachDistances = { 0.08f, 0.06f, 0.045f, 0.035f };
        [Tooltip("允许无人机绕托盘法线转 180° 放置（托盘对称）")] public bool allowPadTurn = true;
        [Tooltip("判定放置成功的位置/角度误差上限")] public float placeTolMm = 2f, placeTolDeg = 1f;

        public bool Running { get; private set; }
        public string Status { get; private set; } = "就绪";
        public readonly List<string> Log = new List<string>();

        float[] home;
        float[] Home => home ??= new float[robot.Dof];   // 零位 = CAD 模型姿态

        CollisionMonitor monitor;
        public CollisionMonitor Monitor
        {
            get
            {
                if (monitor != null) return monitor;
                monitor = GetComponent<CollisionMonitor>();
                if (monitor == null) monitor = gameObject.AddComponent<CollisionMonitor>();
                monitor.robot = robot; monitor.gripper = gripper; monitor.drone = drone;
                monitor.stands = new[] { columnStand, tiltStand };
                monitor.Init();
                return monitor;
            }
        }

        void Start()
        {
            _ = Monitor;
            ApplySetup(custom);
        }

        // 运行中出现的新穿模记录到日志（零件对 + 所处步骤）
        readonly HashSet<string> reported = new HashSet<string>();
        void Update()
        {
            if (!Running) return;
            foreach (var h in Monitor.LastHits)
                if (reported.Add(h)) Warn($"⚠ 穿模：{h}（{Status}）");
        }

        void Warn(string s)
        {
            Log.Add($"[{Time.time:0.0}s] {s}");
            if (Log.Count > 40) Log.RemoveAt(0);
            Debug.LogWarning("[PickPlace] " + s);
        }

        void Say(string s)
        {
            Status = s;
            Log.Add($"[{Time.time:0.0}s] {s}");
            if (Log.Count > 40) Log.RemoveAt(0);
            Debug.Log("[PickPlace] " + s);
        }

        // ---------------------------------------------------------------- 场景布置

        /// <summary>按参数摆放两个支撑架，并把无人机放到起始支撑架上，机器人回零位、夹爪张开。</summary>
        public void ApplySetup(TaskSetup s)
        {
            if (Running) return;
            gripper.Release();
            columnStand.SetPose(new Vector2(s.column.xMm, s.column.zMm) / 1000f, s.column.yawDeg);
            tiltStand.SetPose(new Vector2(s.tilt.xMm, s.tilt.zMm) / 1000f, s.tilt.yawDeg, s.tilt.tiltDeg);
            drone.PlaceOn((s.startOnTilt ? tiltStand : columnStand).pad);
            robot.SetAngles(Home);
            gripper.SetStroke(0f);
            string warn = CheckLayout();
            Say($"已布置「{s.name}」：起点 {(s.startOnTilt ? tiltStand : columnStand).displayName}" + (warn != null ? "；警告：" + warn : ""));
        }

        string CheckLayout()
        {
            Vector2 a = columnStand.positionXZ, b = tiltStand.positionXZ;
            Vector2 robotXZ = new Vector2(robot.transform.position.x, robot.transform.position.z);
            if (Vector2.Distance(a, b) < 0.11f) return "两个支撑架底座重叠";
            if (Vector2.Distance(a, robotXZ) < 0.15f || Vector2.Distance(b, robotXZ) < 0.15f) return "支撑架离机器人底座过近";
            foreach (var p in new[] { a, b })
                if (Mathf.Abs(p.x) > 0.45f || Mathf.Abs(p.y) > 0.45f) return "支撑架超出桌面范围";
            return null;
        }

        public void Run(TaskSetup s)
        {
            if (Running) return;
            ApplySetup(s);
            StartCoroutine(RunRoutine(s));
        }

        /// <summary>返回完整流程的迭代器（供编辑器批处理验证逐帧推进）。</summary>
        public IEnumerator RunEnumerator(TaskSetup s)
        {
            ApplySetup(s);
            return RunRoutine(s);
        }

        public void StopAndReset(TaskSetup s)
        {
            StopAllCoroutines();
            Running = false;
            ApplySetup(s);
        }

        // ---------------------------------------------------------------- 主流程

        IEnumerator RunRoutine(TaskSetup s)
        {
            Running = true;
            reported.Clear();
            SupportStand from = s.startOnTilt ? tiltStand : columnStand;
            SupportStand to = s.startOnTilt ? columnStand : tiltStand;
            var planner = new PathPlanner(robot, Monitor, Home);

            // ---- 阶段 1：零位 → 抓取（转移到安全高度 → 竖直下降到预抓取点 → 沿法线接近）
            Say($"规划抓取路径（含碰撞检查）");
            var pick = new Approach();
            yield return Inline(PlanApproach(planner, from.pad, 0, false, pick));
            if (!pick.ok) { Fail($"{from.displayName}上的无人机没有无碰撞的抓取路径（{pick.reason}），请调整支撑架位置/角度"); yield break; }

            Say($"移动到{from.displayName}上方安全高度");
            yield return Execute(pick.transit);
            Say("竖直下降到预抓取点");
            yield return Execute(pick.down);
            Say("沿托盘法线直线接近无人机");
            yield return Execute(pick.approach);

            Say("夹爪闭合，抓取无人机");
            yield return gripper.Close(drone);
            if (gripper.Held != drone.transform) { Fail("未能夹住无人机"); yield break; }

            // ---- 阶段 2：沿法线退出 → 升到安全高度 → 转移到目标上方 → 下降 → 沿法线放置
            Say("规划搬运路径（含碰撞检查）");
            var lift = new Retreat();
            yield return Inline(PlanRetreat(planner, pick, lift));
            if (!lift.ok) { Fail("抓取后退出路径存在碰撞：" + lift.reason); yield break; }
            var place = new Approach();
            yield return Inline(PlanApproach(planner, to.pad, pick.variant, true, place, lift.end));
            if (!place.ok) { Fail($"{to.displayName}没有无碰撞的放置路径（{place.reason}），请调整支撑架位置/角度"); yield break; }

            Say("沿法线退出并升到安全高度");
            yield return Execute(lift.segs);
            Say($"搬运到{to.displayName}上方安全高度");
            yield return Execute(place.transit);
            Say("竖直下降到预放置点");
            yield return Execute(place.down);
            Say("沿托盘法线直线下放");
            yield return Execute(place.approach);

            Say("松开夹爪");
            gripper.Release();
            yield return gripper.Open();
            string result = Seat(to, place.turn);

            // ---- 阶段 3：沿法线退出 → 升到安全高度 → 回零位
            Say("规划返回路径（含碰撞检查）");
            var back = new Retreat();
            yield return Inline(PlanRetreat(planner, place, back, Home));
            if (!back.ok) { Fail("返回零位的路径存在碰撞：" + back.reason); yield break; }
            Say("沿法线退出并升到安全高度");
            yield return Execute(back.segs);
            Say("回到零位");
            yield return Execute(back.home);

            Running = false;
            Say(result);
        }

        // ---------------------------------------------------------------- 路径规划

        class Approach
        {
            public bool ok;
            public string reason = "不可达";
            public int variant, turn;
            public float[] qTarget;
            public Pose target, pre, high;
            public List<float[]> transit;                        // 关节空间转移（途经点）
            public PathPlanner.Segment down, approach;          // 竖直下降、沿法线接近
            public float cost = float.MaxValue;
        }

        class Retreat
        {
            public bool ok;
            public string reason;
            public float[] end;
            public List<PathPlanner.Segment> segs = new List<PathPlanner.Segment>();
            public List<float[]> home;
        }

        /// <summary>安全高度：TCP 高于最高托盘 0.2 m，此时夹持的无人机底部离托盘约 0.17 m。</summary>
        float SafeHeight => Mathf.Max(columnStand.pad.position.y, tiltStand.pad.position.y) + 0.2f;

        /// <summary>
        /// 在同一帧内直接推进嵌套的规划协程，只有遇到真正的“让出一帧”（yield null，此时机器人已恢复真实姿态）
        /// 才交还给 Unity。若交给 Unity 自己处理嵌套协程，子协程结束后父协程要等到下一帧才继续，
        /// 那一帧会渲染出规划时的虚拟姿态，表现为画面闪烁。
        /// </summary>
        static IEnumerator Inline(IEnumerator root)
        {
            var stack = new Stack<IEnumerator>();
            stack.Push(root);
            while (stack.Count > 0)
            {
                var top = stack.Peek();
                if (!top.MoveNext()) { stack.Pop(); continue; }
                if (top.Current is IEnumerator nested) { stack.Push(nested); continue; }
                yield return top.Current;
            }
        }

        class Candidate
        {
            public int v, t;
            public Pose g, pre, high;
            public float[] qh;
            public PathPlanner.Segment down, appr;
        }

        /// <summary>
        /// 规划到托盘上抓取/放置位姿的完整路径。
        /// 候选 = 退让距离 × 手指朝向 × 托盘转向 × 高处位姿的多组 IK 解，按关节转动量由小到大排序；
        /// 每个候选先做便宜的检查（下降、接近、夹持退出），再查关节转移，找到第一个可行方案即停止。
        /// 简单转移方式都失败时，才对最好的几个候选启用 RRT 搜索。整个过程分帧执行，不阻塞画面。
        /// </summary>
        IEnumerator PlanApproach(PathPlanner planner, Transform pad, int variantIn, bool keepVariant, Approach best, float[] start = null)
        {
            var saved = robot.Angles;
            start ??= saved;
            planner.displayPose = saved;
            planner.ResetBudget();
            int turns = keepVariant && allowPadTurn ? 2 : 1;
            var line = new PathPlanner.Result<PathPlanner.Segment>();
            var carry = new PathPlanner.Result<bool>();
            var tr = new PathPlanner.Result<List<float[]>>();

            foreach (float dist in approachDistances)
            {
                var cands = new List<Candidate>();
                for (int v = 0; v < 2; v++)
                {
                    if (keepVariant && v != variantIn) continue;
                    for (int t = 0; t < turns; t++)
                    {
                        Pose g = drone.GraspPoseOnPad(pad, v, t);
                        Pose pre = new Pose(g.position - (g.rotation * Vector3.forward) * dist, g.rotation);
                        Pose high = Raise(pre, start);
                        if (planner.OverBudget) yield return planner.Yield();
                        yield return planner.SolveAll(high, start, 4, tr);
                        foreach (var qh in tr.value)
                            cands.Add(new Candidate { v = v, t = t, g = g, pre = pre, high = high, qh = qh });
                    }
                }
                cands.Sort((a, b) => RobotArm.Distance(a.qh, start).CompareTo(RobotArm.Distance(b.qh, start)));

                var feasible = new List<Candidate>();
                foreach (var c in cands)
                {
                    yield return planner.Line(c.qh, c.pre, line);
                    if (!line.ok) { best.reason = "下降途中：" + planner.LastHit; continue; }
                    c.down = line.value;
                    yield return planner.Line(c.down.path[c.down.path.Count - 1], c.g, line);
                    if (!line.ok) { best.reason = "接近途中：" + planner.LastHit; continue; }
                    c.appr = line.value;
                    if (!keepVariant)
                    {
                        yield return CarryClear(planner, c.appr, c.down, carry);
                        if (!carry.ok) { best.reason = "夹持后退出途中：" + planner.LastHit; continue; }
                    }
                    feasible.Add(c);
                    yield return planner.Transit(start, c.qh, false, tr);
                    if (tr.ok) { Accept(best, c, tr.value); break; }
                    best.reason = "转移途中碰撞：" + planner.LastHit;
                }
                for (int i = 0; i < feasible.Count && i < 3 && !best.ok; i++)
                {
                    yield return planner.RrtConnect(start, feasible[i].qh, tr);
                    if (tr.ok) Accept(best, feasible[i], tr.value);
                }
                if (best.ok) break;   // 优先使用较长的退让距离
            }
            robot.SetAngles(saved);
        }

        static void Accept(Approach best, Candidate c, List<float[]> wps)
        {
            best.ok = true; best.variant = c.v; best.turn = c.t;
            best.target = c.g; best.pre = c.pre; best.high = c.high;
            best.transit = wps; best.down = c.down; best.approach = c.appr;
            best.qTarget = c.appr.path[c.appr.path.Count - 1];
        }

        /// <summary>
        /// 抓取构型的预检：虚拟地把无人机挂到夹爪上，沿“接近 + 下降”路径倒着走一遍做碰撞检查
        /// （夹住后无人机随夹爪上升，可能撞到压在上方的手臂），检查完把无人机放回原处。
        /// 分帧让出时临时把无人机放回，画面上看不到虚拟夹持。
        /// </summary>
        IEnumerator CarryClear(PathPlanner planner, PathPlanner.Segment appr, PathPlanner.Segment down, PathPlanner.Result<bool> r)
        {
            var parent = drone.transform.parent;
            var lp = drone.transform.localPosition;
            var lr = drone.transform.localRotation;
            var qg = appr.path[appr.path.Count - 1];
            void Hold() { robot.SetAngles(qg); gripper.Attach(drone.transform); }
            void PutBack()
            {
                gripper.Release();
                drone.transform.SetParent(parent, false);
                drone.transform.localPosition = lp;
                drone.transform.localRotation = lr;
            }
            Hold();
            planner.beforeYield = PutBack;
            planner.afterYield = Hold;
            r.ok = true;
            var all = new List<float[]>();
            for (int i = appr.path.Count - 1; i >= 0; i--) all.Add(appr.path[i]);
            for (int i = down.path.Count - 1; i >= 0; i--) all.Add(down.path[i]);
            foreach (var q in all)
            {
                if (!planner.Clear(q)) { r.ok = false; break; }
                if (planner.OverBudget) yield return planner.Yield();
            }
            planner.beforeYield = planner.afterYield = null;
            PutBack();
        }

        /// <summary>从托盘位姿沿法线退回预位姿，再竖直升到安全高度；homeTarget 不为空时再规划回零位。</summary>
        IEnumerator PlanRetreat(PathPlanner planner, Approach ap, Retreat r, float[] homeTarget = null)
        {
            var saved = robot.Angles;
            planner.displayPose = saved;
            planner.ResetBudget();
            var line = new PathPlanner.Result<PathPlanner.Segment>();
            yield return planner.Line(saved, ap.pre, line);
            if (!line.ok) { r.reason = planner.LastHit; robot.SetAngles(saved); yield break; }
            r.segs.Add(line.value);
            var q = line.value.path[line.value.path.Count - 1];
            if (Vector3.Distance(ap.pre.position, ap.high.position) > 0.001f)
            {
                yield return planner.Line(q, ap.high, line);
                if (!line.ok) { r.reason = planner.LastHit; robot.SetAngles(saved); yield break; }
                r.segs.Add(line.value);
                q = line.value.path[line.value.path.Count - 1];
            }
            r.end = q;
            if (homeTarget != null)
            {
                var tr = new PathPlanner.Result<List<float[]>>();
                yield return planner.Transit(q, homeTarget, true, tr);
                if (!tr.ok) { r.reason = planner.LastHit; robot.SetAngles(saved); yield break; }
                r.home = tr.value;
            }
            r.ok = true;
            robot.SetAngles(saved);
        }

        /// <summary>预位姿正上方的安全高度位姿；够不到时逐步降低，最低即预位姿本身。</summary>
        Pose Raise(Pose pre, float[] seed)
        {
            for (float h = SafeHeight; h > pre.position.y + 0.005f; h -= 0.03f)
            {
                var p = new Pose(new Vector3(pre.position.x, h, pre.position.z), pre.rotation);
                if (robot.SolveIK(p, seed, out _)) return p;
            }
            return pre;
        }

        IEnumerator Execute(List<float[]> waypoints)
        {
            foreach (var w in waypoints) yield return robot.MoveJ(w);
        }

        IEnumerator Execute(PathPlanner.Segment seg)
        {
            yield return robot.PlayPath(seg.path, seg.length / robot.linearSpeed);
        }

        IEnumerator Execute(List<PathPlanner.Segment> segs)
        {
            foreach (var s in segs) yield return Execute(s);
        }

        /// <summary>从零位出发能否在该托盘上完成抓取（含预抓取点），供可达范围扫描使用。</summary>
        public bool CanPick(Transform pad)
        {
            var saved = robot.Angles;
            robot.SetAngles(Home);
            bool ok = PlanPair(pad, 0, false, out _, out _, out _, out _, out _);
            robot.SetAngles(saved);
            return ok;
        }

        void Fail(string msg)
        {
            Running = false;
            Say("失败：" + msg);
        }

        /// <summary>
        /// 计算托盘上的抓取/放置位姿及预位姿，并求预位姿的 IK。
        /// keepVariant=true 时手指朝向沿用抓取时的选择（物体已被夹住）。
        /// </summary>
        bool PlanPair(Transform pad, int variantIn, bool keepVariant,
                      out Pose target, out Pose pre, out float[] qPre, out int variant, out int padTurn)
        {
            target = pre = default; qPre = null; variant = variantIn; padTurn = 0;
            var current = robot.Angles;
            int turns = keepVariant && allowPadTurn ? 2 : 1;
            foreach (float dist in approachDistances)
            {
                float bestCost = float.MaxValue;
                for (int v = 0; v < 2; v++)
                {
                    if (keepVariant && v != variantIn) continue;
                    for (int t = 0; t < turns; t++)
                    {
                        Pose g = drone.GraspPoseOnPad(pad, v, t);
                        Vector3 back = -(g.rotation * Vector3.forward) * dist;
                        Pose p = new Pose(g.position + back, g.rotation);
                        if (!robot.SolveIK(p, current, out var q)) continue;
                        if (!robot.SolveIK(g, q, out var qg)) continue;
                        float cost = 0;
                        for (int i = 0; i < q.Length; i++) cost += Mathf.Abs(q[i] - current[i]) + Mathf.Abs(qg[i] - q[i]) * 3f;
                        if (cost < bestCost)
                        {
                            bestCost = cost; target = g; pre = p; qPre = q; variant = v; padTurn = t;
                        }
                    }
                }
                if (qPre != null) return true;   // 优先使用较长的退让距离
            }
            return false;
        }

        /// <summary>检查无人机相对目标托盘的位姿误差，误差在容差内则视为正确落座。</summary>
        string Seat(SupportStand to, int turn)
        {
            Transform pad = to.pad;
            Quaternion want = pad.rotation * Quaternion.AngleAxis(180f * turn, Vector3.up);
            float dPos = Vector3.Distance(drone.transform.position, pad.position) * 1000f;
            float dAng = Quaternion.Angle(drone.transform.rotation, want);
            if (dPos <= placeTolMm && dAng <= placeTolDeg)
            {
                drone.PlaceOn(pad, turn);
                return $"完成：无人机已放置在{to.displayName}上（位置误差 {dPos:0.00} mm，角度误差 {dAng:0.00}°）";
            }
            return $"放置偏差过大：位置误差 {dPos:0.0} mm，角度误差 {dAng:0.0}°";
        }
    }
}
