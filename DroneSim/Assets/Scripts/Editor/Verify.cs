using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Lab4.EditorTools
{
    /// <summary>
    /// 批处理验证：在编辑器中逐帧推进完整抓放流程（不进入 Play 模式），输出结果与关键帧截图。
    /// 命令行：Unity -batchmode -projectPath . -executeMethod Lab4.EditorTools.Verify.Run -quit
    /// </summary>
    public static class Verify
    {
        static string ShotDir => Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Screenshots"));

        [MenuItem("Lab4/批处理验证（自定 + 教师 + 随机）")]
        public static void Run()
        {
            SceneBuilder.Build();
            SceneBuilder.EnsureScene();
            EditorSceneManager.OpenScene("Assets/Scenes/Lab4.unity");
            var task = Object.FindFirstObjectByType<PickPlaceTask>();
            Directory.CreateDirectory(ShotDir);

            var hits = new Dictionary<string, string>();
            bool a = RunOne(task, task.custom, "custom", true, hits);
            bool b = RunOne(task, task.teacher, "teacher", true, hits);

            // 随机布置压力测试：支撑架位置、朝向、倾角随机，两个搬运方向都测
            var rng = new System.Random(20261104);
            int ok = 0, total = 0, skipped = 0, clean = 0;
            var fails = new List<string>();
            var stepStats = new Dictionary<string, int>();
            while (total < 60)
            {
                var s = new TaskSetup
                {
                    name = "随机" + (total + 1),
                    column = new StandPose(R(rng, -350, 350), R(rng, 20, 380), R(rng, -180, 180)),
                    tilt = new StandPose(R(rng, -350, 350), R(rng, 20, 380), R(rng, -180, 180), R(rng, -35, 35)),
                    startOnTilt = rng.Next(2) == 0,
                };
                task.ApplySetup(s);
                if (task.Status.Contains("警告")) { skipped++; continue; }
                total++;
                var h = new Dictionary<string, string>();
                bool success = RunOne(task, s, null, false, h);
                string desc = $"{s.name} 立柱({s.column.xMm:0},{s.column.zMm:0},{s.column.yawDeg:0}°) 斜面({s.tilt.xMm:0},{s.tilt.zMm:0},{s.tilt.yawDeg:0}°,倾角{s.tilt.tiltDeg:0}°) 起点{(s.startOnTilt ? "斜面" : "立柱")}";
                if (success) ok++;
                else fails.Add($"{desc} -> {task.Status}");
                if (success && h.Count == 0) clean++;
                foreach (var kv in h)
                {
                    string key = $"{kv.Value} | {kv.Key}";
                    stepStats[key] = stepStats.TryGetValue(key, out var c) ? c + 1 : 1;
                }
            }
            Debug.Log($"[Verify] 自定场景：{(a ? "成功" : "失败")}；教师默认场景：{(b ? "成功" : "失败")}；" +
                      $"随机布置 {ok}/{total} 成功，其中无穿模 {clean} 个（跳过 {skipped} 个重叠/越界布置）");
            foreach (var kv in hits) Debug.Log($"[Verify] 穿模(自定/教师) {kv.Value} | {kv.Key}");
            foreach (var kv in stepStats) Debug.Log($"[Verify] 穿模(随机) ×{kv.Value} {kv.Key}");
            foreach (var f in fails) Debug.Log("[Verify] 失败 " + f);
            task.ApplySetup(task.custom);
            Shot("final_layout");
        }

        /// <summary>可达范围扫描：托盘放在离机器人底座不同距离、不同方位/朝向/倾角时，抓取位姿是否有 IK 解。</summary>
        [MenuItem("Lab4/可达范围扫描")]
        public static void Reach()
        {
            SceneBuilder.EnsureScene();
            EditorSceneManager.OpenScene("Assets/Scenes/Lab4.unity");
            var task = Object.FindFirstObjectByType<PickPlaceTask>();
            var robot = task.robot;
            var stand = task.tiltStand;
            Vector3 b = robot.transform.position;
            var sb = new System.Text.StringBuilder("[Reach] 距离(mm) | 倾角0° | 倾角±20° | 倾角±35°  （各为方位×朝向组合中可达的比例）\n");
            for (int d = 250; d <= 650; d += 25)
            {
                sb.Append($"[Reach] {d,4} |");
                foreach (float tilt in new[] { 0f, 20f, 35f })
                {
                    int ok = 0, n = 0;
                    for (int az = -60; az <= 60; az += 30)          // 机器人前方 ±60° 扇区
                        for (int yaw = 0; yaw < 360; yaw += 45)
                            foreach (float sgn in tilt == 0 ? new[] { 1f } : new[] { 1f, -1f })
                            {
                                Vector3 dir = Quaternion.AngleAxis(az, Vector3.up) * Vector3.forward;
                                Vector3 p = b + dir * (d / 1000f);
                                stand.SetPose(new Vector2(p.x, p.z), yaw, tilt * sgn);
                                n++;
                                if (task.CanPick(stand.pad)) ok++;
                            }
                    sb.Append($" {100f * ok / n,5:0}% |");
                }
                sb.Append('\n');
            }
            Debug.Log(sb.ToString());
        }

        /// <summary>性能剖析：单次碰撞检测耗时、各步骤耗时及单帧最长耗时，并检查规划阶段是否有虚拟姿态被渲染。</summary>
        [MenuItem("Lab4/性能剖析")]
        public static void Profile()
        {
            SceneBuilder.EnsureScene();
            EditorSceneManager.OpenScene("Assets/Scenes/Lab4.unity");
            var task = Object.FindFirstObjectByType<PickPlaceTask>();
            task.ApplySetup(task.custom);
            var mon = task.Monitor;
            var sw = System.Diagnostics.Stopwatch.StartNew();
            for (int i = 0; i < 200; i++) mon.Check();
            Debug.Log($"[Prof] Check() 平均 {sw.Elapsed.TotalMilliseconds / 200:0.00} ms");

            foreach (var setup in new[] { task.custom, task.teacher })
            {
                var it = task.RunEnumerator(setup);
                var stack = new Stack<IEnumerator>();
                stack.Push(it);
                var perStep = new Dictionary<string, double>();
                var maxFrame = new Dictionary<string, double>();
                var frame = System.Diagnostics.Stopwatch.StartNew();
                float[] planPose = null;
                float leak = 0f;   // 规划阶段渲染帧与真实姿态的最大关节偏差（应为 0：虚拟姿态不应被渲染）
                Vector3 dronePos = Vector3.zero;
                float droneLeak = 0f;
                while (stack.Count > 0)
                {
                    var top = stack.Peek();
                    string st = task.Status;
                    bool more = top.MoveNext();
                    if (!more) { stack.Pop(); continue; }
                    if (top.Current is IEnumerator nested) { stack.Push(nested); continue; }
                    if (task.Status.StartsWith("规划"))
                    {
                        if (planPose == null) { planPose = task.robot.Angles; dronePos = task.drone.transform.position; }
                        var shown = task.robot.Angles;
                        for (int i = 0; i < shown.Length; i++) leak = Mathf.Max(leak, Mathf.Abs(shown[i] - planPose[i]));
                        droneLeak = Mathf.Max(droneLeak, Vector3.Distance(dronePos, task.drone.transform.position));
                    }
                    else planPose = null;
                    double ms = frame.Elapsed.TotalMilliseconds;
                    frame.Restart();
                    perStep[st] = perStep.TryGetValue(st, out var a) ? a + ms : ms;
                    maxFrame[st] = maxFrame.TryGetValue(st, out var m) ? System.Math.Max(m, ms) : ms;
                }
                Debug.Log($"[Prof] ===== {setup.name}：{task.Status}");
                Debug.Log($"[Prof] 规划阶段渲染帧姿态偏差：关节最大 {leak:0.000}°，无人机最大 {droneLeak * 1000:0.000} mm（0 表示没有虚拟姿态被渲染）");
                foreach (var kv in perStep) Debug.Log($"[Prof] {kv.Key,-24} 合计 {kv.Value,8:0} ms  单帧最长 {maxFrame[kv.Key],7:0} ms");
            }
        }

        static float R(System.Random r, float lo, float hi) => Mathf.Round((float)(lo + r.NextDouble() * (hi - lo)));

        /// <summary>逐帧推进一次完整流程；hits 收集出现过的穿模零件对及其首次出现的步骤。</summary>
        static bool RunOne(PickPlaceTask task, TaskSetup s, string shotPrefix, bool log, Dictionary<string, string> hits)
        {
            var monitor = task.Monitor;
            var it = task.RunEnumerator(s);
            if (shotPrefix != null) Shot(shotPrefix + "_0_start");
            string last = task.Status;
            int frames = 0, shot = 1;
            var stack = new Stack<IEnumerator>();
            stack.Push(it);
            while (stack.Count > 0 && frames < 200000)
            {
                var top = stack.Peek();
                if (!top.MoveNext()) { stack.Pop(); continue; }
                if (top.Current is IEnumerator nested) { stack.Push(nested); continue; }
                frames++;
                foreach (var h in monitor.Check())
                {
                    if (hits.ContainsKey(h)) continue;
                    hits[h] = task.Status;
                    if (shotPrefix != null && hits.Count <= 3) Shot($"{shotPrefix}_collision_{hits.Count}");
                }
                if (task.Status != last)
                {
                    last = task.Status;
                    if (shotPrefix != null && (last.Contains("夹爪闭合") || last.Contains("松开") || last.Contains("搬运")))
                        Shot($"{shotPrefix}_{shot++}");
                }
            }
            if (shotPrefix != null) Shot(shotPrefix + "_9_done");
            if (log)
            {
                Debug.Log($"[Verify] ===== {s.name} =====");
                foreach (var l in task.Log) Debug.Log("[Verify]   " + l);
            }
            return task.Status.StartsWith("完成");
        }

        public static void Shot(string name)
        {
            var camGo = new GameObject("ShotCam");
            var cam = camGo.AddComponent<Camera>();
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0.86f, 0.88f, 0.91f);
            cam.nearClipPlane = 0.01f;
            cam.fieldOfView = 40f;
            var task = Object.FindFirstObjectByType<PickPlaceTask>();
            Vector3 focus = (task.columnStand.transform.position + task.tiltStand.transform.position) * 0.5f + Vector3.up * 0.15f;
            focus = Vector3.Lerp(focus, task.robot.transform.position + Vector3.up * 0.25f, 0.35f);
            camGo.transform.position = focus + new Vector3(0.95f, 0.55f, 0.75f);
            camGo.transform.LookAt(focus);
            var rt = new RenderTexture(1280, 860, 24) { antiAliasing = 4 };
            cam.targetTexture = rt;
            cam.Render();
            RenderTexture.active = rt;
            var tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
            tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
            tex.Apply();
            File.WriteAllBytes(Path.Combine(ShotDir, name + ".png"), tex.EncodeToPNG());
            RenderTexture.active = null;
            cam.targetTexture = null;
            Object.DestroyImmediate(rt);
            Object.DestroyImmediate(tex);
            Object.DestroyImmediate(camGo);
        }
    }
}
