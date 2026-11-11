using UnityEngine;

namespace Lab4
{
    /// <summary>
    /// 运行时控制面板（IMGUI）：输入支撑架位置/角度、执行抓放任务、单独控制关节与夹爪。
    /// 所有操作都通过调用控制程序完成，不直接拖动模型。
    /// </summary>
    public class ControlPanel : MonoBehaviour
    {
        public PickPlaceTask task;
        public bool visible = true;

        int tab;               // 0 自定场景 1 教师指定 2 手动控制
        Vector2 logScroll;
        readonly string[] tabs = { "自定场景", "教师指定场景", "手动控制" };
        GUIStyle box, label, small, error, success, warning;

        TaskSetup Current => tab == 1 ? task.teacher : task.custom;

        void Update()
        {
            if (Input.GetKeyDown(KeyCode.Tab)) visible = !visible;
        }

        void Styles()
        {
            if (box != null) return;
            box = new GUIStyle(GUI.skin.box) { alignment = TextAnchor.UpperLeft, padding = new RectOffset(10, 10, 8, 8) };
            label = new GUIStyle(GUI.skin.label) { fontSize = 14 };
            small = new GUIStyle(GUI.skin.label) { fontSize = 12, wordWrap = true };
            error = Colored(new Color(1f, 0.32f, 0.32f), FontStyle.Bold);
            success = Colored(new Color(0.35f, 0.9f, 0.45f), FontStyle.Bold);
            warning = Colored(new Color(1f, 0.7f, 0.25f), FontStyle.Normal);
        }

        GUIStyle Colored(Color c, FontStyle f)
        {
            var s = new GUIStyle(small) { fontStyle = f };
            s.normal.textColor = c;
            return s;
        }

        /// <summary>按内容着色：失败红色加粗、完成绿色、布置警告橙色，其余默认。</summary>
        GUIStyle StyleFor(string msg)
        {
            if (msg.Contains("失败") || msg.Contains("不可达") || msg.Contains("偏差过大")) return error;
            if (msg.Contains("完成")) return success;
            if (msg.Contains("警告")) return warning;
            return small;
        }

        void OnGUI()
        {
            if (!visible || task == null) return;
            Styles();
            float scale = Mathf.Max(1f, Screen.height / 900f);
            GUI.matrix = Matrix4x4.Scale(Vector3.one * scale);
            GUILayout.BeginArea(new Rect(10, 10, 360, Screen.height / scale - 20), box);
            GUILayout.Label("Lab4 无人机自动抓放仿真", label);
            GUILayout.Label("Tab 隐藏面板 | 右键拖动旋转 | 中键平移 | 滚轮缩放 | F1-F4 切换视角", small);
            tab = GUILayout.Toolbar(tab, tabs);
            GUILayout.Space(6);
            GUI.enabled = !task.Running;
            if (tab < 2) SetupGui(Current); else ManualGui();
            GUI.enabled = true;

            GUILayout.Space(6);
            GUILayout.Label("状态：" + task.Status, StyleFor(task.Status));
            logScroll = GUILayout.BeginScrollView(logScroll, GUILayout.Height(150));
            for (int i = task.Log.Count - 1; i >= 0; i--) GUILayout.Label(task.Log[i], StyleFor(task.Log[i]));
            GUILayout.EndScrollView();
            GUILayout.EndArea();
        }

        void SetupGui(TaskSetup s)
        {
            GUILayout.Label("立柱支撑架", label);
            PoseFields(s.column, false);
            GUILayout.Label("斜面支撑架", label);
            PoseFields(s.tilt, true);
            GUILayout.BeginHorizontal();
            GUILayout.Label("起点：", GUILayout.Width(50));
            if (GUILayout.Toggle(s.startOnTilt, "斜面→立柱", "Button")) s.startOnTilt = true;
            if (GUILayout.Toggle(!s.startOnTilt, "立柱→斜面", "Button")) s.startOnTilt = false;
            GUILayout.EndHorizontal();
            GUILayout.Space(4);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("应用布置", GUILayout.Height(28))) task.ApplySetup(s);
            if (GUILayout.Button("执行抓放", GUILayout.Height(28))) task.Run(s);
            GUILayout.EndHorizontal();
            GUI.enabled = true;
            if (GUILayout.Button("停止并复位")) task.StopAndReset(s);
        }

        void PoseFields(StandPose p, bool tilt)
        {
            string k = $"{tab}/{(tilt ? "tilt" : "column")}/";
            p.xMm = Field(k + "x", "X (mm)", p.xMm);
            p.zMm = Field(k + "z", "Z (mm)", p.zMm);
            p.yawDeg = Field(k + "yaw", "朝向 (°)", p.yawDeg);
            if (tilt) p.tiltDeg = Field(k + "tilt", "倾角 (°)", p.tiltDeg);
        }

        // 输入框保留正在输入的文字（如 "-"、空白），只有能解析为数字时才更新数值
        readonly System.Collections.Generic.Dictionary<string, string> text = new System.Collections.Generic.Dictionary<string, string>();
        readonly System.Collections.Generic.Dictionary<string, float> shown = new System.Collections.Generic.Dictionary<string, float>();

        float Field(string key, string name, float v)
        {
            if (!shown.TryGetValue(key, out var last) || last != v || !text.ContainsKey(key))
                text[key] = v.ToString("0.##");     // 数值被程序改动时刷新显示
            GUILayout.BeginHorizontal();
            GUILayout.Label(name, GUILayout.Width(70));
            string s = GUILayout.TextField(text[key], GUILayout.Width(80));
            text[key] = s;
            if (float.TryParse(s, out var r)) v = r;
            shown[key] = v;
            GUILayout.EndHorizontal();
            return v;
        }

        void ManualGui()
        {
            var robot = task.robot;
            var a = robot.Angles;
            bool changed = false;
            for (int i = 0; i < a.Length; i++)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label($"J{i + 1} {a[i],7:0.0}°", GUILayout.Width(90));
                float n = GUILayout.HorizontalSlider(a[i], robot.limitsDeg[i].x, robot.limitsDeg[i].y);
                if (!Mathf.Approximately(n, a[i])) { a[i] = n; changed = true; }
                GUILayout.EndHorizontal();
            }
            if (changed) robot.SetAngles(a);
            if (GUILayout.Button("机器人回零位")) robot.SetAngles(new float[a.Length]);

            var g = task.gripper;
            GUILayout.Label($"夹爪开口 {g.Gap * 1000f:0.0} mm", label);
            float st = GUILayout.HorizontalSlider(g.Stroke, 0f, g.maxStroke);
            if (!Mathf.Approximately(st, g.Stroke)) g.SetStroke(st);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("张开")) task.StartCoroutine(g.Open());
            if (GUILayout.Button("闭合")) task.StartCoroutine(g.Close(task.drone));
            GUILayout.EndHorizontal();

            GUILayout.Label("支撑架实时调整", label);
            var s = task.custom;
            float yaw = Slider("立柱朝向", s.column.yawDeg, -180, 180);
            float tyaw = Slider("斜面朝向", s.tilt.yawDeg, -180, 180);
            float tilt = Slider("斜面倾角", s.tilt.tiltDeg, task.tiltStand.tiltLimits.x, task.tiltStand.tiltLimits.y);
            if (yaw != s.column.yawDeg || tyaw != s.tilt.yawDeg || tilt != s.tilt.tiltDeg)
            {
                s.column.yawDeg = yaw; s.tilt.yawDeg = tyaw; s.tilt.tiltDeg = tilt;
                task.columnStand.SetPose(new Vector2(s.column.xMm, s.column.zMm) / 1000f, yaw);
                task.tiltStand.SetPose(new Vector2(s.tilt.xMm, s.tilt.zMm) / 1000f, tyaw, tilt);
            }
        }

        static float Slider(string name, float v, float lo, float hi)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"{name} {v:0}°", GUILayout.Width(110));
            v = Mathf.Round(GUILayout.HorizontalSlider(v, lo, hi));
            GUILayout.EndHorizontal();
            return v;
        }
    }
}
