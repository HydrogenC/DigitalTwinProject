using System.Collections;
using UnityEngine;

namespace Lab4
{
    /// <summary>
    /// 气动平行夹爪：两指沿开合轴对称运动；抓住物体后把物体挂到 TCP 下随机器人运动。
    /// stroke = 每根手指相对 CAD 位置（全开）向内移动的距离（米）。
    /// </summary>
    public class Gripper : MonoBehaviour
    {
        public Transform jawA;            // 位于开合轴负方向的手指，闭合时沿 +jawAxis 移动
        public Transform jawB;            // 位于开合轴正方向的手指，闭合时沿 -jawAxis 移动
        public Vector3 jawAxisLocal = Vector3.right;   // 开合轴（父节点坐标系）
        public Transform tcp;
        [Tooltip("全开时两指垫内侧间距（米）")] public float openGap = 0.1134f;
        [Tooltip("单指最大行程（米）")] public float maxStroke = 0.012f;
        [Tooltip("单指运动速度（米/秒）")] public float speed = 0.02f;

        [SerializeField] float stroke;
        public float Stroke => stroke;
        public float Gap => openGap - 2f * stroke;
        public Transform Held { get; private set; }
        public bool IsMoving { get; private set; }

        Vector3 jawA0, jawB0;
        bool cached;

        void Awake() => Cache();

        void Cache()
        {
            if (cached) return;
            jawA0 = jawA.localPosition;
            jawB0 = jawB.localPosition;
            cached = true;
        }

        /// <summary>直接设置单指行程（0 = 全开）。</summary>
        public void SetStroke(float s)
        {
            Cache();
            stroke = Mathf.Clamp(s, 0f, maxStroke);
            Vector3 d = jawAxisLocal.normalized * stroke;
            jawA.localPosition = jawA0 + d;
            jawB.localPosition = jawB0 - d;
        }

        public IEnumerator Open() => MoveTo(0f);

        /// <summary>闭合夹爪；若两指之间有可抓取物体，则在接触时停止并夹住它。</summary>
        public IEnumerator Close(Graspable candidate = null)
        {
            float target = maxStroke;
            if (candidate != null && candidate.IsBetweenFingers(tcp))
                target = Mathf.Clamp((openGap - candidate.gripWidth) * 0.5f, 0f, maxStroke);
            yield return MoveTo(target);
            if (candidate != null && candidate.IsBetweenFingers(tcp) && target < maxStroke)
                Attach(candidate.transform);
        }

        IEnumerator MoveTo(float target)
        {
            IsMoving = true;
            while (!Mathf.Approximately(stroke, target))
            {
                SetStroke(Mathf.MoveTowards(stroke, target, speed * SimClock.Dt));
                yield return null;
            }
            IsMoving = false;
        }

        public void Attach(Transform obj)
        {
            Held = obj;
            obj.SetParent(tcp, true);
        }

        public Transform Release()
        {
            var obj = Held;
            if (obj != null) obj.SetParent(null, true);
            Held = null;
            return obj;
        }
    }
}
