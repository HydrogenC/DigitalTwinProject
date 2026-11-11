using UnityEngine;

namespace Lab4
{
    /// <summary>
    /// 可被夹爪抓取的物体（无人机）。
    /// 物体原点 = 放在支撑架上时与托盘坐标系重合的点；graspPoint = 抓取时 TCP 应到达的位姿。
    /// </summary>
    public class Graspable : MonoBehaviour
    {
        public Transform graspPoint;
        [Tooltip("夹持方向上的宽度（米）")] public float gripWidth = 0.106f;
        [Tooltip("判定物体位于两指之间的位置/角度容差")] public float posTolerance = 0.004f;
        public float angleTolerance = 3f;

        /// <summary>两种等价抓取位姿：绕接近方向旋转 180° 后两指互换。</summary>
        public Pose GraspPose(int variant)
        {
            Quaternion r = graspPoint.rotation;
            if (variant == 1) r = r * Quaternion.AngleAxis(180f, Vector3.forward);
            return new Pose(graspPoint.position, r);
        }

        /// <summary>若物体放在给定托盘上，对应的抓取位姿。</summary>
        public Pose GraspPoseOnPad(Transform pad, int variant, int padTurn)
        {
            Quaternion padRot = pad.rotation * Quaternion.AngleAxis(180f * padTurn, Vector3.up);
            Vector3 pos = pad.position + padRot * graspPoint.localPosition;
            Quaternion r = padRot * graspPoint.localRotation;
            if (variant == 1) r = r * Quaternion.AngleAxis(180f, Vector3.forward);
            return new Pose(pos, r);
        }

        public bool IsBetweenFingers(Transform tcp)
        {
            if (Vector3.Distance(tcp.position, graspPoint.position) > posTolerance) return false;
            // 接近方向一致，开合轴与夹持方向平行（正反皆可）
            return Vector3.Angle(tcp.forward, graspPoint.forward) < angleTolerance
                && Vector3.Angle(tcp.right, graspPoint.right) is float a && (a < angleTolerance || a > 180f - angleTolerance);
        }

        /// <summary>放到托盘上（托盘坐标系即物体落座位姿）。</summary>
        public void PlaceOn(Transform pad, int padTurn = 0)
        {
            transform.SetParent(pad, false);
            transform.localPosition = Vector3.zero;
            transform.localRotation = Quaternion.AngleAxis(180f * padTurn, Vector3.up);
        }
    }
}
