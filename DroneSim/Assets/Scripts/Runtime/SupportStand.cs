using UnityEngine;

namespace Lab4
{
    /// <summary>
    /// 支撑架：程序设置其在桌面上的位置（x, z，单位米，以桌面中心为原点）和绕竖直轴的朝向角；
    /// 斜面支撑架还可设置托盘绕铰链的倾斜角。pad 为托盘顶面坐标系（无人机落座位姿）。
    /// </summary>
    public class SupportStand : MonoBehaviour
    {
        public string displayName = "支撑架";
        public Transform pad;

        [Header("位姿参数")]
        public Vector2 positionXZ;
        public float yawDeg;
        [Tooltip("倾斜角（度），仅斜面支撑架有效")] public float tiltDeg;

        [Header("斜面支撑架")]
        public bool canTilt;
        public Transform hinge;
        public Vector3 hingeAxisLocal = Vector3.forward;
        [Tooltip("CAD 模型中铰链所处的倾角")] public float modelTiltDeg;
        [Tooltip("倾角正方向与铰链轴旋转方向的符号关系（由场景生成器测得）")] public float tiltSign = 1f;
        public Vector2 tiltLimits = new Vector2(-45f, 45f);

        public float tableHeight;

        public void SetPose(Vector2 xz, float yaw, float tilt = 0f)
        {
            positionXZ = xz;
            yawDeg = yaw;
            tiltDeg = tilt;
            Apply();
        }

        /// <summary>把参数写入场景（位置、朝向、倾角）。</summary>
        public void Apply()
        {
            transform.position = new Vector3(positionXZ.x, tableHeight, positionXZ.y);
            transform.rotation = Quaternion.AngleAxis(yawDeg, Vector3.up);
            if (canTilt && hinge != null)
            {
                tiltDeg = Mathf.Clamp(tiltDeg, tiltLimits.x, tiltLimits.y);
                hinge.localRotation = Quaternion.AngleAxis(tiltSign * (tiltDeg - modelTiltDeg), hingeAxisLocal);
            }
        }
    }
}
