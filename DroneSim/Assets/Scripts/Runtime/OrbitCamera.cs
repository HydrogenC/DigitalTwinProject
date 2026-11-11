using UnityEngine;

namespace Lab4
{
    /// <summary>环绕相机：右键旋转、中键平移、滚轮缩放；F1-F4 切换预设视角（F4 跟随夹爪）。</summary>
    public class OrbitCamera : MonoBehaviour
    {
        public Transform follow;          // F4 跟随目标（TCP）
        public Vector3 pivot = new Vector3(0f, 0.15f, 0f);
        public float distance = 1.6f, yaw = 210f, pitch = 35f;

        bool following;

        struct View { public Vector3 p; public float d, yaw, pitch; }
        readonly View[] views =
        {
            new View { p = new Vector3(0f, 0.15f, 0f), d = 1.6f, yaw = 210f, pitch = 35f },     // 全景
            new View { p = new Vector3(0f, 0.3f, -0.2f), d = 1.1f, yaw = 150f, pitch = 20f },   // 机器人
            new View { p = new Vector3(0f, 0.1f, 0.2f), d = 0.65f, yaw = 180f, pitch = 30f },   // 支撑架
        };

        void Start()
        {
            // 画质：8x 多重采样抗锯齿、高分辨率近距离阴影、各向异性过滤（场景只有 1 m 见方，阴影距离不需要很远）
            QualitySettings.antiAliasing = 8;
            QualitySettings.anisotropicFiltering = AnisotropicFiltering.ForceEnable;
            QualitySettings.shadows = ShadowQuality.All;
            QualitySettings.shadowResolution = ShadowResolution.VeryHigh;
            QualitySettings.shadowDistance = 4f;
            QualitySettings.shadowCascades = 2;
            QualitySettings.pixelLightCount = 4;
            var cam = GetComponent<Camera>();
            cam.allowMSAA = true;
            // 裁剪面收紧到场景尺度：0.01~1000 m 时深度精度不足，桌面与贴近它的绿色垫面会互相闪烁（z-fighting）
            cam.nearClipPlane = 0.02f;
            cam.farClipPlane = 30f;
            QualitySettings.shadowProjection = ShadowProjection.StableFit;
            Apply();
        }

        void LateUpdate()
        {
            for (int i = 0; i < views.Length; i++)
                if (Input.GetKeyDown(KeyCode.F1 + i))
                {
                    following = false;
                    pivot = views[i].p; distance = views[i].d; yaw = views[i].yaw; pitch = views[i].pitch;
                }
            if (Input.GetKeyDown(KeyCode.F4) && follow != null) { following = true; distance = 0.45f; }

            if (Input.GetMouseButton(1))
            {
                yaw += Input.GetAxis("Mouse X") * 4f;
                pitch = Mathf.Clamp(pitch - Input.GetAxis("Mouse Y") * 4f, -10f, 89f);
            }
            if (Input.GetMouseButton(2))
            {
                following = false;
                pivot -= (transform.right * Input.GetAxis("Mouse X") + transform.up * Input.GetAxis("Mouse Y")) * distance * 0.05f;
            }
            distance = Mathf.Clamp(distance * (1f - Input.GetAxis("Mouse ScrollWheel")), 0.1f, 5f);
            if (following && follow != null) pivot = follow.position;
            Apply();
        }

        void Apply()
        {
            Quaternion r = Quaternion.Euler(pitch, yaw, 0f);
            transform.SetPositionAndRotation(pivot - r * Vector3.forward * distance, r);
        }
    }
}
