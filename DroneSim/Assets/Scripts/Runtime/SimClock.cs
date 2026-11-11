using UnityEngine;

namespace Lab4
{
    /// <summary>运动插补用的时间步长：运行时取帧间隔，编辑器批处理验证时取固定步长。</summary>
    public static class SimClock
    {
        public static float Dt => Application.isPlaying ? Time.deltaTime : 0.02f;
    }
}
