using UnityEditor;
using UnityEditor.SceneManagement;

namespace Lab4.EditorTools
{
    /// <summary>
    /// 打开工程时若当前是空白场景，自动打开 Lab4 仿真场景；
    /// 场景尚未生成（例如刚从仓库克隆，Assets/Generated 与场景不入库）时先由 ModelData 自动生成。
    /// </summary>
    [InitializeOnLoad]
    static class OpenLab4Scene
    {
        const string ScenePath = "Assets/Scenes/Lab4.unity";

        static OpenLab4Scene()
        {
            EditorApplication.delayCall += () =>
            {
                if (UnityEngine.Application.isBatchMode || EditorApplication.isPlayingOrWillChangePlaymode) return;
                var s = EditorSceneManager.GetActiveScene();
                if (!string.IsNullOrEmpty(s.path)) return;
                if (!System.IO.File.Exists(ScenePath)) SceneBuilder.Build();
                EditorSceneManager.OpenScene(ScenePath);
            };
        }
    }
}
