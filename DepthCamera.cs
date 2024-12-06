using UnityEngine;

[RequireComponent(typeof(Camera))]
public class DepthCamera : MonoBehaviour
{
    public Camera depthCamera; // 深度カメラをInspectorで割り当て
    // public RenderTexture depthTexture; // 深度カメラのRenderTextureを割り当て

    void Start()
    {
        if (depthCamera != null)
        {
            // RenderTextureを深度カメラに割り当て
            // depthCamera.targetTexture = depthTexture;
            depthCamera.depthTextureMode = DepthTextureMode.Depth;
        }
    }

    // void Update()
    // {
    //     // RenderTextureをアクティブに設定
    //     RenderTexture.active = depthTexture;

    //     // RenderTextureのサイズに基づいてTexture2Dを作成
    //     Texture2D depthImage = new Texture2D(depthTexture.width, depthTexture.height, TextureFormat.RFloat, false);

    //     // RenderTextureからピクセルデータを読み込む
    //     depthImage.ReadPixels(new Rect(0, 0, depthTexture.width, depthTexture.height), 0, 0);
    //     depthImage.Apply();

    //     // RenderTextureのアクティブ状態を解除
    //     RenderTexture.active = null;

    //     // 深度情報をデバッグ出力
    //     DebugDepthData(depthImage);
    // }

    // private void DebugDepthData(Texture2D depthImage)
    // {
    //     int width = depthImage.width;
    //     int height = depthImage.height;

    //     // 深度情報を1ピクセルずつ出力
    //     for (int y = 0; y < height; y++)
    //     {
    //         for (int x = 0; x < width; x++)
    //         {
    //             float depthValue = depthImage.GetPixel(x, y).r; // Rチャネルに深度が格納される
    //             Debug.Log($"Pixel ({x}, {y}): Depth = {depthValue}");
    //         }
    //     }
    // }
}

