using UnityEngine;
using TMPro; // Import the TextMeshPro namespace

public class FPSCounter : MonoBehaviour
{
    public TextMeshProUGUI fpsText; // Change the type to TextMeshProUGUI
    private float deltaTime = 0.0f;

    void Update()
    {
        deltaTime += (Time.unscaledDeltaTime - deltaTime) * 0.1f;
        float fps = 1.0f / deltaTime;
        fpsText.text = Mathf.Ceil(fps).ToString() + " FPS";
    }
}