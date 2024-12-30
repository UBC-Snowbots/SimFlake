using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ArUcoGenerator : MonoBehaviour
{
    public List<Texture2D> arucoMarkerTextures; // List of ARUCO marker textures
    public GameObject existingCube; // Reference to the existing cube
    private Material arucoMaterial;

    void Start()
    {
        if (existingCube == null)
        {
            Debug.LogError("No cube assigned!");
            return;
        }

        // Initialize the list of ARUCO marker textures
        arucoMarkerTextures = new List<Texture2D>();

        // Load each ARUCO marker texture manually
        for (int i = 0; i < 50; i++)
        {
            Texture2D texture = Resources.Load<Texture2D>($"Textures/Aruco/aruco{i}");
            if (texture != null)
            {
                arucoMarkerTextures.Add(texture);
            }
            else
            {
                Debug.LogWarning($"Texture aruco{i} not found in Resources/Textures/Aruco!");
            }
        }

        Debug.Log("Number of ARUCO marker textures loaded: " + arucoMarkerTextures.Count);

        if (arucoMarkerTextures.Count == 0)
        {
            Debug.LogError("No ARUCO marker textures found in Resources/Textures/Aruco!");
            return;
        }

        // Create a new material
        arucoMaterial = new Material(Shader.Find("Standard"));

        // Assign the material to the existing cube
        existingCube.GetComponent<Renderer>().material = arucoMaterial;

        // Start the coroutine to change the texture every 30 seconds
        StartCoroutine(ChangeArucoMarker());
    }

    IEnumerator ChangeArucoMarker()
    {
        while (true)
        {
            if (arucoMarkerTextures.Count == 0)
            {
                Debug.LogError("No ARUCO marker textures found!");
                yield break;
            }

            // Change the texture of the material
            arucoMaterial.mainTexture = arucoMarkerTextures[Random.Range(0, arucoMarkerTextures.Count)];

            // Wait for 30 seconds before changing the texture again
            yield return new WaitForSeconds(30);
        }
    }

    // Method to change the texture manually
    public void ChangeTexture(int index)
    {
        if (index >= 0 && index < arucoMarkerTextures.Count)
        {
            arucoMaterial.mainTexture = arucoMarkerTextures[index];
        }
        else
        {
            Debug.LogError("Invalid texture index!");
        }
    }
}