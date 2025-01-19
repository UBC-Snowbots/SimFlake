using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TripleAruco : MonoBehaviour
{
    public List<Texture2D> arucoMarkerTextures; // List of ARUCO marker textures
    public GameObject marker1; // Reference to the existing cube
    public GameObject marker2; // Reference to the existing cube
    public GameObject marker3; // Reference to the existing cube

    private Material arucoMaterial;
    
    public int id = 0;
    void Start()
    {
        if (marker1 == null || marker2 == null || marker3 == null)
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
        arucoMaterial = new Material(Shader.Find("Unlit/Texture"));

        // Assign the material to the existing cube
        marker1.GetComponent<Renderer>().material = arucoMaterial;
        marker2.GetComponent<Renderer>().material = arucoMaterial;
        marker3.GetComponent<Renderer>().material = arucoMaterial;
        setTexture(id);
    }


    // Method to change the texture manually
    private void setTexture(int index)
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