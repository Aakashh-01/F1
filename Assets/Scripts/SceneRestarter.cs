using UnityEngine;
using UnityEngine.SceneManagement; 

public class SceneRestarter : MonoBehaviour
{
    void Update()
    {
        // Check if the 'R' key was pressed this frame
        if (Input.GetKeyDown(KeyCode.R))
        {
            RestartCurrentScene();
        }
    }

    void RestartCurrentScene()
    {
        // Get the name of the currently active scene and reload it
        string currentSceneName = SceneManager.GetActiveScene().name;
        SceneManager.LoadScene(currentSceneName);
    }
}