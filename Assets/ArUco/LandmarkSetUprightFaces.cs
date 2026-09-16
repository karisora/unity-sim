using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

[ExecuteAlways]
[DisallowMultipleComponent]
public sealed class LandmarkSetUprightFaces : MonoBehaviour
{
    [SerializeField] private Material[] landmarkMaterials;

    private static Mesh uprightCubeMesh;
#if UNITY_EDITOR
    private bool applyQueued;
#endif

    private void OnEnable()
    {
        if (Application.isPlaying)
        {
            ApplyToLandmarks();
        }
#if UNITY_EDITOR
        else
        {
            QueueApply();
        }
#endif
    }

    private void OnValidate()
    {
#if UNITY_EDITOR
        // MeshFilter.sharedMesh cannot be changed while Unity is running its
        // validation pass. Defer it until the next safe editor update.
        QueueApply();
#endif
    }

#if UNITY_EDITOR
    private void QueueApply()
    {
        if (applyQueued)
        {
            return;
        }

        applyQueued = true;
        EditorApplication.delayCall += ApplyDeferred;
    }

    private void ApplyDeferred()
    {
        applyQueued = false;
        if (this == null || !isActiveAndEnabled || EditorApplication.isPlayingOrWillChangePlaymode)
        {
            return;
        }

        ApplyToLandmarks();
    }

    private void OnDisable()
    {
        if (!applyQueued)
        {
            return;
        }

        EditorApplication.delayCall -= ApplyDeferred;
        applyQueued = false;
    }
#endif

    [ContextMenu("Apply Upright Landmark Faces")]
    public void ApplyToLandmarks()
    {
        if (uprightCubeMesh == null)
        {
            uprightCubeMesh = CreateUprightCubeMesh();
        }

        foreach (Transform landmark in transform)
        {
            if (!TryGetLandmarkNumber(landmark.name, out int number))
            {
                continue;
            }

            MeshFilter target = FindCubeMeshFilter(landmark);
            if (target == null)
            {
                continue;
            }

            if (target.sharedMesh != uprightCubeMesh)
            {
                target.sharedMesh = uprightCubeMesh;
            }

            int materialIndex = number - 1;
            MeshRenderer renderer = target.GetComponent<MeshRenderer>();
            if (renderer != null &&
                landmarkMaterials != null &&
                materialIndex >= 0 &&
                materialIndex < landmarkMaterials.Length &&
                landmarkMaterials[materialIndex] != null)
            {
                Material material = landmarkMaterials[materialIndex];
                if (renderer.sharedMaterial != material)
                {
                    renderer.sharedMaterial = material;
                }
            }
        }
    }

    private static bool TryGetLandmarkNumber(string objectName, out int number)
    {
        number = 0;
        return objectName.Length > 1 &&
               objectName[0] == 'L' &&
               int.TryParse(objectName.Substring(1), out number) &&
               number >= 1 &&
               number <= 15;
    }

    private static MeshFilter FindCubeMeshFilter(Transform landmark)
    {
        MeshFilter[] candidates = landmark.GetComponentsInChildren<MeshFilter>(true);
        foreach (MeshFilter candidate in candidates)
        {
            if (candidate.gameObject.name == "Cube")
            {
                return candidate;
            }
        }

        return null;
    }

    private static Mesh CreateUprightCubeMesh()
    {
        var mesh = new Mesh
        {
            name = "LandmarkBox_Upright_Runtime",
            hideFlags = HideFlags.HideAndDontSave
        };

        Vector3[] vertices =
        {
            // Front (+Z)
            new Vector3(-0.5f, -0.5f,  0.5f), new Vector3( 0.5f, -0.5f,  0.5f),
            new Vector3( 0.5f,  0.5f,  0.5f), new Vector3(-0.5f,  0.5f,  0.5f),
            // Right (+X)
            new Vector3( 0.5f, -0.5f,  0.5f), new Vector3( 0.5f, -0.5f, -0.5f),
            new Vector3( 0.5f,  0.5f, -0.5f), new Vector3( 0.5f,  0.5f,  0.5f),
            // Back (-Z)
            new Vector3( 0.5f, -0.5f, -0.5f), new Vector3(-0.5f, -0.5f, -0.5f),
            new Vector3(-0.5f,  0.5f, -0.5f), new Vector3( 0.5f,  0.5f, -0.5f),
            // Left (-X)
            new Vector3(-0.5f, -0.5f, -0.5f), new Vector3(-0.5f, -0.5f,  0.5f),
            new Vector3(-0.5f,  0.5f,  0.5f), new Vector3(-0.5f,  0.5f, -0.5f),
            // Top (+Y)
            new Vector3(-0.5f,  0.5f,  0.5f), new Vector3( 0.5f,  0.5f,  0.5f),
            new Vector3( 0.5f,  0.5f, -0.5f), new Vector3(-0.5f,  0.5f, -0.5f),
            // Bottom (-Y)
            new Vector3(-0.5f, -0.5f, -0.5f), new Vector3( 0.5f, -0.5f, -0.5f),
            new Vector3( 0.5f, -0.5f,  0.5f), new Vector3(-0.5f, -0.5f,  0.5f)
        };

        // Unity's cube-face viewing direction requires U to run from 1 to 0
        // across these outward-facing vertices. This keeps text readable instead
        // of horizontally mirrored on every vertical face.
        Vector2 bottomLeft = new Vector2(1f, 0f);
        Vector2 bottomRight = new Vector2(0f, 0f);
        Vector2 topRight = new Vector2(0f, 1f);
        Vector2 topLeft = new Vector2(1f, 1f);
        Vector2 blank = new Vector2(0.02f, 0.98f);
        Vector2[] uv = new Vector2[24];
        for (int face = 0; face < 4; face++)
        {
            int offset = face * 4;
            uv[offset] = bottomLeft;
            uv[offset + 1] = bottomRight;
            uv[offset + 2] = topRight;
            uv[offset + 3] = topLeft;
        }
        for (int index = 16; index < uv.Length; index++)
        {
            uv[index] = blank;
        }

        Vector3[] normals = new Vector3[24];
        SetFaceNormal(normals, 0, Vector3.forward);
        SetFaceNormal(normals, 4, Vector3.right);
        SetFaceNormal(normals, 8, Vector3.back);
        SetFaceNormal(normals, 12, Vector3.left);
        SetFaceNormal(normals, 16, Vector3.up);
        SetFaceNormal(normals, 20, Vector3.down);

        int[] triangles = new int[36];
        for (int face = 0; face < 6; face++)
        {
            int vertex = face * 4;
            int triangle = face * 6;
            triangles[triangle] = vertex;
            triangles[triangle + 1] = vertex + 1;
            triangles[triangle + 2] = vertex + 2;
            triangles[triangle + 3] = vertex;
            triangles[triangle + 4] = vertex + 2;
            triangles[triangle + 5] = vertex + 3;
        }

        mesh.vertices = vertices;
        mesh.uv = uv;
        mesh.normals = normals;
        mesh.triangles = triangles;
        mesh.RecalculateBounds();
        return mesh;
    }

    private static void SetFaceNormal(Vector3[] normals, int start, Vector3 normal)
    {
        for (int index = start; index < start + 4; index++)
        {
            normals[index] = normal;
        }
    }
}
