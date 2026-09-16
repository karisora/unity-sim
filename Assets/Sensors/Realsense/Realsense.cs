using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using ROS2;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

[RequireComponent(typeof(Camera))]
[RequireComponent(typeof(ROS2UnityComponent))]
public class RealSense : MonoBehaviour
{
    [Header("Capture")]
    [SerializeField, Min(1)] private int width = 640;
    [SerializeField, Min(1)] private int height = 360;
    [SerializeField, Range(1, 120)] private int fps = 15;
    [SerializeField, Range(1.0f, 179.0f)] private float vFov = 60.0f;
    [SerializeField, Min(0.001f)] private float nearClipPlane = 0.01f;
    [SerializeField, Min(0.01f)] private float farClipPlane = 3000.0f;

    [Header("Outputs")]
    [Tooltip("Publish an RGB image and camera_info.")]
    [SerializeField] private bool publishColor = true;
    [Tooltip("Publish a GPU-rendered 32FC1 depth image in metres.")]
    [SerializeField] private bool publishDepth = false;
    [Tooltip("Publish a point cloud generated from the GPU depth image. No Physics.Raycast calls are used.")]
    [SerializeField] private bool publishPointCloud = true;

    [Header("ROS 2")]
    [Tooltip("Optional ROS node name. When empty, a valid unique name is generated from this GameObject name.")]
    [SerializeField] private string nodeName = "";
    [Tooltip("Optional per-camera namespace. It replaces the default /realsense prefix, so two cameras can use /camera/front and /camera/rear.")]
    [SerializeField] private string topicNamespace = "/realsense";
    [Tooltip("Automatically add _2, _3, ... when another RealSense camera already uses the same namespace.")]
    [SerializeField] private bool ensureUniqueTopicNamespace = true;
    [Tooltip("Use a different frame ID for each camera, for example camera_front_optical_frame and camera_rear_optical_frame.")]
    [SerializeField] private string frameID = "realsense_link";
    [Tooltip("Use a different topic for each camera, for example /camera/front/color/image_raw.")]
    [SerializeField] private string imageTopic = "/realsense/image_raw";
    [SerializeField] private string infoTopic = "/realsense/camera_info";
    [Tooltip("Depth image topic. Encoding is 32FC1 and values are metres.")]
    [SerializeField] private string depthTopic = "/realsense/depth/image_raw";
    [SerializeField] private string pointCloudTopic = "/realsense/cloud";

    [Header("Point cloud")]
    [Tooltip("Pixel sampling interval. Larger values publish fewer points and reduce CPU/network load.")]
    [SerializeField, Range(1, 32)] private int pointCloudStride = 13;
    [SerializeField, Min(1)] private int maxPoints = 20000;

    [Header("GPU")]
    [Tooltip("Optional. Keep this disabled when the shader only copies the image.")]
    [SerializeField] private bool useImageProcessingShader = false;
    [SerializeField] private ComputeShader imageProcessingShader;
    [Tooltip("Linear-depth replacement shader. Hidden/Realsense/DepthOnly is found automatically when this is empty.")]
    [SerializeField] private Shader depthShader;

    private static readonly HashSet<string> ActiveNodeNames = new HashSet<string>();
    private static readonly HashSet<string> ActiveTopicNamespaces = new HashSet<string>();

    private Camera cam;
    private RenderTexture colorTexture;
    private RenderTexture processedTexture;
    private RenderTexture depthTexture;
    private RenderTexture originalTargetTexture;
    private bool originalCameraEnabled;
    private bool originalAllowHdr;
    private bool originalAllowMsaa;
    private float originalFieldOfView;
    private float originalNearClipPlane;
    private float originalFarClipPlane;
    private int imageKernel = -1;
    private float nextCaptureTime;
    private bool colorReadbackPending;
    private bool depthReadbackPending;
    private bool shuttingDown;
    private AsyncGPUReadbackRequest colorRequest;
    private AsyncGPUReadbackRequest depthRequest;
    private byte[] latestRgb;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private string activeNodeName;
    private string activeTopicNamespace;
    private IPublisher<sensor_msgs.msg.Image> imagePub;
    private IPublisher<sensor_msgs.msg.Image> depthPub;
    private IPublisher<sensor_msgs.msg.CameraInfo> infoPub;
    private IPublisher<sensor_msgs.msg.PointCloud2> pc2Pub;

    [StructLayout(LayoutKind.Explicit)]
    private struct UIntFloat
    {
        [FieldOffset(0)] public uint UIntValue;
        [FieldOffset(0)] public float FloatValue;
    }

    private void Awake()
    {
        cam = GetComponent<Camera>();
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    private void Start()
    {
        width = Mathf.Max(1, width);
        height = Mathf.Max(1, height);
        fps = Mathf.Max(1, fps);

        originalTargetTexture = cam.targetTexture;
        originalCameraEnabled = cam.enabled;
        originalAllowHdr = cam.allowHDR;
        originalAllowMsaa = cam.allowMSAA;
        originalFieldOfView = cam.fieldOfView;
        originalNearClipPlane = cam.nearClipPlane;
        originalFarClipPlane = cam.farClipPlane;

        // Render only at the sensor rate. Leaving Camera.enabled on would
        // render every display frame as well and duplicate the GPU work.
        cam.enabled = false;
        cam.allowHDR = false;
        cam.allowMSAA = false;
        cam.depthTextureMode = DepthTextureMode.None;
        cam.fieldOfView = vFov;
        cam.nearClipPlane = nearClipPlane;
        cam.farClipPlane = Mathf.Max(nearClipPlane + 0.01f, farClipPlane);

        if (!SystemInfo.supportsAsyncGPUReadback)
        {
            Debug.LogError($"[{nameof(RealSense)}] Async GPU readback is not supported on this graphics device.", this);
            enabled = false;
            return;
        }

        colorTexture = CreateRenderTexture("Color", RenderTextureFormat.ARGB32, 24, false);
        cam.targetTexture = colorTexture;

        if (useImageProcessingShader && imageProcessingShader != null)
        {
            try
            {
                imageKernel = imageProcessingShader.FindKernel("CSMain");
                processedTexture = CreateRenderTexture("Processed", RenderTextureFormat.ARGB32, 0, true);
            }
            catch (Exception exception)
            {
                imageKernel = -1;
                Debug.LogWarning($"[{nameof(RealSense)}] Image compute shader is unavailable; using the camera texture directly. {exception.Message}", this);
            }
        }

        if (publishDepth || publishPointCloud)
        {
            InitializeDepthRendering();
        }
    }

    private void Update()
    {
        if (shuttingDown || ros2Unity == null || !ros2Unity.Ok())
        {
            return;
        }

        EnsureRosPublishers();

        if (!publishColor && !publishDepth && !publishPointCloud)
        {
            return;
        }

        if (Time.unscaledTime < nextCaptureTime)
        {
            return;
        }

        // Bound in-flight work so two cameras cannot build an ever-growing
        // readback queue if the GPU or ROS subscriber is slow.
        bool needColor = publishColor || publishPointCloud;
        bool needDepth = publishDepth || publishPointCloud;
        if ((needColor && colorReadbackPending) || (needDepth && depthReadbackPending))
        {
            return;
        }

        nextCaptureTime = Time.unscaledTime + 1.0f / fps;
        CaptureFrame(needColor, needDepth);
    }

    private RenderTexture CreateRenderTexture(string suffix, RenderTextureFormat format, int depthBits, bool randomWrite)
    {
        var texture = new RenderTexture(width, height, depthBits, format, RenderTextureReadWrite.Linear)
        {
            name = $"{gameObject.name}_{suffix}",
            antiAliasing = 1,
            filterMode = FilterMode.Point,
            wrapMode = TextureWrapMode.Clamp,
            enableRandomWrite = randomWrite,
            useMipMap = false,
            autoGenerateMips = false
        };
        texture.Create();
        return texture;
    }

    private void InitializeDepthRendering()
    {
        if (!SystemInfo.SupportsRenderTextureFormat(RenderTextureFormat.RFloat))
        {
            Debug.LogError($"[{nameof(RealSense)}] RFloat render textures are not supported; depth output is disabled.", this);
            publishDepth = false;
            publishPointCloud = false;
            return;
        }

        if (depthShader == null)
        {
            depthShader = Shader.Find("Hidden/Realsense/DepthOnly");
        }

        if (depthShader == null)
        {
            Debug.LogError($"[{nameof(RealSense)}] Assign the Hidden/Realsense/DepthOnly shader to enable depth output.", this);
            publishDepth = false;
            publishPointCloud = false;
            return;
        }

        depthTexture = CreateRenderTexture("Depth", RenderTextureFormat.RFloat, 24, false);
    }

    private void EnsureRosPublishers()
    {
        if (ros2Node == null)
        {
            activeNodeName = ClaimUniqueNodeName(nodeName, gameObject.name);
            activeTopicNamespace = ClaimTopicNamespace(topicNamespace, ensureUniqueTopicNamespace);
            if (!string.IsNullOrWhiteSpace(topicNamespace) && activeTopicNamespace != NormalizeNamespace(topicNamespace))
            {
                Debug.Log($"[{nameof(RealSense)}] Topic namespace {topicNamespace} is already in use; {gameObject.name} will use {activeTopicNamespace}.", this);
            }
            ros2Node = ros2Unity.CreateNode(activeNodeName);
        }

        if (infoPub == null)
        {
            infoPub = ros2Node.CreatePublisher<sensor_msgs.msg.CameraInfo>(ResolveTopic(infoTopic));
        }
        if (publishColor && imagePub == null)
        {
            imagePub = ros2Node.CreatePublisher<sensor_msgs.msg.Image>(ResolveTopic(imageTopic));
        }
        if (publishDepth && depthPub == null)
        {
            depthPub = ros2Node.CreatePublisher<sensor_msgs.msg.Image>(ResolveTopic(depthTopic));
        }
        if (publishPointCloud && pc2Pub == null)
        {
            pc2Pub = ros2Node.CreatePublisher<sensor_msgs.msg.PointCloud2>(ResolveTopic(pointCloudTopic));
        }

        // Allow depth/point-cloud output to be enabled in the Inspector during Play mode.
        if ((publishDepth || publishPointCloud) && depthTexture == null)
        {
            InitializeDepthRendering();
        }
    }

    private void CaptureFrame(bool needColor, bool needDepth)
    {
        if (needColor)
        {
            cam.targetTexture = colorTexture;
            cam.Render();

            RenderTexture readbackTexture = colorTexture;
            if (processedTexture != null && imageKernel >= 0)
            {
                imageProcessingShader.SetTexture(imageKernel, "Source", colorTexture);
                imageProcessingShader.SetTexture(imageKernel, "Result", processedTexture);
                imageProcessingShader.SetInt("Width", width);
                imageProcessingShader.SetInt("Height", height);
                imageProcessingShader.Dispatch(imageKernel, Mathf.CeilToInt(width / 8.0f), Mathf.CeilToInt(height / 8.0f), 1);
                readbackTexture = processedTexture;
            }

            colorReadbackPending = true;
            colorRequest = AsyncGPUReadback.Request(readbackTexture, 0, TextureFormat.RGBA32, OnColorReadback);
        }

        if (needDepth && depthTexture != null && depthShader != null)
        {
            CameraClearFlags savedClearFlags = cam.clearFlags;
            Color savedBackground = cam.backgroundColor;

            cam.targetTexture = depthTexture;
            cam.clearFlags = CameraClearFlags.SolidColor;
            // RealSense-compatible convention: pixels with no return remain 0 m.
            cam.backgroundColor = Color.clear;
            cam.RenderWithShader(depthShader, "");

            cam.clearFlags = savedClearFlags;
            cam.backgroundColor = savedBackground;
            cam.targetTexture = colorTexture;

            depthReadbackPending = true;
            depthRequest = AsyncGPUReadback.Request(depthTexture, 0, TextureFormat.RFloat, OnDepthReadback);
        }
    }

    private void OnColorReadback(AsyncGPUReadbackRequest request)
    {
        colorReadbackPending = false;
        if (shuttingDown || request.hasError)
        {
            if (request.hasError && !shuttingDown)
            {
                Debug.LogWarning($"[{nameof(RealSense)}] Color GPU readback failed for {gameObject.name}.", this);
            }
            return;
        }

        NativeArray<byte> rgba = request.GetData<byte>();
        int pixelCount = width * height;
        if (latestRgb == null || latestRgb.Length != pixelCount * 3)
        {
            latestRgb = new byte[pixelCount * 3];
        }

        // Unity readback is bottom-left first; ROS image rows are top-left first.
        for (int y = 0; y < height; ++y)
        {
            int sourceRow = (height - 1 - y) * width * 4;
            int destinationRow = y * width * 3;
            for (int x = 0; x < width; ++x)
            {
                int source = sourceRow + x * 4;
                int destination = destinationRow + x * 3;
                latestRgb[destination] = rgba[source];
                latestRgb[destination + 1] = rgba[source + 1];
                latestRgb[destination + 2] = rgba[source + 2];
            }
        }

        if (!publishColor || imagePub == null || ros2Node == null)
        {
            return;
        }

        var imageMessage = new sensor_msgs.msg.Image
        {
            Header = CreateHeader(),
            Height = (uint)height,
            Width = (uint)width,
            Encoding = "rgb8",
            Is_bigendian = 0,
            Step = (uint)(width * 3),
            Data = (byte[])latestRgb.Clone()
        };
        imagePub.Publish(imageMessage);
        PublishCameraInfo(imageMessage.Header);
    }

    private void OnDepthReadback(AsyncGPUReadbackRequest request)
    {
        depthReadbackPending = false;
        if (shuttingDown || request.hasError)
        {
            if (request.hasError && !shuttingDown)
            {
                Debug.LogWarning($"[{nameof(RealSense)}] Depth GPU readback failed for {gameObject.name}.", this);
            }
            return;
        }

        NativeArray<float> gpuDepth = request.GetData<float>();
        float[] rosDepth = new float[width * height];
        for (int y = 0; y < height; ++y)
        {
            int sourceRow = (height - 1 - y) * width;
            int destinationRow = y * width;
            for (int x = 0; x < width; ++x)
            {
                rosDepth[destinationRow + x] = gpuDepth[sourceRow + x];
            }
        }

        std_msgs.msg.Header depthHeader = null;
        if (publishDepth && depthPub != null && ros2Node != null)
        {
            byte[] depthBytes = new byte[rosDepth.Length * sizeof(float)];
            Buffer.BlockCopy(rosDepth, 0, depthBytes, 0, depthBytes.Length);
            depthHeader = CreateHeader();
            var depthMessage = new sensor_msgs.msg.Image
            {
                Header = depthHeader,
                Height = (uint)height,
                Width = (uint)width,
                Encoding = "32FC1",
                Is_bigendian = (byte)(BitConverter.IsLittleEndian ? 0 : 1),
                Step = (uint)(width * sizeof(float)),
                Data = depthBytes
            };
            depthPub.Publish(depthMessage);
        }

        if (!publishColor && (publishDepth || publishPointCloud))
        {
            PublishCameraInfo(depthHeader ?? CreateHeader());
        }

        if (publishPointCloud && pc2Pub != null && ros2Node != null)
        {
            PublishPointCloud(rosDepth);
        }
    }

    private std_msgs.msg.Header CreateHeader()
    {
        var header = new std_msgs.msg.Header { Frame_id = frameID };
        ros2Node.clock.UpdateROSClockTime(header.Stamp);
        return header;
    }

    private void PublishCameraInfo(std_msgs.msg.Header sourceHeader)
    {
        if (infoPub == null)
        {
            return;
        }

        GetIntrinsics(out double fx, out double fy, out double cx, out double cy);
        var message = new sensor_msgs.msg.CameraInfo
        {
            Header = sourceHeader,
            Height = (uint)height,
            Width = (uint)width,
            Distortion_model = "plumb_bob",
            D = new double[5]
        };

        message.K[0] = fx;
        message.K[2] = cx;
        message.K[4] = fy;
        message.K[5] = cy;
        message.K[8] = 1.0;

        message.R[0] = 1.0;
        message.R[4] = 1.0;
        message.R[8] = 1.0;

        message.P[0] = fx;
        message.P[2] = cx;
        message.P[5] = fy;
        message.P[6] = cy;
        message.P[10] = 1.0;
        infoPub.Publish(message);
    }

    private void GetIntrinsics(out double fx, out double fy, out double cx, out double cy)
    {
        fy = height * 0.5 / Math.Tan(vFov * Mathf.Deg2Rad * 0.5);
        fx = fy;
        cx = (width - 1) * 0.5;
        cy = (height - 1) * 0.5;
    }

    private void PublishPointCloud(float[] depthValues)
    {
        GetIntrinsics(out double fx, out double fy, out double cx, out double cy);
        int stride = Mathf.Max(1, pointCloudStride);
        int capacity = Mathf.Min(maxPoints, ((width + stride - 1) / stride) * ((height + stride - 1) / stride));
        float[] packedPoints = new float[capacity * 4];
        int pointCount = 0;

        for (int y = 0; y < height && pointCount < capacity; y += stride)
        {
            for (int x = 0; x < width && pointCount < capacity; x += stride)
            {
                int pixelIndex = y * width + x;
                float z = depthValues[pixelIndex];
                if (float.IsNaN(z) || float.IsInfinity(z) || z <= cam.nearClipPlane || z >= cam.farClipPlane - 0.001f)
                {
                    continue;
                }

                int output = pointCount * 4;
                packedPoints[output] = (float)((x - cx) * z / fx);
                packedPoints[output + 1] = (float)((y - cy) * z / fy);
                packedPoints[output + 2] = z;

                uint rgb = 0;
                if (latestRgb != null && latestRgb.Length == width * height * 3)
                {
                    int color = pixelIndex * 3;
                    rgb = ((uint)latestRgb[color] << 16) | ((uint)latestRgb[color + 1] << 8) | latestRgb[color + 2];
                }
                packedPoints[output + 3] = new UIntFloat { UIntValue = rgb }.FloatValue;
                ++pointCount;
            }
        }

        const int pointStep = 16;
        byte[] data = new byte[pointCount * pointStep];
        Buffer.BlockCopy(packedPoints, 0, data, 0, data.Length);

        var message = new sensor_msgs.msg.PointCloud2
        {
            Header = CreateHeader(),
            Height = 1,
            Width = (uint)pointCount,
            Is_bigendian = !BitConverter.IsLittleEndian,
            Is_dense = true,
            Point_step = pointStep,
            Row_step = (uint)(pointCount * pointStep),
            Fields = new[]
            {
                new sensor_msgs.msg.PointField { Name = "x", Offset = 0, Datatype = 7, Count = 1 },
                new sensor_msgs.msg.PointField { Name = "y", Offset = 4, Datatype = 7, Count = 1 },
                new sensor_msgs.msg.PointField { Name = "z", Offset = 8, Datatype = 7, Count = 1 },
                new sensor_msgs.msg.PointField { Name = "rgb", Offset = 12, Datatype = 7, Count = 1 }
            },
            Data = data
        };
        pc2Pub.Publish(message);
    }

    private static string ClaimUniqueNodeName(string configuredName, string objectName)
    {
        string source = string.IsNullOrWhiteSpace(configuredName) ? $"realsense_{objectName}" : configuredName;
        var builder = new StringBuilder(source.Length + 16);
        foreach (char character in source)
        {
            bool asciiLetter = (character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z');
            bool asciiDigit = character >= '0' && character <= '9';
            builder.Append(asciiLetter || asciiDigit || character == '_' ? character : '_');
        }

        string candidate = builder.Length == 0 ? "realsense_camera" : builder.ToString();
        if (!char.IsLetter(candidate[0]) && candidate[0] != '_')
        {
            candidate = "realsense_" + candidate;
        }

        string unique = candidate;
        int suffix = 2;
        while (!ActiveNodeNames.Add(unique))
        {
            unique = $"{candidate}_{suffix++}";
        }
        return unique;
    }

    private string ResolveTopic(string configuredTopic)
    {
        string selectedNamespace = activeTopicNamespace ?? NormalizeNamespace(topicNamespace);
        if (string.IsNullOrWhiteSpace(selectedNamespace))
        {
            return configuredTopic;
        }

        string suffix = configuredTopic.Trim();
        const string defaultPrefix = "/realsense";
        if (suffix == defaultPrefix)
        {
            suffix = "";
        }
        else if (suffix.StartsWith(defaultPrefix + "/", StringComparison.Ordinal))
        {
            suffix = suffix.Substring(defaultPrefix.Length);
        }
        else
        {
            suffix = "/" + suffix.TrimStart('/');
        }

        return selectedNamespace + suffix;
    }

    private static string ClaimTopicNamespace(string configuredNamespace, bool ensureUnique)
    {
        string candidate = NormalizeNamespace(configuredNamespace);
        if (string.IsNullOrEmpty(candidate) || !ensureUnique)
        {
            return candidate;
        }

        string unique = candidate;
        int suffix = 2;
        while (!ActiveTopicNamespaces.Add(unique))
        {
            unique = $"{candidate}_{suffix++}";
        }
        return unique;
    }

    private static string NormalizeNamespace(string configuredNamespace)
    {
        if (string.IsNullOrWhiteSpace(configuredNamespace))
        {
            return "";
        }

        string namespaceBody = configuredNamespace.Trim().Trim('/');
        return namespaceBody.Length == 0 ? "" : "/" + namespaceBody;
    }

    private void OnDestroy()
    {
        shuttingDown = true;

        if (colorReadbackPending)
        {
            colorRequest.WaitForCompletion();
        }
        if (depthReadbackPending)
        {
            depthRequest.WaitForCompletion();
        }

        if (cam != null)
        {
            cam.targetTexture = originalTargetTexture;
            cam.enabled = originalCameraEnabled;
            cam.allowHDR = originalAllowHdr;
            cam.allowMSAA = originalAllowMsaa;
            cam.fieldOfView = originalFieldOfView;
            cam.nearClipPlane = originalNearClipPlane;
            cam.farClipPlane = originalFarClipPlane;
        }

        ReleaseRenderTexture(ref colorTexture);
        ReleaseRenderTexture(ref processedTexture);
        ReleaseRenderTexture(ref depthTexture);

        if (ros2Unity != null && ros2Node != null)
        {
            ros2Unity.RemoveNode(ros2Node);
            ros2Node = null;
        }
        if (!string.IsNullOrEmpty(activeNodeName))
        {
            ActiveNodeNames.Remove(activeNodeName);
        }
        if (ensureUniqueTopicNamespace && !string.IsNullOrEmpty(activeTopicNamespace))
        {
            ActiveTopicNamespaces.Remove(activeTopicNamespace);
        }
    }

    private static void ReleaseRenderTexture(ref RenderTexture texture)
    {
        if (texture == null)
        {
            return;
        }
        texture.Release();
        Destroy(texture);
        texture = null;
    }

    private void OnValidate()
    {
        width = Mathf.Max(1, width);
        height = Mathf.Max(1, height);
        fps = Mathf.Max(1, fps);
        nearClipPlane = Mathf.Max(0.001f, nearClipPlane);
        farClipPlane = Mathf.Max(nearClipPlane + 0.01f, farClipPlane);
        pointCloudStride = Mathf.Max(1, pointCloudStride);
        maxPoints = Mathf.Max(1, maxPoints);
    }
}
