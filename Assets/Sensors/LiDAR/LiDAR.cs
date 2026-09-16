using System;
using System.Text;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using ROS2;

namespace ROS2
{
    [RequireComponent(typeof(ROS2UnityComponent))]
    public class LiDAR : MonoBehaviour
    {
        private const byte PointFieldUint8 = 2;
        private const byte PointFieldFloat32 = 7;
        private const byte PointFieldFloat64 = 8;
        private const int FloatFieldsPerPoint = 4; // x, y, z, intensity
        private const int LivoxPointStepBytes = 26; // x, y, z, intensity, tag, line, timestamp
        private const int LivoxTagOffset = 16;
        private const int LivoxLineOffset = 17;
        private const int LivoxTimestampOffset = 18;
        private const int LivoxMid360LineCount = 4;
        private const double NanosecondsPerSecond = 1000000000.0;

        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<sensor_msgs.msg.PointCloud2> pointCloudPub;
        private IPublisher<sensor_msgs.msg.Imu> imuPub;

        public string frameID = "livox_frame";
        public string topicName = "/livox/lidar";

        [Header("LiDAR Publish Settings")]
        public float TopicPublishHz = 10f;

        [Header("IMU Publish Settings")]
        public bool publishImu = true;
        public string imuFrameID = "livox_frame";
        public string imuTopicName = "/livox/imu";
        public float ImuPublishHz = 100f;

        [Header("IMU Unity Physics Source")]
        [Tooltip("Rigidbody used as the ideal Unity motion source. When empty, the closest parent Rigidbody is used.")]
        public Rigidbody imuRigidbody;
        [Tooltip("Use Rigidbody point velocity and angular velocity instead of differentiating the rendered Transform.")]
        public bool useRigidbodyKinematics = true;
        [Min(0f)]
        [Tooltip("Low-pass time constant for acceleration in seconds. Set to zero to disable filtering.")]
        public float accelerationFilterTimeConstant = 0.02f;
        [Min(0f)]
        [Tooltip("Low-pass time constant for angular velocity in seconds. Rigidbody angular velocity is already clean, so zero is recommended.")]
        public float angularVelocityFilterTimeConstant = 0f;

        [Header("Livox MID-360 Scan Settings")]
        public bool useLivoxMid360Preset = true;
        [Min(1)]
        public int pointRate = 200000;
        [Range(1f, 360f)]
        public float horizontalFov = 360f;
        [Range(-89f, 89f)]
        public float horizontalCenter = 0f;
        [Range(-89f, 89f)]
        public float verticalMinAngle = -7f;
        [Range(-89f, 89f)]
        public float verticalMaxAngle = 40f;
        [Min(1)]
        public int verticalScanLines = 32;
        [Min(0.01f)]
        public float minDistance = 0.1f;
        [Min(0.1f)]
        public float maxDistance = 10f;
        [Min(0f)]
        public float lowReflectivityRange = 40f;
        [Min(0f)]
        public float rangePrecisionStdDev = 0.02f;
        [Range(0f, 1f)]
        public float minIntensity = 0.05f;
        [Min(0f)]
        public float rayStartOffset = 0.02f;
        public LayerMask scanLayerMask = Physics.DefaultRaycastLayers;
        public QueryTriggerInteraction queryTriggerInteraction = QueryTriggerInteraction.Ignore;

        [Header("LiDAR Performance")]
        public bool useBatchedRaycasts = true;
        [Min(1)]
        public int raycastCommandsPerJob = 64;
        public bool estimateReflectivityFromMaterial = false;

        [Header("Current LiDAR State")]
        public int currentPointCount;
        public int currentRayCount;
        public float lastPublishTime;
        public Vector3 acceleration; // IMU local frame, including gravity (m/s^2, ROS coordinates)
        public Vector3 angularVelocity;
        public Quaternion orientation;

        private float publishTimer = 0f;
        private float imuPublishTimer = 0f;
        private Vector3 lastImuPosition;
        private Quaternion lastImuRotation;
        private Vector3 lastImuWorldVelocity;
        private bool imuSampleInitialized;
        private Vector3 lastScanPosition;
        private Quaternion lastScanRotation;
        private Vector3[] localRayDirections;
        private float[] pointBuffer;
        private byte[] pointTagBuffer;
        private byte[] pointLineBuffer;
        private double[] pointTimestampBuffer;
        private NativeArray<RaycastCommand> raycastCommands;
        private NativeArray<RaycastHit> raycastHits;
        private int scanFrameIndex = 0;

        void OnValidate()
        {
            if (useLivoxMid360Preset)
            {
                TopicPublishHz = 10f;
                pointRate = 200000;
                horizontalFov = 360f;
                horizontalCenter = 0f;
                verticalMinAngle = -7f;
                verticalMaxAngle = 40f;
                verticalScanLines = 32;
                minDistance = 0.1f;
                maxDistance = 10f;
                lowReflectivityRange = 40f;
                rangePrecisionStdDev = 0.02f;
            }

            pointRate = Mathf.Max(1, pointRate);
            TopicPublishHz = Mathf.Max(0.01f, TopicPublishHz);
            ImuPublishHz = Mathf.Max(0.01f, ImuPublishHz);
            accelerationFilterTimeConstant = Mathf.Max(0f, accelerationFilterTimeConstant);
            angularVelocityFilterTimeConstant = Mathf.Max(0f, angularVelocityFilterTimeConstant);
            if (verticalMaxAngle < verticalMinAngle)
            {
                float temp = verticalMinAngle;
                verticalMinAngle = verticalMaxAngle;
                verticalMaxAngle = temp;
            }
            minDistance = Mathf.Max(0.01f, minDistance);
            maxDistance = Mathf.Max(minDistance, maxDistance);
            lowReflectivityRange = Mathf.Clamp(lowReflectivityRange, minDistance, maxDistance);
            rangePrecisionStdDev = Mathf.Max(0f, rangePrecisionStdDev);
            minIntensity = Mathf.Clamp01(minIntensity);
            rayStartOffset = Mathf.Max(0f, rayStartOffset);
            verticalScanLines = Mathf.Max(1, verticalScanLines);
            raycastCommandsPerJob = Mathf.Max(1, raycastCommandsPerJob);

            BuildScanPattern();
        }

        void Start()
        {
            BuildScanPattern();
            InitializeImuState();
            InitializeScanState();

            ros2Unity = GetComponent<ROS2UnityComponent>();
            if (ros2Unity == null)
            {
                Debug.LogError("ROS2UnityComponent not found");
                return;
            }

            try
            {
                ros2Node = ros2Unity.CreateNode(GetNodeName());
                pointCloudPub = ros2Node.CreatePublisher<sensor_msgs.msg.PointCloud2>(topicName);
                if (publishImu)
                {
                    imuPub = ros2Node.CreatePublisher<sensor_msgs.msg.Imu>(imuTopicName);
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"Failed to initialize LiDAR ROS2 publishers: {e.Message}");
                Debug.LogException(e);
                ros2Node = null;
                pointCloudPub = null;
                imuPub = null;
            }
        }

        void OnDestroy()
        {
            DisposeBatchedRaycastBuffers();
        }

        void Update()
        {
            if (pointCloudPub == null)
            {
                return;
            }

            publishTimer += Time.deltaTime;
            if (publishTimer >= 1f / Mathf.Max(0.01f, TopicPublishHz))
            {
                PublishPointCloud(publishTimer);
                publishTimer = 0f;
            }
        }

        void FixedUpdate()
        {
            if (!publishImu)
            {
                return;
            }

            float sampleDt = Time.fixedDeltaTime;
            SampleImuFromUnityPhysics(sampleDt);

            if (imuPub == null)
            {
                return;
            }

            float publishInterval = 1f / Mathf.Max(0.01f, ImuPublishHz);
            imuPublishTimer += sampleDt;
            if (imuPublishTimer + Mathf.Epsilon >= publishInterval)
            {
                // There is only one new physics sample per FixedUpdate. If the
                // requested rate is higher than the physics rate, do not publish
                // duplicate values with misleading acquisition timestamps.
                imuPublishTimer = Mathf.Repeat(imuPublishTimer, publishInterval);
                PublishImu();
            }
        }

        private void InitializeImuState()
        {
            if (useRigidbodyKinematics && imuRigidbody == null)
            {
                imuRigidbody = GetComponentInParent<Rigidbody>();
            }

            lastImuPosition = transform.position;
            lastImuRotation = transform.rotation;
            lastImuWorldVelocity = HasRigidbodyMotionSource()
                ? imuRigidbody.GetPointVelocity(transform.position)
                : Vector3.zero;
            orientation = transform.rotation.Unity2Ros();
            acceleration = transform.InverseTransformDirection(-Physics.gravity).Unity2Ros();
            angularVelocity = HasRigidbodyMotionSource()
                ? UnityAngularVelocityToRos(
                    transform.InverseTransformDirection(imuRigidbody.angularVelocity))
                : Vector3.zero;
            imuSampleInitialized = true;
        }

        private void InitializeScanState()
        {
            lastScanPosition = transform.position;
            lastScanRotation = transform.rotation;
        }

        private void EnsurePointBuffer(int rayCount)
        {
            int requiredLength = rayCount * FloatFieldsPerPoint;
            if (pointBuffer == null || pointBuffer.Length < requiredLength)
            {
                pointBuffer = new float[requiredLength];
            }

            if (pointTagBuffer == null || pointTagBuffer.Length < rayCount)
            {
                pointTagBuffer = new byte[rayCount];
            }

            if (pointLineBuffer == null || pointLineBuffer.Length < rayCount)
            {
                pointLineBuffer = new byte[rayCount];
            }

            if (pointTimestampBuffer == null || pointTimestampBuffer.Length < rayCount)
            {
                pointTimestampBuffer = new double[rayCount];
            }
        }

        private void EnsureBatchedRaycastBuffers(int rayCount)
        {
            if (raycastCommands.IsCreated && raycastCommands.Length == rayCount && raycastHits.IsCreated && raycastHits.Length == rayCount)
            {
                return;
            }

            DisposeBatchedRaycastBuffers();

            raycastCommands = new NativeArray<RaycastCommand>(rayCount, Allocator.Persistent);
            raycastHits = new NativeArray<RaycastHit>(rayCount, Allocator.Persistent);
        }

        private void DisposeBatchedRaycastBuffers()
        {
            if (raycastCommands.IsCreated)
            {
                raycastCommands.Dispose();
            }

            if (raycastHits.IsCreated)
            {
                raycastHits.Dispose();
            }
        }

        private void BuildScanPattern()
        {
            int raysPerFrame = GetRaysPerFrame();
            int verticalCount = Mathf.Clamp(verticalScanLines, 1, raysPerFrame);
            int horizontalCount = Mathf.Max(1, Mathf.CeilToInt((float)raysPerFrame / verticalCount));
            localRayDirections = new Vector3[horizontalCount * verticalCount];

            int index = 0;
            for (int horizontalIndex = 0; horizontalIndex < horizontalCount; horizontalIndex++)
            {
                float horizontalAngle = GetHorizontalAngle(horizontalIndex, horizontalCount);
                for (int verticalIndex = 0; verticalIndex < verticalCount; verticalIndex++)
                {
                    float verticalAngle = GetVerticalAngle(verticalIndex, verticalCount);
                    localRayDirections[index] = GetScanDirection(horizontalAngle, verticalAngle);
                    index++;
                }
            }
        }

        private int GetRaysPerFrame()
        {
            return Mathf.Max(1, Mathf.RoundToInt(pointRate / Mathf.Max(0.01f, TopicPublishHz)));
        }

        private float GetHorizontalAngle(int horizontalIndex, int horizontalCount)
        {
            if (horizontalFov >= 359.9f)
            {
                float fullCircleT = horizontalIndex / (float)horizontalCount;
                return horizontalCenter + fullCircleT * 360f;
            }

            float fovT = horizontalCount <= 1 ? 0.5f : horizontalIndex / (float)(horizontalCount - 1);
            return horizontalCenter + (fovT - 0.5f) * horizontalFov;
        }

        private float GetVerticalAngle(int verticalIndex, int verticalCount)
        {
            float t = verticalCount <= 1 ? 0.5f : verticalIndex / (float)(verticalCount - 1);
            return Mathf.Lerp(verticalMinAngle, verticalMaxAngle, t);
        }

        private Vector3 GetScanDirection(float horizontalAngle, float verticalAngle)
        {
            float horizontalRad = horizontalAngle * Mathf.Deg2Rad;
            float verticalRad = verticalAngle * Mathf.Deg2Rad;
            float horizontalRadius = Mathf.Cos(verticalRad);

            return new Vector3(
                Mathf.Sin(horizontalRad) * horizontalRadius,
                -Mathf.Sin(verticalRad),
                Mathf.Cos(horizontalRad) * horizontalRadius
            ).normalized;
        }

        private float EstimateReflectivity(RaycastHit hit)
        {
            if (!estimateReflectivityFromMaterial)
            {
                return 0.8f;
            }

            Renderer renderer = hit.collider.GetComponentInParent<Renderer>();
            if (renderer == null || renderer.sharedMaterial == null)
            {
                return 0.8f;
            }

            if (!renderer.sharedMaterial.HasProperty("_Color"))
            {
                return 0.8f;
            }

            Color color = renderer.sharedMaterial.color;
            return Mathf.Clamp01(color.grayscale);
        }

        private float GetDetectionLimit(float reflectivity)
        {
            float t = Mathf.InverseLerp(0.1f, 0.8f, reflectivity);
            return Mathf.Lerp(lowReflectivityRange, maxDistance, t);
        }

        private float GetGaussianNoise(int frameIndex, int rayIndex)
        {
            float u1 = Mathf.Max(0.0001f, GetUnitNoise(frameIndex, rayIndex, 83492791));
            float u2 = GetUnitNoise(frameIndex, rayIndex, 297121507);
            float gaussian = Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
            return Mathf.Clamp(gaussian, -3f, 3f);
        }

        private float GetUnitNoise(int frameIndex, int rayIndex, int salt)
        {
            int hash = frameIndex * 73856093 ^ rayIndex * 19349663 ^ salt;
            hash = (hash << 13) ^ hash;
            int value = (hash * (hash * hash * 15731 + 789221) + 1376312589) & 0x7fffffff;
            return value / 2147483647f;
        }

        private void PublishPointCloud(float scanDuration)
        {
            if (localRayDirections == null || localRayDirections.Length == 0)
            {
                BuildScanPattern();
            }

            scanDuration = Mathf.Max(scanDuration, Mathf.Epsilon);
            Vector3 scanStartPosition = lastScanPosition;
            Quaternion scanStartRotation = lastScanRotation;
            Vector3 scanEndPosition = transform.position;
            Quaternion scanEndRotation = transform.rotation;

            // FAST-LIO interprets the header as the beginning of the scan and each
            // point timestamp as a positive offset from it. Capture the end stamp
            // now, then move it back by the actual Unity scan interval.
            var msg = new sensor_msgs.msg.PointCloud2();
            msg.Header = new std_msgs.msg.Header();
            msg.Header.Frame_id = frameID;
            ros2Node.clock.UpdateROSClockTime(msg.Header.Stamp);
            SubtractSecondsFromStamp(msg.Header.Stamp, scanDuration);

            EnsurePointBuffer(localRayDirections.Length);

            int pointCount = useBatchedRaycasts
                ? CollectPointsBatched(scanStartPosition, scanStartRotation, scanEndPosition, scanEndRotation, scanDuration)
                : CollectPointsImmediate(scanStartPosition, scanStartRotation, scanEndPosition, scanEndRotation, scanDuration);

            lastScanPosition = scanEndPosition;
            lastScanRotation = scanEndRotation;

            msg.Height = 1;
            msg.Width = (uint)pointCount;
            msg.Is_bigendian = false;
            msg.Is_dense = true;
            msg.Point_step = LivoxPointStepBytes;
            msg.Row_step = (uint)(pointCount * LivoxPointStepBytes);

            msg.Fields = new sensor_msgs.msg.PointField[7];
            msg.Fields[0] = new sensor_msgs.msg.PointField { Name = "x", Offset = 0, Datatype = PointFieldFloat32, Count = 1 };
            msg.Fields[1] = new sensor_msgs.msg.PointField { Name = "y", Offset = 4, Datatype = PointFieldFloat32, Count = 1 };
            msg.Fields[2] = new sensor_msgs.msg.PointField { Name = "z", Offset = 8, Datatype = PointFieldFloat32, Count = 1 };
            msg.Fields[3] = new sensor_msgs.msg.PointField { Name = "intensity", Offset = 12, Datatype = PointFieldFloat32, Count = 1 };
            msg.Fields[4] = new sensor_msgs.msg.PointField { Name = "tag", Offset = LivoxTagOffset, Datatype = PointFieldUint8, Count = 1 };
            msg.Fields[5] = new sensor_msgs.msg.PointField { Name = "line", Offset = LivoxLineOffset, Datatype = PointFieldUint8, Count = 1 };
            msg.Fields[6] = new sensor_msgs.msg.PointField { Name = "timestamp", Offset = LivoxTimestampOffset, Datatype = PointFieldFloat64, Count = 1 };

            byte[] data = BuildLivoxPointCloudData(pointCount);

            msg.Data = data;

            pointCloudPub.Publish(msg);

            currentPointCount = pointCount;
            currentRayCount = localRayDirections.Length;
            lastPublishTime = Time.time;
            scanFrameIndex++;
        }

        private static void SubtractSecondsFromStamp(builtin_interfaces.msg.Time stamp, double seconds)
        {
            const long nanosecondsPerSecond = 1000000000L;
            long stampNanoseconds = (long)stamp.Sec * nanosecondsPerSecond + stamp.Nanosec;
            long offsetNanoseconds = (long)Math.Round(seconds * nanosecondsPerSecond);
            long scanStartNanoseconds = Math.Max(0L, stampNanoseconds - offsetNanoseconds);

            stamp.Sec = (int)(scanStartNanoseconds / nanosecondsPerSecond);
            stamp.Nanosec = (uint)(scanStartNanoseconds % nanosecondsPerSecond);
        }

        private byte[] BuildLivoxPointCloudData(int pointCount)
        {
            byte[] data = new byte[pointCount * LivoxPointStepBytes];
            for (int i = 0; i < pointCount; i++)
            {
                int dataBaseIndex = i * LivoxPointStepBytes;
                Buffer.BlockCopy(pointBuffer, i * FloatFieldsPerPoint * sizeof(float), data, dataBaseIndex, FloatFieldsPerPoint * sizeof(float));
                data[dataBaseIndex + LivoxTagOffset] = pointTagBuffer[i];
                data[dataBaseIndex + LivoxLineOffset] = pointLineBuffer[i];
                Buffer.BlockCopy(pointTimestampBuffer, i * sizeof(double), data, dataBaseIndex + LivoxTimestampOffset, sizeof(double));
            }

            return data;
        }

        private bool HasRigidbodyMotionSource()
        {
            return useRigidbodyKinematics && imuRigidbody != null;
        }

        private void SampleImuFromUnityPhysics(float dt)
        {
            if (dt <= Mathf.Epsilon)
            {
                return;
            }

            bool hasRigidbody = HasRigidbodyMotionSource();
            Vector3 worldVelocity = hasRigidbody
                ? imuRigidbody.GetPointVelocity(transform.position)
                : (transform.position - lastImuPosition) / dt;

            if (!imuSampleInitialized)
            {
                lastImuWorldVelocity = worldVelocity;
                imuSampleInitialized = true;
            }

            Vector3 worldAcceleration = (worldVelocity - lastImuWorldVelocity) / dt;

            // An accelerometer measures specific force rather than only the
            // kinematic acceleration. Subtracting Unity's gravity makes a
            // stationary, level IMU report approximately +9.81 m/s^2 on its
            // upward axis, as a real Livox IMU does.
            Vector3 specificForceWorld = worldAcceleration - Physics.gravity;
            Vector3 specificForceImu = transform.InverseTransformDirection(specificForceWorld);
            Vector3 rawAcceleration = specificForceImu.Unity2Ros();

            Vector3 worldAngularVelocity;
            if (hasRigidbody)
            {
                // Rigidbody.angularVelocity is expressed in Unity world axes.
                // sensor_msgs/Imu requires the vector in the IMU's local frame.
                worldAngularVelocity = imuRigidbody.angularVelocity;
            }
            else
            {
                Quaternion deltaRotation = transform.rotation * Quaternion.Inverse(lastImuRotation);
                deltaRotation.ToAngleAxis(out float angle, out Vector3 axis);
                if (angle > 180f)
                {
                    angle -= 360f;
                }
                worldAngularVelocity = axis * angle * Mathf.Deg2Rad / dt;
            }

            Vector3 angularVelocityImu = transform.InverseTransformDirection(worldAngularVelocity);
            Vector3 rawAngularVelocity = UnityAngularVelocityToRos(angularVelocityImu);

            acceleration = LowPass(
                acceleration,
                rawAcceleration,
                accelerationFilterTimeConstant,
                dt);
            angularVelocity = LowPass(
                angularVelocity,
                rawAngularVelocity,
                angularVelocityFilterTimeConstant,
                dt);

            orientation = transform.rotation.Unity2Ros();

            lastImuPosition = transform.position;
            lastImuRotation = transform.rotation;
            lastImuWorldVelocity = worldVelocity;
        }

        private static Vector3 UnityAngularVelocityToRos(Vector3 unityAngularVelocity)
        {
            // Unity-to-ROS position conversion changes handedness. Angular
            // velocity is an axial vector, so it needs the additional minus
            // sign to stay consistent with the converted orientation quaternion.
            return -unityAngularVelocity.Unity2Ros();
        }

        private static Vector3 LowPass(Vector3 previous, Vector3 current, float timeConstant, float dt)
        {
            if (timeConstant <= Mathf.Epsilon)
            {
                return current;
            }

            float alpha = 1f - Mathf.Exp(-dt / timeConstant);
            return Vector3.LerpUnclamped(previous, current, alpha);
        }

        private void PublishImu()
        {
            var msg = new sensor_msgs.msg.Imu();
            msg.Header = new std_msgs.msg.Header();
            msg.Header.Frame_id = imuFrameID;
            ros2Node.clock.UpdateROSClockTime(msg.Header.Stamp);

            msg.Linear_acceleration.X = acceleration.x;
            msg.Linear_acceleration.Y = acceleration.y;
            msg.Linear_acceleration.Z = acceleration.z;

            msg.Angular_velocity.X = angularVelocity.x;
            msg.Angular_velocity.Y = angularVelocity.y;
            msg.Angular_velocity.Z = angularVelocity.z;

            msg.Orientation.X = orientation.x;
            msg.Orientation.Y = orientation.y;
            msg.Orientation.Z = orientation.z;
            msg.Orientation.W = orientation.w;

            imuPub.Publish(msg);
        }

        private int CollectPointsBatched(
            Vector3 scanStartPosition,
            Quaternion scanStartRotation,
            Vector3 scanEndPosition,
            Quaternion scanEndRotation,
            float scanDuration)
        {
            int rayCount = localRayDirections.Length;
            EnsureBatchedRaycastBuffers(rayCount);

            QueryParameters queryParameters = new QueryParameters(scanLayerMask, false, queryTriggerInteraction, false);
            float rayDistance = maxDistance + rayStartOffset;

            for (int i = 0; i < rayCount; i++)
            {
                float scanProgress = GetScanProgress(i, rayCount);
                Vector3 rayPosition = Vector3.Lerp(scanStartPosition, scanEndPosition, scanProgress);
                Quaternion rayRotation = Quaternion.Slerp(scanStartRotation, scanEndRotation, scanProgress);
                Vector3 worldDirection = rayRotation * localRayDirections[i];
                raycastCommands[i] = new RaycastCommand(
                    rayPosition + worldDirection * rayStartOffset,
                    worldDirection,
                    queryParameters,
                    rayDistance
                );
            }

            JobHandle raycastHandle = RaycastCommand.ScheduleBatch(
                raycastCommands,
                raycastHits,
                Mathf.Max(1, raycastCommandsPerJob),
                1
            );
            raycastHandle.Complete();

            int pointCount = 0;
            for (int i = 0; i < rayCount; i++)
            {
                RaycastHit hit = raycastHits[i];
                if (hit.collider == null)
                {
                    continue;
                }

                float scanProgress = GetScanProgress(i, rayCount);
                Vector3 rayPosition = Vector3.Lerp(scanStartPosition, scanEndPosition, scanProgress);
                Quaternion rayRotation = Quaternion.Slerp(scanStartRotation, scanEndRotation, scanProgress);
                TryWritePoint(hit, i, rayPosition, rayRotation, scanDuration, ref pointCount);
            }

            return pointCount;
        }

        private int CollectPointsImmediate(
            Vector3 scanStartPosition,
            Quaternion scanStartRotation,
            Vector3 scanEndPosition,
            Quaternion scanEndRotation,
            float scanDuration)
        {
            int pointCount = 0;
            float rayDistance = maxDistance + rayStartOffset;
            int rayCount = localRayDirections.Length;

            for (int i = 0; i < rayCount; i++)
            {
                float scanProgress = GetScanProgress(i, rayCount);
                Vector3 rayPosition = Vector3.Lerp(scanStartPosition, scanEndPosition, scanProgress);
                Quaternion rayRotation = Quaternion.Slerp(scanStartRotation, scanEndRotation, scanProgress);
                Vector3 worldDirection = rayRotation * localRayDirections[i];
                Vector3 rayOrigin = rayPosition + worldDirection * rayStartOffset;

                if (!Physics.Raycast(rayOrigin, worldDirection, out RaycastHit hit, rayDistance, scanLayerMask, queryTriggerInteraction))
                {
                    continue;
                }

                TryWritePoint(hit, i, rayPosition, rayRotation, scanDuration, ref pointCount);
            }

            return pointCount;
        }

        private void TryWritePoint(
            RaycastHit hit,
            int rayIndex,
            Vector3 rayPosition,
            Quaternion rayRotation,
            float scanDuration,
            ref int pointCount)
        {
            float measuredDistance = hit.distance + rayStartOffset;
            float reflectivity = EstimateReflectivity(hit);
            float detectionLimit = GetDetectionLimit(reflectivity);
            if (measuredDistance < minDistance || measuredDistance > detectionLimit)
            {
                return;
            }

            // Express the return in the LiDAR frame at this point's acquisition
            // time. FAST-LIO can then deskew it into the scan-end frame using IMU.
            // Quaternion inversion intentionally avoids applying Transform scale.
            Vector3 localPoint = Quaternion.Inverse(rayRotation) * (hit.point - rayPosition);
            if (rangePrecisionStdDev > 0f)
            {
                float rangeNoise = GetGaussianNoise(scanFrameIndex, rayIndex) * rangePrecisionStdDev;
                localPoint += localRayDirections[rayIndex] * rangeNoise;
            }

            Vector3 rosPoint = localPoint.Unity2Ros();
            float intensity = Mathf.Max(minIntensity, reflectivity * (1f - Mathf.InverseLerp(minDistance, maxDistance, measuredDistance)));

            int baseIndex = pointCount * FloatFieldsPerPoint;
            pointBuffer[baseIndex + 0] = rosPoint.x;
            pointBuffer[baseIndex + 1] = rosPoint.y;
            pointBuffer[baseIndex + 2] = rosPoint.z;
            pointBuffer[baseIndex + 3] = intensity;
            pointTagBuffer[pointCount] = 0;
            pointLineBuffer[pointCount] = (byte)(rayIndex % LivoxMid360LineCount);
            pointTimestampBuffer[pointCount] = GetPointOffsetTimeNanoseconds(rayIndex, scanDuration);
            pointCount++;
        }

        private static float GetScanProgress(int rayIndex, int rayCount)
        {
            return rayCount <= 1 ? 0f : rayIndex / (float)(rayCount - 1);
        }

        private double GetPointOffsetTimeNanoseconds(int rayIndex, float scanDuration)
        {
            int rayCount = localRayDirections == null ? 0 : localRayDirections.Length;
            if (rayCount <= 1)
            {
                return 0.0;
            }

            return GetScanProgress(rayIndex, rayCount) * scanDuration * NanosecondsPerSecond;
        }

        private string GetNodeName()
        {
            StringBuilder builder = new StringBuilder("lidar_node_");

            foreach (char c in gameObject.name)
            {
                if (char.IsLetterOrDigit(c) || c == '_')
                {
                    builder.Append(char.ToLowerInvariant(c));
                }
                else
                {
                    builder.Append('_');
                }
            }

            return builder.ToString().TrimEnd('_');
        }
    }
}
