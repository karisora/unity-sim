Shader "Hidden/Realsense/DepthOnly"
{
    SubShader
    {
        Tags { "RenderType" = "Opaque" }

        Pass
        {
            ZTest LEqual
            ZWrite On
            Cull Back

            CGPROGRAM
            #pragma target 3.0
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
            };

            struct v2f
            {
                float4 position : SV_POSITION;
                float eyeDepth : TEXCOORD0;
            };

            v2f vert(appdata input)
            {
                v2f output;
                output.position = UnityObjectToClipPos(input.vertex);
                output.eyeDepth = -UnityObjectToViewPos(input.vertex).z;
                return output;
            }

            float frag(v2f input) : SV_Target
            {
                // Linear distance along the optical Z axis in Unity world units (metres).
                return input.eyeDepth;
            }
            ENDCG
        }
    }

    Fallback Off
}
