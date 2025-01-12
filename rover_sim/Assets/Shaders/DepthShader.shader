Shader "Custom/DepthShader"
{
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Pass
        {
            ZWrite On
            ZTest LEqual
            Cull Off
            Fog { Mode Off }
            ColorMask RGBA

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata_t
            {
                float4 vertex : POSITION;
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
                float depth : DEPTH;
            };

            v2f vert(appdata_t v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.depth = o.pos.z / o.pos.w;
                return o;
            }

            half4 frag(v2f i) : SV_Target
            {
                return half4(i.depth, i.depth, i.depth, 1.0);
            }
            ENDCG
        }
    }
}
