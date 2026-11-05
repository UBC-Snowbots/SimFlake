// Lit, double-sided, alpha-tested foliage for the built-in pipeline (shrubs, grass cards).
// The TerrainSampleAssets plant materials use an unlit Shader Graph, which looks flat; this one takes their
// BaseColor (alpha = cutout) and Normal textures. Works for terrain trees and instanced terrain details.
Shader "SimFlake/Foliage"
{
    Properties
    {
        _Color ("Tint", Color) = (1, 1, 1, 1)
        _MainTex ("Base color (A = cutout)", 2D) = "white" {}
        [Normal] _BumpMap ("Normal", 2D) = "bump" {}
        _Cutoff ("Alpha cutoff", Range(0, 1)) = 0.45
        _Saturation ("Saturation", Range(0, 1.5)) = 1
        _Translucency ("Back-lit glow", Range(0, 1)) = 0.35
    }

    SubShader
    {
        Tags { "Queue" = "AlphaTest" "RenderType" = "TransparentCutout" "IgnoreProjector" = "True" }
        Cull Off
        LOD 200

        CGPROGRAM
        #pragma surface surf Foliage alphatest:_Cutoff addshadow fullforwardshadows
        #pragma multi_compile_instancing
        #pragma target 3.0

        sampler2D _MainTex, _BumpMap;
        fixed4 _Color;
        half _Translucency, _Saturation;

        struct Input
        {
            float2 uv_MainTex;
            fixed facing : VFACE;
        };

        // Lambert plus a little light through the leaves when lit from behind
        half4 LightingFoliage(SurfaceOutput s, half3 lightDir, half atten)
        {
            half ndl = dot(s.Normal, lightDir);
            half diff = saturate(ndl) + _Translucency * saturate(-ndl);
            half4 c;
            c.rgb = s.Albedo * _LightColor0.rgb * (diff * atten);
            c.a = s.Alpha;
            return c;
        }

        void surf(Input IN, inout SurfaceOutput o)
        {
            fixed4 c = tex2D(_MainTex, IN.uv_MainTex) * _Color;
            o.Albedo = lerp(dot(c.rgb, half3(0.3, 0.59, 0.11)).xxx, c.rgb, _Saturation);
            o.Alpha = c.a;
            float3 n = UnpackNormal(tex2D(_BumpMap, IN.uv_MainTex));
            n.z *= IN.facing > 0 ? 1 : -1; // back faces get a flipped normal so they light correctly
            o.Normal = n;
        }
        ENDCG
    }
    Fallback "Legacy Shaders/Transparent/Cutout/VertexLit"
}
