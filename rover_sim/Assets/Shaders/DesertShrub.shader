// Cut-out cards for the generated desert shrubs (UrcSiteProps): texture atlas x vertex colour, double sided, lit softly.
// The meshes carry dome-shaped normals and a darker base in the vertex colours, so a handful of flat cards reads as
// one rounded plant. Works for terrain trees.
Shader "SimFlake/DesertShrub"
{
    Properties
    {
        _Color ("Tint", Color) = (1, 1, 1, 1)
        _MainTex ("Card atlas (A = cutout)", 2D) = "white" {}
        _Cutoff ("Alpha cutoff", Range(0, 1)) = 0.4
        _Translucency ("Back-lit glow", Range(0, 1)) = 0.3
    }

    SubShader
    {
        Tags { "Queue" = "AlphaTest" "RenderType" = "TransparentCutout" "IgnoreProjector" = "True" }
        Cull Off
        LOD 200

        CGPROGRAM
        #pragma surface surf Shrub alphatest:_Cutoff addshadow fullforwardshadows
        #pragma multi_compile_instancing
        #pragma target 3.0

        sampler2D _MainTex;
        fixed4 _Color;
        half _Translucency;

        struct Input
        {
            float2 uv_MainTex;
            fixed4 color : COLOR;
        };

        // wrapped Lambert: thin twiggy plants never go black on the shaded side
        half4 LightingShrub(SurfaceOutput s, half3 lightDir, half atten)
        {
            half ndl = dot(s.Normal, lightDir);
            half diff = saturate(ndl * 0.6 + 0.4) + _Translucency * saturate(-ndl);
            half4 c;
            c.rgb = s.Albedo * _LightColor0.rgb * (diff * atten);
            c.a = s.Alpha;
            return c;
        }

        void surf(Input IN, inout SurfaceOutput o)
        {
            fixed4 c = tex2D(_MainTex, IN.uv_MainTex);
            o.Albedo = c.rgb * _Color.rgb * IN.color.rgb;
            o.Alpha = c.a;
        }
        ENDCG
    }
    Fallback "Legacy Shaders/Transparent/Cutout/VertexLit"
}
