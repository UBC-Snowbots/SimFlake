// Generated variants share SoilTerrain.cginc. Additive pass for terrain layers 5-8. Tessellated: ruts and relief are real geometry.
Shader "Hidden/SimFlake/SoilTerrainTessAdd"
{
    Properties
    {
        [HideInInspector] _TerrainHolesTexture("Holes Map (RGB)", 2D) = "white" {}
    }
    SubShader
    {
        Tags { "Queue" = "Geometry-99" "IgnoreProjector" = "True" "RenderType" = "Opaque" "TerrainCompatible" = "True" }
        CGPROGRAM
        #pragma surface surf Standard vertex:SoilVert finalcolor:SoilFinalColor finalgbuffer:SoilFinalGBuffer fullforwardshadows nometa decal:add tessellate:SoilTess
        #pragma multi_compile_fog
        #pragma target 4.6
        #pragma multi_compile_local_fragment __ _ALPHATEST_ON
        #pragma multi_compile_local __ _NORMALMAP
        #include "UnityPBSLighting.cginc"
        #define TERRAIN_SPLAT_ADDPASS
        #define SOIL_TESSELLATION
        #include "SoilTerrain.cginc"
        void surf(Input IN, inout SurfaceOutputStandard o) { SoilSurf(IN, o); }
        ENDCG
    }
    Fallback "Nature/Terrain/Standard"
}
