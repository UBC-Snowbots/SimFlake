// Generated variants share SoilTerrain.cginc. First pass (layers 1-4). Tessellated: ruts and relief are real geometry.
Shader "SimFlake/SoilTerrainTess"
{
    Properties
    {
        [HideInInspector] _TerrainHolesTexture("Holes Map (RGB)", 2D) = "white" {}
    }
    SubShader
    {
        Tags { "Queue" = "Geometry-100" "RenderType" = "Opaque" "TerrainCompatible" = "True" }
        CGPROGRAM
        #pragma surface surf Standard vertex:SoilVert finalcolor:SoilFinalColor finalgbuffer:SoilFinalGBuffer fullforwardshadows nometa addshadow tessellate:SoilTess
        #pragma multi_compile_fog
        #pragma target 4.6
        #pragma multi_compile_local_fragment __ _ALPHATEST_ON
        #pragma multi_compile_local __ _NORMALMAP
        #include "UnityPBSLighting.cginc"
        #define SOIL_TESSELLATION
        #include "SoilTerrain.cginc"
        void surf(Input IN, inout SurfaceOutputStandard o) { SoilSurf(IN, o); }
        ENDCG
        UsePass "Hidden/Nature/Terrain/Utilities/PICKING"
        UsePass "Hidden/Nature/Terrain/Utilities/SELECTION"
    }
    Dependency "AddPassShader" = "Hidden/SimFlake/SoilTerrainTessAdd"
    Dependency "BaseMapShader" = "Hidden/TerrainEngine/Splatmap/Standard-Base"
    Dependency "BaseMapGenShader" = "Hidden/TerrainEngine/Splatmap/Standard-BaseGen"
    Fallback "Nature/Terrain/Standard"
}
