// Generated variants share SoilTerrain.cginc. First pass (layers 1-4). No tessellation: ruts and relief are shading only.
Shader "SimFlake/SoilTerrain"
{
    Properties
    {
        [HideInInspector] _TerrainHolesTexture("Holes Map (RGB)", 2D) = "white" {}
    }
    SubShader
    {
        Tags { "Queue" = "Geometry-100" "RenderType" = "Opaque" "TerrainCompatible" = "True" }
        CGPROGRAM
        #pragma surface surf Standard vertex:SoilVert finalcolor:SoilFinalColor finalgbuffer:SoilFinalGBuffer fullforwardshadows nometa addshadow
        #pragma multi_compile_fog
        #pragma target 3.0
        #pragma multi_compile_local_fragment __ _ALPHATEST_ON
        #pragma multi_compile_local __ _NORMALMAP
        #include "UnityPBSLighting.cginc"
        #include "SoilTerrain.cginc"
        void surf(Input IN, inout SurfaceOutputStandard o) { SoilSurf(IN, o); }
        ENDCG
        UsePass "Hidden/Nature/Terrain/Utilities/PICKING"
        UsePass "Hidden/Nature/Terrain/Utilities/SELECTION"
    }
    Dependency "AddPassShader" = "Hidden/SimFlake/SoilTerrainAdd"
    Dependency "BaseMapShader" = "Hidden/TerrainEngine/Splatmap/Standard-Base"
    Dependency "BaseMapGenShader" = "Hidden/TerrainEngine/Splatmap/Standard-BaseGen"
    Fallback "Nature/Terrain/Standard"
}
