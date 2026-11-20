// Unity's standard terrain shader + one ground-colour map over the whole site (an aerial photo with the lighting taken
// out). The terrain layers are mid-grey detail textures: albedo = layers * 2 * ground colour. Used by the URC site world
// (UrcSiteBuilder). In Play the soil shaders take over and SoilRenderer hands them the same map (_MacroTex / _MacroST).
Shader "SimFlake/SiteTerrain"
{
    Properties
    {
        _MacroTex ("Ground colour (whole site)", 2D) = "gray" {}
        _MacroST ("Terrain uv -> ground colour uv (scale xy, offset zw)", Vector) = (1, 1, 0, 0)
        [HideInInspector] _MainTex ("BaseMap (RGB)", 2D) = "white" {}
        [HideInInspector] _Color ("Main Color", Color) = (1,1,1,1)
        [HideInInspector] _TerrainHolesTexture("Holes Map (RGB)", 2D) = "white" {}
    }
    SubShader
    {
        Tags { "Queue" = "Geometry-100" "RenderType" = "Opaque" "TerrainCompatible" = "True" }
        CGPROGRAM
        #pragma surface surf Standard vertex:SplatmapVert finalcolor:SplatmapFinalColor finalgbuffer:SplatmapFinalGBuffer addshadow fullforwardshadows
        #pragma instancing_options assumeuniformscaling nomatrices nolightprobe nolightmap forwardadd
        #pragma multi_compile_fog
        #pragma target 3.0
        #include "UnityPBSLighting.cginc"
        #pragma multi_compile_local_fragment __ _ALPHATEST_ON
        #pragma multi_compile_local __ _NORMALMAP
        #define TERRAIN_STANDARD_SHADER
        #define TERRAIN_INSTANCED_PERPIXEL_NORMAL
        #define TERRAIN_SURFACE_OUTPUT SurfaceOutputStandard
        #include "TerrainSplatmapCommon.cginc"
        #include "SiteTerrain.cginc"
        ENDCG
        UsePass "Hidden/Nature/Terrain/Utilities/PICKING"
        UsePass "Hidden/Nature/Terrain/Utilities/SELECTION"
    }
    Dependency "AddPassShader" = "Hidden/SimFlake/SiteTerrainAdd"
    Dependency "BaseMapShader" = "Hidden/TerrainEngine/Splatmap/Standard-Base"
    Dependency "BaseMapGenShader" = "Hidden/TerrainEngine/Splatmap/Standard-BaseGen"
    Fallback "Nature/Terrain/Standard"
}
