// Additive pass of SimFlake/SiteTerrain for terrain layers 5-8.
Shader "Hidden/SimFlake/SiteTerrainAdd"
{
    Properties
    {
        _MacroTex ("Ground colour (whole site)", 2D) = "gray" {}
        _MacroST ("Terrain uv -> ground colour uv (scale xy, offset zw)", Vector) = (1, 1, 0, 0)
        [HideInInspector] _TerrainHolesTexture("Holes Map (RGB)", 2D) = "white" {}
    }
    SubShader
    {
        Tags { "Queue" = "Geometry-99" "IgnoreProjector" = "True" "RenderType" = "Opaque" "TerrainCompatible" = "True" }
        CGPROGRAM
        #pragma surface surf Standard decal:add vertex:SplatmapVert finalcolor:SplatmapFinalColor finalgbuffer:SplatmapFinalGBuffer fullforwardshadows nometa
        #pragma instancing_options assumeuniformscaling nomatrices nolightprobe nolightmap forwardadd
        #pragma multi_compile_fog
        #pragma target 3.0
        #include "UnityPBSLighting.cginc"
        #pragma multi_compile_local_fragment __ _ALPHATEST_ON
        #pragma multi_compile_local __ _NORMALMAP
        #define TERRAIN_SPLAT_ADDPASS
        #define TERRAIN_STANDARD_SHADER
        #define TERRAIN_INSTANCED_PERPIXEL_NORMAL
        #define TERRAIN_SURFACE_OUTPUT SurfaceOutputStandard
        #include "TerrainSplatmapCommon.cginc"
        #include "SiteTerrain.cginc"
        ENDCG
    }
    Fallback "Hidden/TerrainEngine/Splatmap/Standard-AddPass"
}
