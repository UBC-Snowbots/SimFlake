// Terrain splat shading (same look as Unity's Nature/Terrain/Standard) + the soil surface from SoilWorld:
// small-scale relief and wheel ruts as real displacement (tessellated variants) and as shading (all variants).
// Globals come from SoilRenderer; _SoilReliefTex/_SoilTerrainRect per terrain (material property block).
// Non-instanced terrains only (SoilRenderer turns Draw Instanced off on soil terrains).
#ifndef SOIL_TERRAIN_INCLUDED
#define SOIL_TERRAIN_INCLUDED

struct Input
{
    float4 tc;
    float3 worldPos;
    UNITY_FOG_COORDS(0)
};

sampler2D _Control;
float4 _Control_ST, _Control_TexelSize;
// Separate textures and samplers: each normal map and _MacroTex borrow a layer texture's sampler. With one sampler per
// texture the forward passes went over the 16 sampler limit in some light variants.
UNITY_DECLARE_TEX2D(_Splat0); UNITY_DECLARE_TEX2D(_Splat1); UNITY_DECLARE_TEX2D(_Splat2); UNITY_DECLARE_TEX2D(_Splat3);
float4 _Splat0_ST, _Splat1_ST, _Splat2_ST, _Splat3_ST;
half _Metallic0, _Metallic1, _Metallic2, _Metallic3;
half _Smoothness0, _Smoothness1, _Smoothness2, _Smoothness3;
#ifdef _NORMALMAP
UNITY_DECLARE_TEX2D_NOSAMPLER(_Normal0); UNITY_DECLARE_TEX2D_NOSAMPLER(_Normal1);
UNITY_DECLARE_TEX2D_NOSAMPLER(_Normal2); UNITY_DECLARE_TEX2D_NOSAMPLER(_Normal3);
float _NormalScale0, _NormalScale1, _NormalScale2, _NormalScale3;
#endif
#ifdef _ALPHATEST_ON
sampler2D _TerrainHolesTexture;
#endif

UNITY_DECLARE_TEX2D_NOSAMPLER(_MacroTex);   // optional ground colour over a whole site (aerial photo, UrcSite): albedo = layers * 2 * macro
float4 _MacroST;             // terrain uv -> macro uv (scale xy, offset zw)
float _MacroOn;              // 0 when the terrain has no macro map (set per terrain by SoilRenderer)

sampler2D _SoilTex;          // window around the rover: R = deformation (m), G = disturbed 0..1
float4 _SoilWindow;          // xy = centre (world xz), z = 1 / window size (m), w = half size (m)
float4 _SoilParams;          // x = cell (m), y = tessellation edge (px), z = tessellation distance (m), w = soil on (0/1)
sampler2D _SoilNoiseTex;
float4 _SoilNoiseScale;      // x = 1 / fine repeat (m), y = 1 / coarse repeat (m), z = relief scale
sampler2D _SoilReliefTex;    // per terrain: R = fine amplitude (m), G = coarse amplitude (m), B = darkening of disturbed soil
float4 _SoilTerrainRect;     // xy = terrain origin (world xz), zw = 1 / size
float4 _SoilReliefTex_TexelSize;

// x = height above the undisturbed terrain (m), y = disturbed 0..1
float2 SoilSurface(float2 wxz, float3 relief)
{
    float2 d = abs(wxz - _SoilWindow.xy);
    float inWin = saturate((_SoilWindow.w - max(d.x, d.y)) * 2.0) * _SoilParams.w;
    float4 s = tex2Dlod(_SoilTex, float4(wxz * _SoilWindow.z, 0, 0));
    float h = relief.x * tex2Dlod(_SoilNoiseTex, float4(wxz * _SoilNoiseScale.x, 0, 0)).r
            + relief.y * tex2Dlod(_SoilNoiseTex, float4(wxz * _SoilNoiseScale.y + float2(0.37, 0.11), 0, 0)).r;
    return float2(h * _SoilNoiseScale.z * _SoilParams.w + s.r * inWin, s.g * inWin);
}

float3 SoilRelief(float2 wxz)
{
    float2 t = (wxz - _SoilTerrainRect.xy) * _SoilTerrainRect.zw;
    float2 uv = (t * (_SoilReliefTex_TexelSize.zw - 1.0) + 0.5) * _SoilReliefTex_TexelSize.xy;
    return tex2Dlod(_SoilReliefTex, float4(uv, 0, 0)).rgb;
}

#ifdef SOIL_TESSELLATION
// With tessellation Unity calls the one-argument form on the new vertices (domain shader): move them onto the soil
// surface (along world up). The two-argument form below then runs as usual.
void SoilVert(inout appdata_full v)
{
    float3 w = mul(unity_ObjectToWorld, v.vertex).xyz;
    v.vertex.y += SoilSurface(w.xz, SoilRelief(w.xz)).x;
}
#endif

void SoilVert(inout appdata_full v, out Input data)
{
    UNITY_INITIALIZE_OUTPUT(Input, data);
    v.tangent.xyz = cross(v.normal, float3(0, 0, 1));
    v.tangent.w = -1;
    data.tc.xy = v.texcoord.xy;
    float4 pos = UnityObjectToClipPos(v.vertex);
    UNITY_TRANSFER_FOG(data, pos);
}

#ifdef SOIL_TESSELLATION
#include "Tessellation.cginc"
float SoilEdgeFactor(float3 a, float3 b)
{
    float3 mid = 0.5 * (a + b);
    float dist = distance(mid, _WorldSpaceCameraPos);
    float f = max(distance(a, b) * _ScreenParams.y / (_SoilParams.y * dist), 1.0);
    return lerp(f, 1.0, saturate((dist - 0.6 * _SoilParams.z) / (0.4 * _SoilParams.z)));
}

float4 SoilTess(appdata_full v0, appdata_full v1, appdata_full v2)
{
    float3 p0 = mul(unity_ObjectToWorld, v0.vertex).xyz;
    float3 p1 = mul(unity_ObjectToWorld, v1.vertex).xyz;
    float3 p2 = mul(unity_ObjectToWorld, v2.vertex).xyz;
    float4 t;
    t.x = SoilEdgeFactor(p1, p2);
    t.y = SoilEdgeFactor(p2, p0);
    t.z = SoilEdgeFactor(p0, p1);
    t.w = (t.x + t.y + t.z) / 3.0;
    return t;
}
#endif

void SoilSurf(Input IN, inout SurfaceOutputStandard o)
{
    // Terrain uv from the world position: with tessellation the vertex function's Input is not filled in, which made
    // every pixel sample the splat maps at (0,0) (whole terrain drawn as one layer).
    IN.tc.xy = (IN.worldPos.xz - _SoilTerrainRect.xy) * _SoilTerrainRect.zw;
    #ifdef _ALPHATEST_ON
        clip(tex2D(_TerrainHolesTexture, IN.tc.xy).r < 0.0005 ? -1 : 1);
    #endif
    float2 splatUV = (IN.tc.xy * (_Control_TexelSize.zw - 1.0f) + 0.5f) * _Control_TexelSize.xy;
    half4 splat_control = tex2D(_Control, splatUV);
    half weight = dot(splat_control, half4(1, 1, 1, 1));
    #ifdef TERRAIN_SPLAT_ADDPASS
        clip(weight == 0.0f ? -1 : 1);
    #endif
    splat_control /= (weight + 1e-3f);

    float2 uv0 = TRANSFORM_TEX(IN.tc.xy, _Splat0), uv1 = TRANSFORM_TEX(IN.tc.xy, _Splat1);
    float2 uv2 = TRANSFORM_TEX(IN.tc.xy, _Splat2), uv3 = TRANSFORM_TEX(IN.tc.xy, _Splat3);
    half4 mixed = 0;
    mixed += splat_control.r * UNITY_SAMPLE_TEX2D(_Splat0, uv0) * half4(1, 1, 1, _Smoothness0);
    mixed += splat_control.g * UNITY_SAMPLE_TEX2D(_Splat1, uv1) * half4(1, 1, 1, _Smoothness1);
    mixed += splat_control.b * UNITY_SAMPLE_TEX2D(_Splat2, uv2) * half4(1, 1, 1, _Smoothness2);
    mixed += splat_control.a * UNITY_SAMPLE_TEX2D(_Splat3, uv3) * half4(1, 1, 1, _Smoothness3);

    mixed.rgb *= lerp(half3(1, 1, 1), 2.0 * UNITY_SAMPLE_TEX2D_SAMPLER(_MacroTex, _Splat0, IN.tc.xy * _MacroST.xy + _MacroST.zw).rgb, _MacroOn);

    fixed3 n = fixed3(0, 0, 1);
    #ifdef _NORMALMAP
        n  = UnpackNormalWithScale(UNITY_SAMPLE_TEX2D_SAMPLER(_Normal0, _Splat0, uv0), _NormalScale0) * splat_control.r;
        n += UnpackNormalWithScale(UNITY_SAMPLE_TEX2D_SAMPLER(_Normal1, _Splat1, uv1), _NormalScale1) * splat_control.g;
        n += UnpackNormalWithScale(UNITY_SAMPLE_TEX2D_SAMPLER(_Normal2, _Splat2, uv2), _NormalScale2) * splat_control.b;
        n += UnpackNormalWithScale(UNITY_SAMPLE_TEX2D_SAMPLER(_Normal3, _Splat3, uv3), _NormalScale3) * splat_control.a;
        n.z += 1e-5f;
    #endif

    // Layers 5-8 (add pass) skip the soil shading: with 4 splats + 4 normal maps + shadows the pass is at the
    // 16-sampler limit. They still get the displaced geometry (vertex stage).
    #ifndef TERRAIN_SPLAT_ADDPASS
    // soil surface: slope of relief + ruts (tangent space here = world x, world z, up), darker where disturbed
    float2 wxz = IN.worldPos.xz;
    float3 relief = SoilRelief(wxz);
    float e = max(_SoilParams.x, 0.004);
    float2 c0 = SoilSurface(wxz, relief);
    float hx = SoilSurface(wxz + float2(e, 0), relief).x, hz = SoilSurface(wxz + float2(0, e), relief).x;
    float fade = saturate(1.5 - distance(IN.worldPos, _WorldSpaceCameraPos) / max(_SoilParams.z * 2.0, 1.0));
    float2 slope = float2(c0.x - hx, c0.x - hz) / e * fade;
    n = normalize(float3(n.xy + slope, n.z));
    float mark = c0.y * fade;
    // freshly pressed/turned soil is darker; rut floors a little smoother
    mixed.rgb *= 1.0 - relief.z * mark;
    mixed.a = lerp(mixed.a, mixed.a + 0.08, saturate(mark * 2.0 - 1.0));

    #endif
    o.Normal = n;
    o.Albedo = mixed.rgb;
    o.Alpha = weight;
    o.Smoothness = mixed.a;
    o.Metallic = dot(splat_control, half4(_Metallic0, _Metallic1, _Metallic2, _Metallic3));
}

void SoilFinalColor(Input IN, SurfaceOutputStandard o, inout fixed4 color)
{
    color *= o.Alpha;
    #ifdef TERRAIN_SPLAT_ADDPASS
        UNITY_APPLY_FOG_COLOR(IN.fogCoord, color, fixed4(0, 0, 0, 0));
    #else
        UNITY_APPLY_FOG(IN.fogCoord, color);
    #endif
}

void SoilFinalGBuffer(Input IN, SurfaceOutputStandard o, inout half4 outGBuffer0, inout half4 outGBuffer1, inout half4 outGBuffer2, inout half4 emission)
{
    UnityStandardDataApplyWeightToGbuffer(outGBuffer0, outGBuffer1, outGBuffer2, o.Alpha);
    emission *= o.Alpha;
}
#endif
