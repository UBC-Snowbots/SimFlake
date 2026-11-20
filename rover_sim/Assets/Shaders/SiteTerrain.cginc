// Shared by SiteTerrain / SiteTerrainAdd: Unity's standard splat mix, times the site's ground-colour map.
#ifndef SITE_TERRAIN_INCLUDED
#define SITE_TERRAIN_INCLUDED

half _Metallic0, _Metallic1, _Metallic2, _Metallic3;
half _Smoothness0, _Smoothness1, _Smoothness2, _Smoothness3;
sampler2D _MacroTex;
float4 _MacroST;

void surf(Input IN, inout SurfaceOutputStandard o)
{
    half4 splat_control;
    half weight;
    fixed4 mixedDiffuse;
    half4 defaultSmoothness = half4(_Smoothness0, _Smoothness1, _Smoothness2, _Smoothness3);
    SplatmapMix(IN, defaultSmoothness, splat_control, weight, mixedDiffuse, o.Normal);
    o.Albedo = mixedDiffuse.rgb * 2.0 * tex2D(_MacroTex, IN.tc.xy * _MacroST.xy + _MacroST.zw).rgb;
    o.Alpha = weight;
    o.Smoothness = mixedDiffuse.a;
    o.Metallic = dot(splat_control, half4(_Metallic0, _Metallic1, _Metallic2, _Metallic3));
}
#endif
