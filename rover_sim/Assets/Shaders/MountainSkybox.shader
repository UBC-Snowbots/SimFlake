// Procedural skybox: sky gradient, sun or moon disc (follows the scene's directional light / RenderSettings.sun),
// two layers of mountain silhouettes on the horizon with snow caps, drifting clouds, stars and an overcast mode.
// No textures, so it costs nothing in the repo. Tweak the look from the material inspector; SkyEnvironment
// overrides the time-of-day and weather properties at runtime.
Shader "SimFlake/MountainSkybox"
{
    Properties
    {
        [Header(Sky)]
        _ZenithColor ("Zenith", Color) = (0.20, 0.30, 0.55, 1)
        _HorizonColor ("Horizon", Color) = (0.95, 0.66, 0.45, 1)
        _GroundColor ("Below horizon", Color) = (0.36, 0.30, 0.27, 1)
        _Exposure ("Exposure", Range(0.2, 3)) = 1

        [Header(Sun)]
        _SunColor ("Sun color", Color) = (1.0, 0.72, 0.42, 1)
        _SunSize ("Sun size", Range(0.00005, 0.005)) = 0.0004
        _SunGlow ("Sun glow", Range(0, 3)) = 1.1
        [HDR] _DiscColor ("Sun/moon disc", Color) = (2.0, 1.84, 1.5, 1)
        _StarIntensity ("Stars", Range(0, 2)) = 0

        [Header(Mountains)]
        _MountainHeight ("Max height (degrees)", Range(0, 25)) = 9
        _FarMountainColor ("Far range", Color) = (0.50, 0.45, 0.55, 1)
        _NearMountainColor ("Near range", Color) = (0.25, 0.22, 0.27, 1)
        _AlpenGlow ("Alpenglow (faces away from sun)", Color) = (0.95, 0.55, 0.50, 1)
        _GlowAmount ("Alpenglow strength", Range(0, 1)) = 1
        _SnowLine ("Snow line (fraction of max)", Range(0, 1.2)) = 0.62
        _Seed ("Range seed", Float) = 3.7
        _LandTint ("Land tint (mountains, ground)", Color) = (1, 1, 1, 1)

        [Header(Clouds)]
        _CloudCover ("Cover", Range(0, 1)) = 0.42
        _CloudScale ("Scale", Range(0.1, 3)) = 0.9
        _CloudSpeed ("Drift speed", Range(0, 0.05)) = 0.004
        _CloudShadow ("Shadow color", Color) = (0.42, 0.36, 0.48, 1)
        _CloudLit ("Lit color", Color) = (1.0, 0.70, 0.52, 1)
        _Overcast ("Overcast (greys the sky, hides the sun)", Range(0, 1)) = 0
    }

    SubShader
    {
        Tags { "Queue" = "Background" "RenderType" = "Background" "PreviewType" = "Skybox" }
        Cull Off ZWrite Off

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"
            #include "Lighting.cginc"

            fixed4 _ZenithColor, _HorizonColor, _GroundColor, _SunColor;
            fixed4 _FarMountainColor, _NearMountainColor, _AlpenGlow, _CloudShadow, _CloudLit, _LandTint;
            float4 _DiscColor;
            float _Exposure, _SunSize, _SunGlow, _MountainHeight, _SnowLine, _Seed;
            float _CloudCover, _CloudScale, _CloudSpeed, _StarIntensity, _Overcast, _GlowAmount;

            struct v2f { float4 pos : SV_POSITION; float3 dir : TEXCOORD0; };

            v2f vert(appdata_base v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.dir = v.vertex.xyz;
                return o;
            }

            float hash(float2 p)
            {
                p = frac(p * float2(123.34, 456.21));
                p += dot(p, p + 45.32);
                return frac(p.x * p.y);
            }

            float vnoise(float2 p)
            {
                float2 i = floor(p), f = frac(p);
                float2 u = f * f * (3 - 2 * f);
                return lerp(lerp(hash(i), hash(i + float2(1, 0)), u.x),
                            lerp(hash(i + float2(0, 1)), hash(i + 1), u.x), u.y);
            }

            float fbm(float2 p)
            {
                float s = 0, a = 0.5;
                for (int k = 0; k < 5; k++) { s += a * vnoise(p); p = p * 2.03 + 17.1; a *= 0.5; }
                return s;
            }

            // Ridged noise around the horizon. Sampling on a circle keeps it seamless at 360 deg.
            float ridge(float azimuth, float radius, float2 offset)
            {
                float2 c = float2(cos(azimuth), sin(azimuth)) * radius + offset;
                float s = 0, a = 0.55;
                for (int k = 0; k < 5; k++)
                {
                    float n = 1 - abs(2 * vnoise(c) - 1);
                    s += a * n * n;
                    c = c * 2.1 + 9.7; a *= 0.45;
                }
                return s; // ~0..1
            }

            fixed4 frag(v2f i) : SV_Target
            {
                float3 d = normalize(i.dir);
                float3 sunDir = normalize(_WorldSpaceLightPos0.xyz + float3(0, 1e-4, 0));
                float mu = dot(d, sunDir);
                float elev = asin(clamp(d.y, -1, 1));
                float az = atan2(d.x, d.z);
                float sunAz = atan2(sunDir.x, sunDir.z);
                float towardSun = 0.5 + 0.5 * cos(az - sunAz); // 1 = looking at the sun's side of the horizon

                // --- sky
                float t = saturate(d.y);
                float3 horizon = _HorizonColor.rgb + _SunColor.rgb * 0.45 * pow(towardSun, 3) * (1 - 0.8 * _Overcast);
                float3 sky = lerp(horizon, _ZenithColor.rgb, pow(t, 0.45));
                // overcast: grey, flat sky with no sun glow
                float lum = dot(sky, float3(0.3, 0.55, 0.15));
                sky = lerp(sky, lum * float3(0.97, 0.99, 1.03), _Overcast * 0.8);
                float glow = _SunGlow * (1 - 0.85 * _Overcast);
                sky += _SunColor.rgb * glow * (0.35 * pow(saturate(mu), 12) + 0.25 * pow(saturate(mu), 200));

                // stars (cells on a direction grid; fade out near the horizon)
                if (_StarIntensity > 0 && d.y > 0)
                {
                    float3 sd = d * 260;
                    float3 cell = floor(sd);
                    float h = hash(cell.xy + cell.z * 17.13);
                    float r = length(frac(sd) - 0.5);
                    float star = step(0.9965, h) * smoothstep(0.4, 0.05, r) * (0.4 + 0.6 * hash(cell.zx + 3.1));
                    sky += star * _StarIntensity * smoothstep(0.03, 0.25, d.y) * float3(0.9, 0.95, 1.1);
                }

                float disc = smoothstep(1 - _SunSize, 1 - _SunSize * 0.6, mu) * (1 - _Overcast);
                sky = lerp(sky, _DiscColor.rgb, disc);

                // --- clouds on a virtual plane above the camera
                if (d.y > 0.01)
                {
                    float2 uv = d.xz / (d.y + 0.12) * _CloudScale + _Time.y * _CloudSpeed * float2(1, 0.3);
                    float n = fbm(uv + _Seed);
                    float cov = smoothstep(1 - _CloudCover, 1 - _CloudCover + 0.25, n) * smoothstep(0.01, 0.15, d.y);
                    float thick = fbm(uv * 1.7 + 4.2);
                    float3 cloud = lerp(_CloudShadow.rgb, _CloudLit.rgb, saturate(0.25 + 0.75 * pow(towardSun, 1.5) * (1.2 - thick)));
                    cloud += _SunColor.rgb * 0.8 * pow(saturate(mu), 18) * (1 - cov); // bright rims near the sun
                    sky = lerp(sky, cloud, cov * 0.9);
                }

                // --- mountains: far range first, near range drawn over it
                float maxH = radians(_MountainHeight);
                float hFar = maxH * (0.25 + 0.95 * ridge(az, 2.2, float2(_Seed, 1.3)));
                float hNear = maxH * (0.05 + 0.55 * ridge(az, 3.4, float2(7.1 + _Seed, -4.0)));
                float aa = fwidth(elev) * 1.5;

                float3 col = sky;
                float farMask = 1 - smoothstep(hFar - aa, hFar + aa, elev);
                if (farMask > 0)
                {
                    float3 m = lerp(_FarMountainColor.rgb, _AlpenGlow.rgb, 0.45 * (1 - towardSun) * _GlowAmount);
                    // ragged snow line: varies along the range and streaks down gullies
                    float snowLine = maxH * (_SnowLine + 0.35 * (fbm(float2(az * 9 + _Seed, 2.3)) - 0.5));
                    float streak = 0.07 * maxH * (vnoise(float2(az * 60, 1.7)) - 0.5);
                    float snow = smoothstep(snowLine, snowLine + 0.004, elev + streak) * smoothstep(0, 0.15 * maxH, hFar - snowLine);
                    m = lerp(m, lerp(float3(0.95, 0.93, 0.95), float3(1.0, 0.75, 0.65), (1 - towardSun) * _GlowAmount) * 0.9, snow * 0.85);
                    m *= _LandTint.rgb;
                    m = lerp(horizon, m, 0.65);                    // distance haze
                    m = lerp(horizon, m, saturate(elev / (hFar * 0.6) + 0.15)); // thicker haze at the base
                    col = lerp(col, m, farMask);
                }
                float nearMask = 1 - smoothstep(hNear - aa, hNear + aa, elev);
                if (nearMask > 0)
                {
                    float3 m = lerp(_NearMountainColor.rgb, _NearMountainColor.rgb * 1.6 * _AlpenGlow.rgb, 0.5 * (1 - towardSun) * _GlowAmount);
                    m *= 0.85 + 0.3 * vnoise(float2(az * 120, elev * 300)); // rocky texture
                    m *= _LandTint.rgb;
                    m = lerp(horizon, m, saturate(elev / (hNear * 0.8) + 0.35));
                    col = lerp(col, m, nearMask);
                }

                // --- below the horizon (normally hidden by terrain)
                if (elev < 0) col = lerp(horizon * 0.8, _GroundColor.rgb * _LandTint.rgb, saturate(-d.y * 6));

                // fade the bottom of the sky into the scene's fog so the far terrain edge blends in
                if (unity_FogColor.a > 0) col = lerp(col, unity_FogColor.rgb, exp(-max(elev, 0) * 45) * 0.85);

                return fixed4(col * _Exposure, 1);
            }
            ENDCG
        }
    }
    Fallback Off
}
