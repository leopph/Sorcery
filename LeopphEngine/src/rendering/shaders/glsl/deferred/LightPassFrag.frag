#version 410 core

#define MIN_SHADOW_BIAS 0.0001
#define MAX_SHADOW_BIAS 0.01

// General variables used in all cases

layout (location = 0) in vec2 in_TexCoords;

layout (location = 0) out vec4 out_FragColor;

uniform sampler2D u_NormTex;
uniform usampler2D u_ColorGlossTex;
uniform sampler2D u_DepthTex;

uniform vec3 u_CamPos;
uniform vec3 u_AmbientLight;
uniform mat4 u_CamViewProjInv;

struct Fragment
{
	vec3 pos;
	vec3 normal;
	vec3 diff;
	vec3 spec;
	float gloss;
};

// General functions

float CalcAtten(float constant, float linear, float quadratic, float dist)
{
	return 1.0 / (constant + linear * dist + quadratic * pow(dist, 2));
}

vec3 CalcBlinnPhong(Fragment frag, vec3 dirToLight, vec3 lightDiff, vec3 lightSpec)
{
	float diffuseDot = max(dot(dirToLight, frag.normal), 0);
	vec3 light = frag.diff * diffuseDot * lightDiff;

	if (diffuseDot > 0)
	{
		vec3 halfway = normalize(dirToLight + normalize(u_CamPos - frag.pos));
		light += frag.spec * pow(max(dot(frag.normal, halfway), 0), 4 * frag.gloss) * lightSpec;
	}

	return light;
}


// Only when there is a DirectionalLight
#if DIRLIGHT
struct DirLight
{
	vec3 direction;
	
	vec3 diffuseColor;
	vec3 specularColor;
};

uniform DirLight u_DirLight;

#if DIRLIGHT_SHADOW
uniform mat4 u_CascadeMatrices[NUM_CASCADES];
uniform float u_CascadeBoundsNdc[NUM_CASCADES];
uniform sampler2DShadow u_DirShadowMaps[NUM_CASCADES];

float CalcDirShadow(vec3 fragPos, float fragPosNdcZ, vec3 fragNormal)
{
	for (int i = 0; i < NUM_CASCADES; ++i)
	{
		if (fragPosNdcZ < u_CascadeBoundsNdc[i])
		{
			vec3 fragPosLightNorm = vec3(vec4(fragPos, 1) * u_CascadeMatrices[i]) * 0.5 + 0.5;
			float bias = max(MAX_SHADOW_BIAS * (1.0 - dot(fragNormal, -u_DirLight.direction)), MIN_SHADOW_BIAS);
			return texture(u_DirShadowMaps[i], vec3(fragPosLightNorm.xy, fragPosLightNorm.z - bias));
		}
	}
	return 1.0;
}
#endif
#endif


// Only when there are SpotLights
#if NUM_SPOTLIGHTS > 0
struct SpotLight
{
	vec3 position;
	vec3 direction;

	vec3 diffuseColor;
	vec3 specularColor;

	float constant;
	float linear;
	float quadratic;
	float range;
	
	float innerAngleCosine;
	float outerAngleCosine;
};

vec3 CalcSpotLightEffect(Fragment frag, SpotLight spotLight)
{
	vec3 dirToLight = spotLight.position - frag.pos;
	float dist = length(dirToLight);

	if (dist > spotLight.range)
	{
		return vec3(0);
	}

	dirToLight = normalize(dirToLight);

	/* Let theta be the angle between
	 * the direction vector pointing from the fragment to the light
	 * and the reverse of the light's direction vector. */
	float thetaCosine = dot(dirToLight, -spotLight.direction);

	/* Let epsilon be the difference between
	 * the cosines of the light's cutoff angles. */
	float epsilon = spotLight.innerAngleCosine - spotLight.outerAngleCosine;

	/* Determine if the frag is
	 * inside the inner angle, or
	 * between the inner and outer angles, or
	 * outside the outer angle. */
	float intensity = clamp((thetaCosine - spotLight.outerAngleCosine) / epsilon, 0.0, 1.0);

	if (intensity == 0)
	{
		return vec3(0);
	}

	vec3 spotLightEffect = CalcBlinnPhong(frag, dirToLight, spotLight.diffuseColor, spotLight.specularColor);
	spotLightEffect *= CalcAtten(spotLight.constant, spotLight.linear, spotLight.quadratic, dist);
	return spotLightEffect;
}

#if NUM_SPOTLIGHTS > NUM_SPOTLIGHT_SHADOWS
uniform SpotLight u_SpotLightsNoShadow[NUM_SPOTLIGHTS - NUM_SPOTLIGHT_SHADOWS];
#endif
#if NUM_SPOTLIGHT_SHADOWS > 0
uniform SpotLight u_SpotLightsShadow[NUM_SPOTLIGHT_SHADOWS];
uniform sampler2DShadow u_SpotShadowMaps[NUM_SPOTLIGHT_SHADOWS];
uniform mat4 u_SpotShadowMats[NUM_SPOTLIGHT_SHADOWS];

float CalcSpotShadow(int shadowIndex, vec3 dirToLight, vec3 fragPos, vec3 fragNormal)
{
	vec4 fragPosLightSpace = vec4(fragPos, 1) * u_SpotShadowMats[shadowIndex];
	vec3 normalizedPos = (fragPosLightSpace.xyz / fragPosLightSpace.w) * 0.5 + 0.5;
	float bias = max(MAX_SHADOW_BIAS * (1.0 - dot(fragNormal, dirToLight)), MIN_SHADOW_BIAS);
	return texture(u_SpotShadowMaps[shadowIndex], vec3(normalizedPos.xy, normalizedPos.z - bias));
}
#endif
#endif


// Only if there are PointLights
#if NUM_POINTLIGHTS > 0
struct PointLight
{
	vec3 position;

	vec3 diffuseColor;
	vec3 specularColor;

	float constant;
	float linear;
	float quadratic;
	float range;
};

vec3 CalcPointLightEffect(Fragment frag, PointLight pointLight)
{
	vec3 dirToLight = pointLight.position - frag.pos;
	float dist = length(dirToLight);

	if (dist > pointLight.range)
	{
		return vec3(0);
	}

	dirToLight = normalize(dirToLight);

	vec3 pointLightEffect = CalcBlinnPhong(frag, dirToLight, pointLight.diffuseColor, pointLight.specularColor);
	pointLightEffect *= CalcAtten(pointLight.constant, pointLight.linear, pointLight.quadratic, dist);
	return pointLightEffect;
}

#if NUM_POINTLIGHTS > NUM_POINTLIGHT_SHADOWS
uniform PointLight u_PointLightsNoShadow[NUM_POINTLIGHTS - NUM_POINTLIGHT_SHADOWS];
#endif
#if NUM_POINTLIGHT_SHADOWS > 0
uniform PointLight u_PointLightsShadow[NUM_POINTLIGHT_SHADOWS];
uniform samplerCubeShadow u_PointShadowMaps[NUM_POINTLIGHT_SHADOWS];

float CalcPointShadow(uint shadowIndex, vec3 fragPos, vec3 fragNormal, vec3 lightPos, float lightRange)
{
	vec3 dirToFrag = fragPos - lightPos;
	float bias = max(MAX_SHADOW_BIAS * (1.0 - dot(fragNormal, normalize(-dirToFrag))), MIN_SHADOW_BIAS);
	return texture(u_PointShadowMaps[shadowIndex], vec4(dirToFrag, (length(dirToFrag) / lightRange) - bias));
}
#endif
#endif


void main()
{	
	// Parse gbuffer contents
	Fragment frag;

	// Decode normal
	vec2 fragCompNorm = texture(u_NormTex, in_TexCoords).xy;
	frag.normal.z = dot(fragCompNorm, fragCompNorm) * 2 - 1;
	frag.normal.xy = normalize(fragCompNorm) * sqrt(1 - frag.normal.z * frag.normal.z);

	// Reconstruct pos from depth
	vec4 fragPosNdc = vec4(in_TexCoords * 2 - 1, texture(u_DepthTex, in_TexCoords).r * 2 - 1, 1);
	vec4 fragPosWorld = fragPosNdc * u_CamViewProjInv;
	fragPosWorld /= fragPosWorld.w;
	frag.pos = fragPosWorld.xyz;

	// Unpack color and gloss bits
	uvec4 packedColorGloss = texture(u_ColorGlossTex, in_TexCoords);
	vec4 unPack = unpackUnorm4x8(packedColorGloss.x);
	frag.diff.rg = unPack.xy;
	unPack = unpackUnorm4x8(packedColorGloss.y);
	frag.diff.b = unPack.x;
	frag.spec.r = unPack.y;
	unPack = unpackUnorm4x8(packedColorGloss.z);
	frag.spec.gb = unPack.xy;
	frag.gloss = unpackHalf2x16(packedColorGloss.w).x;

	// Accumulate light effects in this
	vec3 colorSum = vec3(0);

	// Add ambient effect
	colorSum += frag.diff * u_AmbientLight;

	// Add directional effects
	#if DIRLIGHT
	vec3 dirLightEffect = CalcBlinnPhong(frag, -u_DirLight.direction, u_DirLight.diffuseColor, u_DirLight.specularColor);
	#if DIRLIGHT_SHADOW
	dirLightEffect *= CalcDirShadow(frag.pos, fragPosNdc.z, frag.normal);
	#endif
	colorSum += dirLightEffect;
	#endif

	// Add spot effects
	#if NUM_SPOTLIGHTS > NUM_SPOTLIGHT_SHADOWS
	for (int i = 0; i < NUM_SPOTLIGHTS - NUM_SPOTLIGHT_SHADOWS; ++i)
	{
		colorSum += CalcSpotLightEffect(frag, u_SpotLightsNoShadow[i]);
	}
	#endif
	#if NUM_SPOTLIGHT_SHADOWS > 0
	for (int i = 0; i < NUM_SPOTLIGHT_SHADOWS; ++i)
	{
		vec3 spotLightEffect = CalcSpotLightEffect(frag, u_SpotLightsShadow[i]);
		spotLightEffect *= CalcSpotShadow(i, normalize(u_SpotLightsShadow[i].position - frag.pos), frag.pos, frag.normal);
		colorSum += spotLightEffect;
	}
	#endif

	// Add point effects
	#if NUM_POINTLIGHTS > NUM_POINTLIGHT_SHADOWS
	for (int i = 0; i < NUM_POINTLIGHTS - NUM_POINTLIGHT_SHADOWS; ++i)
	{
		colorSum += CalcPointLightEffect(frag, u_PointLightsNoShadow[i]);
	}
	#endif
	#if NUM_POINTLIGHT_SHADOWS > 0
	for (int i = 0; i < NUM_POINTLIGHT_SHADOWS; ++i)
	{
		vec3 pointLightEffect = CalcPointLightEffect(frag, u_PointLightsShadow[i]);
		pointLightEffect *= CalcPointShadow(i, frag.pos, frag.normal, u_PointLightsShadow[i].position, u_PointLightsShadow[i].range);
		colorSum += pointLightEffect;
	}
	#endif

	out_FragColor = vec4(colorSum, 1);
}