#version 460 core

#define MAX_DIR_LIGHT_CASCADE_COUNT 3
#define MIN_SHADOW_BIAS 0.0001
#define MAX_SHADOW_BIAS 0.0005


struct DirLight
{
	vec3 direction;
	
	vec3 diffuseColor;
	vec3 specularColor;
};


layout (location = 0) in vec2 in_TexCoords;

layout (location = 0) out vec4 out_FragmentColor;

layout (location = 5) uniform sampler2D u_PositionTexture;
layout (location = 6) uniform sampler2D u_NormalTexture;
layout (location = 7) uniform sampler2D u_DiffuseTexture;
layout (location = 8) uniform sampler2D u_SpecularTexture;
layout (location = 9) uniform sampler2D u_ShineTexture;
layout (location = 1) uniform DirLight u_DirLight;
layout (location = 0) uniform vec3 u_CameraPosition;
layout (location = 4) uniform uint u_CascadeCount;
layout (location = 10) uniform sampler2DShadow u_ShadowMaps[MAX_DIR_LIGHT_CASCADE_COUNT];
layout (location = 10 + 8 * MAX_DIR_LIGHT_CASCADE_COUNT) uniform float u_CascadeFarBounds[MAX_DIR_LIGHT_CASCADE_COUNT];
layout (location = 10 + MAX_DIR_LIGHT_CASCADE_COUNT * 4) uniform mat4 u_LightClipMatrices[MAX_DIR_LIGHT_CASCADE_COUNT];



vec3 CalculateBlinnPhong(vec3 fragPos, vec3 fragNormal, vec3 fragDiffuse, vec3 fragSpecular, float fragShine)
{
	vec3 directionToLight = -u_DirLight.direction;

	float diffuseDot = max(dot(directionToLight, fragNormal), 0);
	vec3 light = fragDiffuse * diffuseDot * u_DirLight.diffuseColor;

	if (diffuseDot > 0)
	{
		vec3 halfway = normalize(directionToLight + normalize(u_CameraPosition - fragPos));
		light += fragSpecular * pow(max(dot(fragNormal, halfway), 0), 4 * fragShine) * u_DirLight.specularColor;
	}

	return light;
}



float CalculateShadow(vec3 fragPos, float fragPosCameraClipZ, vec3 fragNormal)
{
	int cascadeIndex = -1;
	for (int i = 0; i < u_CascadeCount; ++i)
	{
		if (fragPosCameraClipZ < u_CascadeFarBounds[i])
		{
			cascadeIndex = i;
			break;
		}
	}

	vec4 fragPosLightSpace = vec4(fragPos, 1) * u_LightClipMatrices[cascadeIndex];
	vec3 normalizedPos = fragPosLightSpace.xyz;
	normalizedPos *= 0.5;
	normalizedPos += 0.5;

	vec2 texelSize = 1.0 / textureSize(u_ShadowMaps[cascadeIndex], 0);
	float bias = max(MAX_SHADOW_BIAS * (1.0 - dot(fragNormal, -u_DirLight.direction)), MIN_SHADOW_BIAS);
	float shadow = 0;

	for (int i = -1; i <= 1; i++)
	{
		for (int j = -1; j <= 1; j++)
		{
			shadow += texture(u_ShadowMaps[cascadeIndex], vec3(normalizedPos.xy + vec2(i, j) * texelSize, normalizedPos.z - bias));
		}
	}

	return shadow / 9;
}



void main()
{
    vec4 fragPos = texture(u_PositionTexture, in_TexCoords);
    vec3 fragNormal = texture(u_NormalTexture, in_TexCoords).rgb;
    vec3 fragDiffuse = texture(u_DiffuseTexture, in_TexCoords).rgb;
    vec3 fragSpecular = texture(u_SpecularTexture, in_TexCoords).rgb;
	float fragShine = texture(u_ShineTexture, in_TexCoords).r;

	vec3 light = CalculateBlinnPhong(fragPos.xyz, fragNormal, fragDiffuse, fragSpecular, fragShine);
	light *= CalculateShadow(fragPos.xyz, fragPos.w, fragNormal);

	out_FragmentColor = vec4(light, 1);
}