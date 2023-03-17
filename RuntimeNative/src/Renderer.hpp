#pragma once

#include "Core.hpp"
#include "StaticMeshComponent.hpp"
#include "LightComponents.hpp"
#include "Util.hpp"
#include "Platform.hpp"
#include "SkyboxComponent.hpp"
#include "RenderCamera.hpp"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <d3d11.h>
#include <dxgi1_6.h>
#include <wrl/client.h>

#include <array>
#include <optional>


namespace leopph {
class Renderer {
	struct ShadowAtlas {
		Microsoft::WRL::ComPtr<ID3D11Texture2D> tex;
		Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> srv;
		Microsoft::WRL::ComPtr<ID3D11DepthStencilView> dsv;
		int width;
		int height;
	};

	struct Resources {
		Microsoft::WRL::ComPtr<ID3D11Device> device;
		Microsoft::WRL::ComPtr<ID3D11DeviceContext> context;

		Microsoft::WRL::ComPtr<IDXGISwapChain1> swapChain;
		Microsoft::WRL::ComPtr<ID3D11Texture2D> gameHdrTexture;
		Microsoft::WRL::ComPtr<ID3D11Texture2D> gameOutputTexture;
		Microsoft::WRL::ComPtr<ID3D11Texture2D> gameDSTex;
		Microsoft::WRL::ComPtr<ID3D11Texture2D> sceneHdrTexture;
		Microsoft::WRL::ComPtr<ID3D11Texture2D> sceneOutputTexture;
		Microsoft::WRL::ComPtr<ID3D11Texture2D> sceneDSTex;

		Microsoft::WRL::ComPtr<ID3D11RenderTargetView> swapChainRtv;
		Microsoft::WRL::ComPtr<ID3D11RenderTargetView> gameHdrTextureRtv;
		Microsoft::WRL::ComPtr<ID3D11RenderTargetView> gameOutputTextureRtv;
		Microsoft::WRL::ComPtr<ID3D11RenderTargetView> sceneHdrTextureRtv;
		Microsoft::WRL::ComPtr<ID3D11RenderTargetView> sceneOutputTextureRtv;

		Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> gameHdrTextureSrv;
		Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> gameOutputTextureSrv;
		Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> sceneHdrTextureSrv;
		Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> sceneOutputTextureSrv;

		Microsoft::WRL::ComPtr<ID3D11DepthStencilView> gameDSV;
		Microsoft::WRL::ComPtr<ID3D11DepthStencilView> sceneDSV;

		Microsoft::WRL::ComPtr<ID3D11PixelShader> clearColorPS;
		Microsoft::WRL::ComPtr<ID3D11VertexShader> quadVS;
		Microsoft::WRL::ComPtr<ID3D11PixelShader> meshBlinnPhongPS;
		Microsoft::WRL::ComPtr<ID3D11VertexShader> meshVS;
		Microsoft::WRL::ComPtr<ID3D11PixelShader> meshPbrPS;
		Microsoft::WRL::ComPtr<ID3D11VertexShader> texQuadVS;
		Microsoft::WRL::ComPtr<ID3D11PixelShader> toneMapGammaPS;
		Microsoft::WRL::ComPtr<ID3D11PixelShader> skyboxPS;
		Microsoft::WRL::ComPtr<ID3D11VertexShader> skyboxVS;

		Microsoft::WRL::ComPtr<ID3D11Buffer> perFrameCB;
		Microsoft::WRL::ComPtr<ID3D11Buffer> perCamCB;
		Microsoft::WRL::ComPtr<ID3D11Buffer> perModelCB;
		Microsoft::WRL::ComPtr<ID3D11Buffer> clearColorCB;
		Microsoft::WRL::ComPtr<ID3D11Buffer> toneMapGammaCB;
		Microsoft::WRL::ComPtr<ID3D11Buffer> skyboxCB;

		Microsoft::WRL::ComPtr<ID3D11InputLayout> meshIL;
		Microsoft::WRL::ComPtr<ID3D11InputLayout> quadIL;
		Microsoft::WRL::ComPtr<ID3D11InputLayout> texQuadIL;
		Microsoft::WRL::ComPtr<ID3D11InputLayout> skyboxIL;

		Microsoft::WRL::ComPtr<ID3D11Buffer> quadPosVB;
		Microsoft::WRL::ComPtr<ID3D11Buffer> quadUvVB;
		Microsoft::WRL::ComPtr<ID3D11Buffer> quadIB;

		Microsoft::WRL::ComPtr<ID3D11SamplerState> hdrTextureSS;
		Microsoft::WRL::ComPtr<ID3D11SamplerState> materialSS;
		Microsoft::WRL::ComPtr<ID3D11SamplerState> shadowSS;

		Microsoft::WRL::ComPtr<ID3D11RasterizerState> skyboxPassRS;

		Microsoft::WRL::ComPtr<ID3D11DepthStencilState> skyboxPassDSS;

		std::shared_ptr<Material> defaultMaterial;
		std::shared_ptr<Mesh> cubeMesh;

		ShadowAtlas spotPointShadowAtlas;
	};

	struct ShadowGenerationData {
		Matrix4 lightViewProj;
		int lightIdx;
	};

#pragma warning(push)
#pragma warning(disable: 4324)
	struct ShadowAtlasAllocation {
		std::optional<ShadowGenerationData> q1Light;
		std::array<std::optional<ShadowGenerationData>, 4> q2Lights;
		std::array<std::optional<ShadowGenerationData>, 16> q3Lights;
		std::array<std::optional<ShadowGenerationData>, 64> q4Lights;
	};
#pragma warning(pop)

	constexpr static int SPOT_POINT_SHADOW_ATLAS_SIZE{ 4096 };

	auto RecreateGameTexturesAndViews(u32 width, u32 height) const -> void;
	auto RecreateSceneTexturesAndViews(u32 width, u32 height) const -> void;
	static auto on_window_resize(Renderer* self, Extent2D<u32> size) -> void;

	auto CreateDeviceAndContext() const -> void;
	auto SetDebugBreaks() const -> void;
	auto CheckTearingSupport(IDXGIFactory2* factory2) -> void;
	auto CreateInputLayouts() const -> void;
	auto CreateShaders() const -> void;
	auto CreateSwapChain(IDXGIFactory2* factory2) const -> void;
	auto RecreateSwapChainRtv() const -> void;
	auto CreateVertexAndIndexBuffers() const -> void;
	auto CreateConstantBuffers() const -> void;
	auto CreateRasterizerStates() const -> void;
	auto CreateDepthStencilStates() const -> void;
	auto CreateShadowAtlases() const -> void;
	auto CreateSamplerStates() const -> void;
	auto DrawMeshes() const noexcept -> void;
	auto UpdatePerFrameCB() const noexcept -> void;
	auto DoToneMapGammaCorrectionStep(ID3D11ShaderResourceView* src, ID3D11RenderTargetView* dst) const noexcept -> void;
	auto DrawSkybox(Matrix4 const& camViewMtx, Matrix4 const& camProjMtx) const noexcept -> void;
	auto DrawFullWithCameras(std::span<RenderCamera const* const> cameras, ID3D11RenderTargetView* rtv, ID3D11DepthStencilView* dsv, ID3D11ShaderResourceView* srv, ID3D11RenderTargetView* outRtv) const noexcept -> void;
	auto DrawShadowMaps(Matrix4 const& camViewProj) const -> void;

	Resources* mResources{ nullptr };
	UINT mPresentFlags{ 0 };
	UINT mSwapChainFlags{ 0 };
	Extent2D<u32> mGameRes;
	f32 mGameAspect;
	Extent2D<u32> mSceneRes;
	f32 mSceneAspect;
	u32 mSyncInterval{ 0 };
	std::vector<StaticMeshComponent const*> mStaticMeshComponents;
	std::vector<LightComponent const*> mLights;
	f32 mInvGamma{ 1.f / 2.2f };
	std::vector<SkyboxComponent const*> mSkyboxes;
	std::vector<RenderCamera const*> mGameRenderCameras;

	ShadowAtlasAllocation mSpotPointShadowAtlasAlloc;

public:
	Renderer() noexcept = default;
	~Renderer() noexcept = default;

	LEOPPHAPI auto StartUp() -> void;
	LEOPPHAPI auto ShutDown() noexcept -> void;

	LEOPPHAPI auto DrawGame() const noexcept -> void;
	LEOPPHAPI auto DrawSceneView(RenderCamera const& cam) const noexcept -> void;

	[[nodiscard]] LEOPPHAPI auto GetGameResolution() const noexcept -> Extent2D<u32>;
	LEOPPHAPI auto SetGameResolution(Extent2D<u32> resolution) noexcept -> void;

	LEOPPHAPI [[nodiscard]] auto GetSceneResolution() const noexcept -> Extent2D<u32>;
	LEOPPHAPI auto SetSceneResolution(Extent2D<u32> resolution) noexcept -> void;

	[[nodiscard]] LEOPPHAPI auto GetGameFrame() const noexcept -> ID3D11ShaderResourceView*;
	[[nodiscard]] LEOPPHAPI auto GetSceneFrame() const noexcept -> ID3D11ShaderResourceView*;

	[[nodiscard]] LEOPPHAPI auto GetGameAspectRatio() const noexcept -> f32;
	[[nodiscard]] LEOPPHAPI auto GetSceneAspectRatio() const noexcept -> f32;

	LEOPPHAPI auto BindAndClearSwapChain() const noexcept -> void;

	LEOPPHAPI auto Present() const noexcept -> void;

	[[nodiscard]] LEOPPHAPI auto GetSyncInterval() const noexcept -> u32;
	LEOPPHAPI auto SetSyncInterval(u32 interval) noexcept -> void;

	LEOPPHAPI auto RegisterStaticMesh(StaticMeshComponent const* staticMesh) -> void;
	LEOPPHAPI auto UnregisterStaticMesh(StaticMeshComponent const* staticMesh) -> void;

	[[nodiscard]] LEOPPHAPI auto GetDevice() const noexcept -> ID3D11Device*;
	[[nodiscard]] LEOPPHAPI auto GetImmediateContext() const noexcept -> ID3D11DeviceContext*;

	LEOPPHAPI auto RegisterLight(LightComponent const* light) -> void;
	LEOPPHAPI auto UnregisterLight(LightComponent const* light) -> void;

	[[nodiscard]] LEOPPHAPI auto GetDefaultMaterial() const noexcept -> std::shared_ptr<Material>;
	[[nodiscard]] LEOPPHAPI auto GetCubeMesh() const noexcept -> std::shared_ptr<Mesh>;

	[[nodiscard]] LEOPPHAPI auto GetGamma() const noexcept -> f32;
	LEOPPHAPI auto SetGamma(f32 gamma) noexcept -> void;

	auto LEOPPHAPI RegisterSkybox(SkyboxComponent const* skybox) -> void;
	auto LEOPPHAPI UnregisterSkybox(SkyboxComponent const* skybox) -> void;

	auto LEOPPHAPI RegisterGameCamera(RenderCamera const& cam) -> void;
	auto LEOPPHAPI UnregisterGameCamera(RenderCamera const& cam) -> void;
};
}
