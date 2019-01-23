#pragma once
#include "AdapterReader.h"
#include "Shaders.h"
#include "Vertex.h"
#include "VertexBuffer.h"
#include "IndexBuffer.h"
#include "ConstantBuffer.h"
#include  "Camera.h"
#include "../Timer.h"
#include "../Physics/PhysicsData.h"
#include "LineRenderer.h"

#include <DirectXTK/SpriteBatch.h>
#include <DirectXTK/SpriteFont.h>
#include <DirectXTK/WICTextureLoader.h>

class Graphics
{
public:
	bool Initialize(HWND hwnd, int width, int height);
	void RenderFrame(const PhysicsData& pData);

	Camera camera;
private:
	bool InitializeDirectX(HWND hwnd);
	bool InitializeShaders();
	bool InitializeScene();

	// Low Level
	Microsoft::WRL::ComPtr<ID3D11Device> device;
	Microsoft::WRL::ComPtr<ID3D11DeviceContext> deviceContext;
	Microsoft::WRL::ComPtr<IDXGISwapChain> swapchain;
	Microsoft::WRL::ComPtr<ID3D11RenderTargetView> renderTargetView;
	// Low Level

	// Depth Setup
	Microsoft::WRL::ComPtr<ID3D11DepthStencilView> depthStencilView;
	Microsoft::WRL::ComPtr<ID3D11Texture2D> depthStencilBuffer;
	Microsoft::WRL::ComPtr<ID3D11DepthStencilState> depthStencilState;
	// Depth Setup

	// Rasterizer Setup
	Microsoft::WRL::ComPtr<ID3D11RasterizerState> rasterizerState;
	// Rasterizer Setup

	// Shader
	VertexShader vertexshader;
	PixelShader pixelshader;
	// Shader

	// Vertex Buffers
	VertexBuffer<Vertex> vertexBuffer;
	IndexBuffer indicesBuffer;
	ConstantBuffer<CB_VS_vertexshader> constantbuffer;
	// Vertex Buffers
	
	// Font Setup
	std::unique_ptr<DirectX::SpriteBatch> spriteBatch;
	std::unique_ptr<DirectX::SpriteFont> spriteFont;
	// Font Setup

	// Texture Setup
	Microsoft::WRL::ComPtr<ID3D11SamplerState> samplerState;
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> myTexture;
	// Texture Setup

	int windowWidth = 0;
	int windowHeight = 0;

	Timer fpsTimer;

	lineRenderer myLineRenderer;
};