#include "Graphics.h"

bool Graphics::Initialize(HWND hwnd, int width, int height)
{
	this->windowWidth = width;
	this->windowHeight = height;
	this->fpsTimer.Start();

	if (!InitializeDirectX(hwnd))
		return false;

	if (!InitializeShaders())
		return false;

	if (!InitializeScene())
		return false;

	return true;
}

void Graphics::RenderFrame(const PhysicsData& pData)
{
	float bgColor[] = { 0.1f, 0.1f, 0.1f, 1.f };
	this->deviceContext->ClearRenderTargetView(this->renderTargetView.Get(), bgColor);
	this->deviceContext->ClearDepthStencilView(this->depthStencilView.Get(),
		D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

	// Specific setup
	this->deviceContext->IASetInputLayout(this->vertexshader.GetInputLayout());
	this->deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY::D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	this->deviceContext->RSSetState(this->rasterizerState.Get());
	this->deviceContext->OMSetDepthStencilState(this->depthStencilState.Get(), 0);
	this->deviceContext->VSSetShader(vertexshader.GetShader(), NULL, 0);
	this->deviceContext->PSSetShader(pixelshader.GetShader(), NULL, 0);
	
	UINT offset = 0;
	this->deviceContext->PSSetSamplers(0, 1, this->samplerState.GetAddressOf());
	this->deviceContext->PSSetShaderResources(0, 1, this->myTexture.GetAddressOf());
	this->deviceContext->IASetVertexBuffers(0, 1, vertexBuffer.GetAddressOf(), vertexBuffer.StridePtr(), &offset);
	this->deviceContext->IASetIndexBuffer(indicesBuffer.Get(), DXGI_FORMAT_R32_UINT, 0);

	XMMATRIX viewProj = camera.GetViewMatrix() * camera.GetProjectionMatrix();
	for (unsigned i = 0; i < pData.numActors; ++i)
	{
		constantbuffer.data.mat = pData.actors[i].modelMatrix * viewProj;
		constantbuffer.data.mat = DirectX::XMMatrixTranspose(constantbuffer.data.mat);
		constantbuffer.ApplyChanges();
		this->deviceContext->VSSetConstantBuffers(0, 1, constantbuffer.GetAddressOf());
		this->deviceContext->DrawIndexed(indicesBuffer.BufferSize(), 0, 0);
	}

	// Line
	myLineRenderer.addLine(XMFLOAT3(0, 0, 0), XMFLOAT3(0, 5, 0), XMFLOAT3(1, 1, 1));
	myLineRenderer.addLine(XMFLOAT3(0, 0, 0), XMFLOAT3(1, 4, 0), XMFLOAT3(1, 0, 0));
	myLineRenderer.renderAndFlush(this->deviceContext.Get(), DirectX::XMMatrixTranspose(viewProj));
	// Line

	// Draw Text
	static int fpsCount = 0;
	static std::string fpsString = "FPS: 0";
	fpsCount += 1;
	if (fpsTimer.GetMilisecondsElapsed() > 1000.0)
	{
		fpsString = "FPS: " + std::to_string(fpsCount);
		fpsCount = 0;
		fpsTimer.Restart();
	}
	
	spriteBatch->Begin();
	XMVECTOR result = spriteFont->MeasureString(L"temp");
	float height = XMVectorGetY(result);


	spriteFont->DrawString(spriteBatch.get(), StringConverter::StringToWide(fpsString).c_str(),
		DirectX::XMFLOAT2(0, 0));
	
	XMFLOAT3 camPos = camera.GetPositionFloat3();
	std::string temp = std::to_string(camPos.x) + ' ' + std::to_string(camPos.y) + ' ' + std::to_string(camPos.z);
	spriteFont->DrawString(spriteBatch.get(), StringConverter::StringToWide(temp).c_str(), XMFLOAT2(0, height));

	temp = "Iterations : " + std::to_string(pData.iterations);
	spriteFont->DrawString(spriteBatch.get(), StringConverter::StringToWide(temp).c_str(), XMFLOAT2(0, height * 2));

	temp = "Number of Bodies : " + std::to_string(pData.numActors);
	spriteFont->DrawString(spriteBatch.get(), StringConverter::StringToWide(temp).c_str(), XMFLOAT2(0, height* 3));

	temp = "WarmStart(M) : " + std::to_string(pData.useWarmStart);
	spriteFont->DrawString(spriteBatch.get(), StringConverter::StringToWide(temp).c_str(), XMFLOAT2(0, height * 4));

	temp = "AccumImpulse(N) : " + std::to_string(pData.accumulateImpulse);
	spriteFont->DrawString(spriteBatch.get(), StringConverter::StringToWide(temp).c_str(), XMFLOAT2(0, height * 5));

	temp = "PosCorrection(B) : " + std::to_string(pData.positionCorrection);
	spriteFont->DrawString(spriteBatch.get(), StringConverter::StringToWide(temp).c_str(), XMFLOAT2(0, height * 6));

	spriteBatch->End();

	// Vsync for Physics
	this->swapchain->Present(1, NULL);
}

bool Graphics::InitializeDirectX(HWND hwnd)
{
	std::vector<AdapterData> adapters = AdapterReader::GetAdapters();

	if (adapters.size() < 1)
	{
		ErrorLogger::Log("No IDXGI Adapters found.");
		return false;
	}

	DXGI_SWAP_CHAIN_DESC scd;
	ZeroMemory(&scd, sizeof(DXGI_SWAP_CHAIN_DESC));
	scd.BufferDesc.Width = this->windowWidth;
	scd.BufferDesc.Height = this->windowHeight;
	scd.BufferDesc.RefreshRate.Numerator = 60;
	scd.BufferDesc.RefreshRate.Denominator = 1;
	scd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	scd.BufferDesc.ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED;
	scd.BufferDesc.Scaling = DXGI_MODE_SCALING_UNSPECIFIED;
	
	scd.SampleDesc.Count = 1;
	scd.SampleDesc.Quality = 0;

	scd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	scd.BufferCount = 1;
	scd.OutputWindow = hwnd;
	scd.Windowed = TRUE;
	scd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
	scd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;

	HRESULT hr;
	hr = D3D11CreateDeviceAndSwapChain
	(
		adapters[0].pAdatpter,		//IDXGI Adapter
		D3D_DRIVER_TYPE_UNKNOWN,
		NULL,						// for software driver type
		NULL,						// flags for runtime layers
		NULL,						// feature levesl array
		0,							// # of feature levels in array
		D3D11_SDK_VERSION,
		&scd,						// Swapcahin description
		this->swapchain.GetAddressOf(), 
		this->device.GetAddressOf(),
		NULL,						// Supported feature level
		this->deviceContext.GetAddressOf()
	);

	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "Failed to create device and swapchain.");
		return false;
	}

	Microsoft::WRL::ComPtr<ID3D11Texture2D> backBuffer;
	hr = this->swapchain->GetBuffer(0, __uuidof(ID3D11Texture2D), reinterpret_cast<void**>(backBuffer.GetAddressOf()));
	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "GetBuffer Failed.");
		return false;
	}

	hr = this->device->CreateRenderTargetView
	(
		backBuffer.Get(),
		NULL,
		this->renderTargetView.GetAddressOf()
	);
	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "Failed to create render target view.");
		return false;
	}

	// Describe our Depth/Stencil Buffer
	D3D11_TEXTURE2D_DESC depthStencilDesc;
	depthStencilDesc.Width = this->windowWidth;
	depthStencilDesc.Height = this->windowHeight;
	depthStencilDesc.MipLevels = 1;
	depthStencilDesc.ArraySize = 1;
	depthStencilDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
	depthStencilDesc.SampleDesc.Count = 1;
	depthStencilDesc.SampleDesc.Quality = 0;
	depthStencilDesc.Usage = D3D11_USAGE_DEFAULT;
	depthStencilDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
	depthStencilDesc.CPUAccessFlags = 0;
	depthStencilDesc.MiscFlags = 0;

	hr = this->device->CreateTexture2D(&depthStencilDesc, NULL, this->depthStencilBuffer.GetAddressOf());
	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "Failed to create depth stencil buffer.");
		return false;
	}

	hr = this->device->CreateDepthStencilView(this->depthStencilBuffer.Get(), NULL, this->depthStencilView.GetAddressOf());
	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "Failed to create depth stencil view.");
		return false;
	}

	this->deviceContext->OMSetRenderTargets(1, this->renderTargetView.GetAddressOf(), this->depthStencilView.Get());

	// Create depth stencil state
	D3D11_DEPTH_STENCIL_DESC depthstencildesc;
	ZeroMemory(&depthstencildesc, sizeof(D3D11_DEPTH_STENCIL_DESC));

	depthstencildesc.DepthEnable = true;
	depthstencildesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK::D3D11_DEPTH_WRITE_MASK_ALL;
	depthstencildesc.DepthFunc = D3D11_COMPARISON_FUNC::D3D11_COMPARISON_LESS_EQUAL;

	hr = this->device->CreateDepthStencilState(&depthstencildesc, this->depthStencilState.GetAddressOf());
	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "Failed to create depth stencil state.");
		return false;
	}

	// Create the Viewport
	D3D11_VIEWPORT viewport;
	ZeroMemory(&viewport, sizeof(D3D11_VIEWPORT));

	viewport.TopLeftX = 0;
	viewport.TopLeftY = 0;
	viewport.Width = (FLOAT)this->windowWidth;
	viewport.Height = (FLOAT)this->windowHeight;
	viewport.MinDepth = 0.0f;
	viewport.MaxDepth = 1.0f;

	this->deviceContext->RSSetViewports(1, &viewport);

	// Create Rasterizer State
	D3D11_RASTERIZER_DESC rasterizerDesc;
	ZeroMemory(&rasterizerDesc, sizeof(D3D11_RASTERIZER_DESC));

	rasterizerDesc.FillMode = D3D11_FILL_MODE::D3D11_FILL_SOLID;
	rasterizerDesc.CullMode = D3D11_CULL_MODE::D3D11_CULL_BACK;
	hr = this->device->CreateRasterizerState(&rasterizerDesc, rasterizerState.GetAddressOf());
	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "Failed to create rasterizer state.");
		return false;
	}

	spriteBatch = std::make_unique<DirectX::SpriteBatch>(this->deviceContext.Get());
	spriteFont = std::make_unique<DirectX::SpriteFont>(this->device.Get(), L"Data/Fonts/comic_sans_ms.spritefont");

	// Create sampler description for sampler state
	D3D11_SAMPLER_DESC sampDesc;
	ZeroMemory(&sampDesc, sizeof(D3D11_SAMPLER_DESC));
	sampDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
	sampDesc.AddressU = D3D11_TEXTURE_ADDRESS_WRAP;
	sampDesc.AddressV = D3D11_TEXTURE_ADDRESS_WRAP;
	sampDesc.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
	sampDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
	sampDesc.MinLOD = 0;
	sampDesc.MaxLOD = D3D11_FLOAT32_MAX;
	hr = this->device->CreateSamplerState(&sampDesc, this->samplerState.GetAddressOf());
	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "Failed to create sampler state.");
		return false;
	}


	return true;
}

bool Graphics::InitializeShaders()
{
	std::wstring shaderFolder = L"";
#pragma region DetermineShaderPath
	if (IsDebuggerPresent() == TRUE)
	{
#ifdef  _DEBUG // Debug Mode
	#ifdef _WIN64
		shaderFolder = L"../x64/Debug";
	#else // _WIN64
		shaderFolder = L"../Debug/";
	#endif
#else // Release Mode
	#ifdef _WIN64
		shaderFolder = L"../x64/Release";
	#else // _WIN64
		shaderFolder = L"../Release/";
	#endif
#endif
	}

	D3D11_INPUT_ELEMENT_DESC layout[] =
	{
		{"POSITION", 0, DXGI_FORMAT::DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA },
		{"TEXCOORD", 0, DXGI_FORMAT::DXGI_FORMAT_R32G32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA }
	};

	UINT numElements = ARRAYSIZE(layout);

	if (!vertexshader.Initialize(this->device, shaderFolder + L"vertexshader.cso",
		layout, numElements))
		return false;

	if (!pixelshader.Initialize(this->device, shaderFolder + L"pixelshader.cso"))
		return false;

	if (!myLineRenderer.Initialize(device.Get(), shaderFolder + L"lineRendererVertexShader.cso",
		shaderFolder + L"lineRendererPixelShader.cso"))
		return false;
	 
	return true;
}

bool Graphics::InitializeScene()
{
	Vertex v[] = 
	{
		Vertex(-0.5f, -0.5f, 0.f, 0.0f, 1.0f), // Bottom Left
		Vertex(-0.5f, 0.5f, 0.f, 0.f, 0.f), // Top Left
		Vertex(0.5f, 0.5f, 0.f, 1.0f, 0.0f), //  Top Right
		Vertex(0.5f, -0.5f, 0.f, 1.0f, 1.0f), //  Bottom Right
	};

	DWORD indices[] =
	{
		0, 1, 2,
		0, 2, 3,
	};

	// Load Vertex Data
	HRESULT hr = vertexBuffer.Initialize(this->device.Get(), v, ARRAYSIZE(v));
	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "Failed to create vertex buffer.");
		return false;
	}

	// Load Index Data
	hr = indicesBuffer.Initialize(this->device.Get(), indices, ARRAYSIZE(indices));
	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "Failed to create indices buffer.");
		return hr;
	}

	hr = constantbuffer.Initialize(this->device.Get(), this->deviceContext.Get());
	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "Failed to create constant buffer.");
		return hr;
	}

	// Load Texture
	hr = DirectX::CreateWICTextureFromFile(this->device.Get(), L"Data/Textures/fieldGrass.jpg", 
		nullptr, myTexture.GetAddressOf());
	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "Failed to create wic texture from file.");
		return false;
	}

	camera.SetPosition(0.f, 8.f, 0.f);
	camera.SetProjectionValues(90.f, static_cast<float>(windowWidth) / static_cast<float>(windowHeight), 0.1f, 1000.f);
	float aspect = (float)windowWidth / (float)windowHeight;
	float zoom = 10.f;
	float pan_y = 8.0;
	// camera.SetOrthoValue(zoom * aspect * 2, zoom * 2, -1, 1);
	return true;
}
