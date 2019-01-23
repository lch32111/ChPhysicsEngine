#include "LineRenderer.h"
#include "../ErrorLogger.h"

lineRenderer::lineRenderer()
{ }

lineRenderer::~lineRenderer()
{
	VertexBuffer->Release();
	Vertexlayout->Release();
	vertexShader->Release();
	vertexShaderBuffer->Release();
	pixelShader->Release();
	pixelShaderBuffer->Release();
}

bool lineRenderer::Initialize(ID3D11Device* device, const std::wstring& vPath, const std::wstring& pPath)
{
	// Vertex Buffer
	D3D11_BUFFER_DESC vBD;
	ZeroMemory(&vBD, sizeof(D3D11_BUFFER_DESC));
	vBD.Usage = D3D11_USAGE_DYNAMIC;
	vBD.ByteWidth = sizeof(DirectX::XMFLOAT3) * 2 * e_maxVertices;
	vBD.BindFlags = D3D11_BIND_VERTEX_BUFFER;
	vBD.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
	vBD.MiscFlags = 0;

	HRESULT hr = device->CreateBuffer(&vBD, NULL, &VertexBuffer);
	if (FAILED(hr))
	{
		std::wstring errorMSG = L"Failed to Cretae Vertex Buffer for line Renderer";
		ErrorLogger::Log(hr, errorMSG);
		return false;
	}
	m_vertexNumb = 0;
	stride = sizeof(DirectX::XMFLOAT3)* 2;

	// Constant Buffer
	ZeroMemory(&vBD, sizeof(D3D11_BUFFER_DESC));
	vBD.Usage = D3D11_USAGE_DYNAMIC;
	vBD.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	vBD.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
	vBD.MiscFlags = 0;
	vBD.ByteWidth = sizeof(DirectX::XMMATRIX);
	vBD.StructureByteStride = 0;
	hr = device->CreateBuffer(&vBD, 0, &ConstantBuffer);
	if (FAILED(hr))
	{
		std::wstring errorMSG = L"Failed to Cretae Constant Buffer for line Renderer";
		ErrorLogger::Log(hr, errorMSG);
		return false;
	}

	// Vertex Shader
	hr = D3DReadFileToBlob(vPath.c_str(), &vertexShaderBuffer);
	if (FAILED(hr))
	{
		std::wstring errorMsg = L"Failed to load shader: ";
		errorMsg += vPath;
		ErrorLogger::Log(hr, errorMsg);
		return false;
	}

	hr = device->CreateVertexShader(
		vertexShaderBuffer->GetBufferPointer(),
		vertexShaderBuffer->GetBufferSize(),
		NULL,
		&vertexShader);
	if (FAILED(hr))
	{
		std::wstring errorMsg = L"Failed to Create Vertex Shader: ";
		errorMsg += vPath;
		ErrorLogger::Log(hr, errorMsg);
		return false;
	}

	D3D11_INPUT_ELEMENT_DESC layout[] =
	{
		{"POSITION", 0, DXGI_FORMAT::DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{"COLOR", 0, DXGI_FORMAT::DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 }
	};
	UINT numElements = ARRAYSIZE(layout);
	hr = device->CreateInputLayout(layout, numElements,
		vertexShaderBuffer->GetBufferPointer(), vertexShaderBuffer->GetBufferSize(),
		&Vertexlayout);
	if (FAILED(hr))
	{
		ErrorLogger::Log(hr, "Failed to create input layout.");
		return false;
	}
	// Vertex Shader

	// Pixel Shader
	hr = D3DReadFileToBlob(pPath.c_str(), &pixelShaderBuffer);
	if (FAILED(hr))
	{
		std::wstring errorMsg = L"Failed to load shader: ";
		errorMsg += pPath;
		ErrorLogger::Log(hr, errorMsg);
		return false;
	}
	hr = device->CreatePixelShader(
		pixelShaderBuffer->GetBufferPointer(),
		pixelShaderBuffer->GetBufferSize(), NULL,
		&pixelShader);
	if (FAILED(hr))
	{
		std::wstring errorMsg = L"Failed to create pixel shader: ";
		errorMsg += pPath;
		ErrorLogger::Log(hr, errorMsg);
		return false;
	}
	// Pixel Shader

	return true;
}

void lineRenderer::addLine(
	const DirectX::XMFLOAT3 & from, const DirectX::XMFLOAT3 & to, const DirectX::XMFLOAT3 & c)
{
	if (m_vertexNumb + 2 >= e_maxVertices * 2) ErrorLogger::Log("Exceed the limit of vertex size");
	vertexANDcolor[m_vertexNumb] = from;
	++m_vertexNumb;
	vertexANDcolor[m_vertexNumb] = c;
	++m_vertexNumb;

	if (m_vertexNumb + 2 >= e_maxVertices * 2) ErrorLogger::Log("Exceed the limit of vertex size");
	vertexANDcolor[m_vertexNumb] = to;
	++m_vertexNumb;
	vertexANDcolor[m_vertexNumb] = c;
	++m_vertexNumb;
}

void lineRenderer::renderAndFlush(ID3D11DeviceContext* context, const DirectX::XMMATRIX& viewProj)
{
	if (m_vertexNumb < 4) return;

	// Send the data to GPU
	D3D11_MAPPED_SUBRESOURCE ms;
	context->Map(VertexBuffer, NULL, D3D11_MAP_WRITE_DISCARD, NULL, &ms);
	memcpy(ms.pData, vertexANDcolor, sizeof(DirectX::XMFLOAT3) * m_vertexNumb);
	context->Unmap(VertexBuffer, NULL);

	D3D11_MAPPED_SUBRESOURCE cms;
	context->Map(ConstantBuffer, NULL, D3D11_MAP_WRITE_DISCARD, NULL, &cms);
	memcpy(cms.pData, &viewProj, sizeof(DirectX::XMMATRIX));
	context->Unmap(ConstantBuffer, NULL);

	// context setup
	UINT offset = 0;
	context->IASetVertexBuffers(0, 1, &VertexBuffer, &stride, &offset);
	context->IASetInputLayout(Vertexlayout);
	context->VSSetShader(vertexShader, NULL, 0);
	context->PSSetShader(pixelShader, NULL, 0);
	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY::D3D10_PRIMITIVE_TOPOLOGY_LINELIST);
	context->VSSetConstantBuffers(0, 1, &ConstantBuffer);

	// Draw
	context->Draw(m_vertexNumb / 2, 0);

	m_vertexNumb = 0;
}
