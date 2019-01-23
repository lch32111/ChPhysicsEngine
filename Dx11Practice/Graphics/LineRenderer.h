#pragma once
#include <DirectXMath.h>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <string>

class lineRenderer
{
public:
	lineRenderer();
	~lineRenderer();

	bool Initialize(ID3D11Device* device, const std::wstring& vPath, const std::wstring& pPath);

	void addLine(const DirectX::XMFLOAT3& from, const DirectX::XMFLOAT3& to, 
		const DirectX::XMFLOAT3& color);
	void renderAndFlush(ID3D11DeviceContext* context, const DirectX::XMMATRIX& viewProj);
private:
	enum {e_maxVertices = 1024};
	DirectX::XMFLOAT3 vertexANDcolor[e_maxVertices * 2];
	unsigned m_vertexNumb;
	UINT stride;

	// Graphics
	ID3D11Buffer* VertexBuffer;
	ID3D11InputLayout* Vertexlayout;

	ID3D11VertexShader* vertexShader;
	ID3D10Blob* vertexShaderBuffer;

	ID3D11PixelShader* pixelShader;
	ID3D10Blob* pixelShaderBuffer;

	ID3D11Buffer* ConstantBuffer;
};