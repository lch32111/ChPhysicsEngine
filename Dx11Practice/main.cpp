#include <windows.h>    // include the basic windows header file
#include <windowsx.h>
#include <d3d11.h>
#include <DirectXTK/SimpleMath.h>
#include <d3dcompiler.h>

// the WindowProc function prototype
LRESULT CALLBACK WindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

// define the screen resolution
#define SCREEN_WIDTH  800
#define SCREEN_HEIGHT 600

// global declarations
IDXGISwapChain *swapchain;             // the pointer to the swap chain interface
ID3D11Device *dev;                     // the pointer to our Direct3D device interface
ID3D11DeviceContext *devcon;           // the pointer to our Direct3D device context
ID3D11RenderTargetView *backbuffer;    // the pointer to our back buffer
ID3D11InputLayout *pLayout;            // the pointer to the input layout
ID3D11VertexShader *pVS;               // the pointer to the vertex shader
ID3D11PixelShader *pPS;                // the pointer to the pixel shader
ID3D11Buffer *pVBuffer;                // the pointer to the vertex buffer

// a struct to define a single vertex
#define D3DX DirectX::SimpleMath
struct VERTEX { FLOAT X, Y, Z;  D3DX::Color Color; };

// function prototypes
void InitD3D(HWND hWnd);
void RenderFrame(void);     // renders a single frame
void CleanD3D();
void InitGraphics(void);    // creates the shape to render
void InitPipeline(void);    // loads and prepares the shaders
HRESULT CompileShader(_In_ LPCWSTR srcFile, _In_ LPCSTR entryPoint, _In_ LPCSTR profile, _Outptr_ ID3DBlob** blob);

// the entry point for any Windows program
int WINAPI WinMain(HINSTANCE hInstance,	HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	// the handle for the window, filled by a function
	HWND hWnd;
	// this struct holds information for the window class
	WNDCLASSEX wc;

	// clear out the window class for use
	ZeroMemory(&wc, sizeof(WNDCLASSEX));

	// fill in the struct with the needed information
	wc.cbSize = sizeof(WNDCLASSEX);
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.lpfnWndProc = WindowProc;
	wc.hInstance = hInstance;
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	// wc.hbrBackground = (HBRUSH)COLOR_WINDOW;
	wc.lpszClassName = "WindowClass";

	// register the window class
	RegisterClassEx(&wc);

	RECT wr = { 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT }; // set the size but not the position
	AdjustWindowRect(&wr, WS_OVERLAPPEDWINDOW, FALSE);

	// create the window and use the result as the handle
	hWnd = CreateWindowEx(
		NULL,
		"WindowClass",		// name of the widnow class
		"Our First Direct3D Program",		// title of the widnow
		WS_OVERLAPPEDWINDOW,		// window style
		300,		// x-position of the window
		300,		// y-position of the window
		wr.right - wr.left, // width of the window
		wr.bottom - wr.top, // height of the window
		NULL,		// we have no parent window, NULL
		NULL,		// we aren't using menus, NULL
		hInstance,	// application handle
		NULL);		// used with multiple widnows, NULL

	// display the widnow on the screen
	ShowWindow(hWnd, nCmdShow);

	// set up and initialize Direct3D
	InitD3D(hWnd);

	// enther the main loop:
	// this struct holds Windows event messages
	MSG msg = { 0 };

	// wait for the next message in the queue, store the reulst in 'msg'
	while (TRUE)
	{
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			// translate keystroke messages into the right format
			TranslateMessage(&msg);

			// send the message to the WindowProc function
			DispatchMessage(&msg);

			// check to see if it's time to quit
			if (msg.message == WM_QUIT)
				break;
		}

		// Run Game Code here
		RenderFrame();
	}

	CleanD3D();
	
	return msg.wParam;
}

LRESULT CALLBACK WindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	// sort through and find what cod to run for the message give
	switch (message)
	{
		// this message is read when the window is closed
	case WM_DESTROY:
	{
		// close the application entirely
		PostQuitMessage(0);
		return 9;
	}
	break;
	}

	// Handle any messages the switch statement didn't
	return DefWindowProc(hWnd, message, wParam, lParam);
}

void InitD3D(HWND hWnd)
{
	// create a struct to hold information about the swap chain
	DXGI_SWAP_CHAIN_DESC scd;

	// clear out the struct for use
	ZeroMemory(&scd, sizeof(DXGI_SWAP_CHAIN_DESC));

	// fill the swap chain description struct
	scd.BufferCount = 1;                                    // one back buffer
	scd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;     // use 32-bit color
	scd.BufferDesc.Width = SCREEN_WIDTH;                    // set the back buffer width
	scd.BufferDesc.Height = SCREEN_HEIGHT;                  // set the back buffer height
	scd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;      // how swap chain is to be used
	scd.OutputWindow = hWnd;                                // the window to be used
	scd.SampleDesc.Count = 4;                               // how many multisamples
	scd.Windowed = TRUE;                                    // windowed/full-screen mode
	scd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;     // allow full-screen switching

	// create a device, device context and swap chain using the information in the scd struct
	D3D11CreateDeviceAndSwapChain(NULL,
		D3D_DRIVER_TYPE_HARDWARE,
		NULL,
		NULL,
		NULL,
		NULL,
		D3D11_SDK_VERSION,
		&scd,
		&swapchain,
		&dev,
		NULL,
		&devcon);

	// get the address of the back buffer
	ID3D11Texture2D *pBackBuffer;
	swapchain->GetBuffer(0, __uuidof(ID3D11Texture2D), (LPVOID*)&pBackBuffer);

	// use the back buffer address to create the render target
	dev->CreateRenderTargetView(pBackBuffer, NULL, &backbuffer);
	pBackBuffer->Release();

	// set the render target as the back buffer
	devcon->OMSetRenderTargets(1, &backbuffer, NULL);


	// Set the viewport
	D3D11_VIEWPORT viewport;
	ZeroMemory(&viewport, sizeof(D3D11_VIEWPORT));

	viewport.TopLeftX = 0;
	viewport.TopLeftY = 0;
	viewport.Width = SCREEN_WIDTH;
	viewport.Height = SCREEN_HEIGHT;

	devcon->RSSetViewports(1, &viewport);

	InitPipeline();
	InitGraphics();
}

// this is the function used to render a single frame
void RenderFrame(void)
{
	
	// clear the back buffer to a deep blue
	devcon->ClearRenderTargetView(backbuffer, DirectX::SimpleMath::Color(0.0f, 0.2f, 0.4f, 1.0f));

	// do 3D rendering on the back buffer here
	// select which vertex buffer to display
	UINT stride = sizeof(VERTEX);
	UINT offset = 0;
	devcon->IASetVertexBuffers(0, 1, &pVBuffer, &stride, &offset);

	// select which primtive type we are using
	devcon->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	// draw the vertex buffer to the back buffer
	devcon->Draw(3, 0);

	// switch the back buffer and the front buffer
	swapchain->Present(0, 0);
}


void CleanD3D()
{
	swapchain->SetFullscreenState(FALSE, NULL);    // switch to windowed mode

	// close and release all existing COM objects
	pLayout->Release();
	pVS->Release();
	pPS->Release();
	pVBuffer->Release();
	swapchain->Release();
	backbuffer->Release();
	dev->Release();
	devcon->Release();
}

// this is the function that creates the shape to render
void InitGraphics()
{
	// create a triangle using the VERTEX struct
	VERTEX OurVertices[] =
	{
		{0.0f, 0.5f, 0.0f, D3DX::Color(1.0f, 0.0f, 0.0f, 1.0f)},
		{0.45f, -0.5, 0.0f, D3DX::Color(0.0f, 1.0f, 0.0f, 1.0f)},
		{-0.45f, -0.5f, 0.0f, D3DX::Color(0.0f, 0.0f, 1.0f, 1.0f)}
	};


	// create the vertex buffer
	D3D11_BUFFER_DESC bd;
	ZeroMemory(&bd, sizeof(bd));

	bd.Usage = D3D11_USAGE_DYNAMIC;                // write access access by CPU and GPU
	bd.ByteWidth = sizeof(VERTEX) * 3;             // size is the VERTEX struct * 3
	bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;       // use as a vertex buffer
	bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;    // allow CPU to write in buffer

	dev->CreateBuffer(&bd, NULL, &pVBuffer);       // create the buffer


	// copy the vertices into the buffer
	D3D11_MAPPED_SUBRESOURCE ms;
	devcon->Map(pVBuffer, NULL, D3D11_MAP_WRITE_DISCARD, NULL, &ms);    // map the buffer
	memcpy(ms.pData, OurVertices, sizeof(OurVertices));                 // copy the data
	devcon->Unmap(pVBuffer, NULL);                                      // unmap the buffer
}


// this function loads and prepares the shaders
void InitPipeline()
{
	// load and compile the two shaders
	ID3D10Blob *VS, *PS;
	HRESULT hr = CompileShader(L"shaders.shader", "VShader", "vs_4_0", &VS);
	if (FAILED(hr))
	{
		printf("Failed compiling vertex shader %08X\n", hr);
		return;
	}

	hr = CompileShader(L"shaders.shader", "PShader", "ps_4_0", &PS);
	if (FAILED(hr))
	{
		VS->Release();
		printf("Failed compiling pixel shader %08X\n", hr);
		return;
	}

	// encapsulate both shaders into shader objects
	dev->CreateVertexShader(VS->GetBufferPointer(), VS->GetBufferSize(), NULL, &pVS);
	dev->CreatePixelShader(PS->GetBufferPointer(), PS->GetBufferSize(), NULL, &pPS);

	// set the shader objects
	devcon->VSSetShader(pVS, 0, 0);
	devcon->PSSetShader(pPS, 0, 0);

	// create the input layout object
	D3D11_INPUT_ELEMENT_DESC ied[] =
	{
		{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
		{"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0},
	};

	dev->CreateInputLayout(ied, 2, VS->GetBufferPointer(), VS->GetBufferSize(), &pLayout);
	devcon->IASetInputLayout(pLayout);
}

HRESULT CompileShader(_In_ LPCWSTR srcFile, _In_ LPCSTR entryPoint, _In_ LPCSTR profile, _Outptr_ ID3DBlob** blob)
{
	if (!srcFile || !entryPoint || !profile || !blob)
		return E_INVALIDARG;

	*blob = nullptr;

	UINT flags = D3DCOMPILE_ENABLE_STRICTNESS;
#if defined( DEBUG ) || defined( _DEBUG )
	flags |= D3DCOMPILE_DEBUG;
#endif

	const D3D_SHADER_MACRO defines[] =
	{
		"EXAMPLE_DEFINE", "1",
		NULL, NULL
	};

	ID3DBlob* shaderBlob = nullptr;
	ID3DBlob* errorBlob = nullptr;
	HRESULT hr = D3DCompileFromFile(srcFile, defines, D3D_COMPILE_STANDARD_FILE_INCLUDE,
		entryPoint, profile,
		flags, 0, &shaderBlob, &errorBlob);
	if (FAILED(hr))
	{
		if (errorBlob)
		{
			OutputDebugStringA((char*)errorBlob->GetBufferPointer());
			errorBlob->Release();
		}

		if (shaderBlob)
			shaderBlob->Release();

		return hr;
	}

	*blob = shaderBlob;

	return hr;
}
