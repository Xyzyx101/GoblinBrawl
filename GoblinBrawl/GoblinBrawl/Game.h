#pragma once
#include <d3dx11.h>

#define MAX_LOADSTRING 100

class Game {
public:
	Game( HINSTANCE hInstance, int clientWidth, int clientHeight );
	~Game();
	bool Init();
	void OnResize();
	LRESULT MsgProc( HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam );
	int Run();
private:
	bool InitMainWindow();
	bool InitDirect3D();
	void DisplayWinError( LPTSTR lpszFunction );
	TCHAR					wndTitle[MAX_LOADSTRING];					
	TCHAR					wndClass[MAX_LOADSTRING];	
	HINSTANCE				hAppInstance;
	HWND					hMainWnd;
	ID3D11Device*			d3DDevice;
	ID3D11DeviceContext*	d3DImmediateContext;
	UINT					msaaQuality;
	bool					enable4xMSAA;
	int						clientWidth;
	int						clientHeight;
	IDXGISwapChain*			swapChain;
	ID3D11Texture2D*		depthStencilBuffer;
	ID3D11RenderTargetView* renderTargetView;
	ID3D11DepthStencilView* depthStencilView;
	D3D11_VIEWPORT			screenViewport;
};

