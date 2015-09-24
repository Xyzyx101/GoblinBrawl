#pragma once
#include <vector>
#include <memory>
#include "DirectX_11_1_Includes.h"
#include "GameTimer.h"
#include "Camera.h"
#include "Floor.h"
#include "Lava.h"
#include "Walls.h"
#include "FirePlinth.h"
#include "Goblin.h"
#include "Lighting.h"
#include "Keyboard.h"
#include "GamePad.h"

class PhysicsWorld;

#define MAX_LOADSTRING 100

class Game {
public:
	Game( HINSTANCE hInstance, int clientWidth, int clientHeight );
	~Game();
	bool Init();
	void OnResize();
	LRESULT MsgProc( HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam );
	int Run();
	void OnMouseDown( WPARAM btnState, int x, int y );
	void OnMouseUp( WPARAM btnState, int x, int y );
	void OnMouseMove( WPARAM btnState, int x, int y );

private:
	bool InitMainWindow();
	bool InitDirect3D();
	bool LoadGameObjects();
	void DisplayWinError( LPTSTR lpszFunction );
	void CalculateFrameStats();
	void Update(float dt);
	void Draw();
	float AspectRatio();
	TCHAR								wndTitle[MAX_LOADSTRING];					
	TCHAR								wndClass[MAX_LOADSTRING];	
	HINSTANCE							hAppInstance;
	HWND								hMainWnd;
	ID3D11Device*						d3DDevice;
	ID3D11DeviceContext*				d3DImmediateContext;
	ID3D11RasterizerState*				d3DRasterizerState;
	UINT								msaaQuality;
	bool								enable4xMSAA;
	int									clientWidth;
	int									clientHeight;
	IDXGISwapChain*						swapChain;
	ID3D11Texture2D*					depthStencilBuffer;
	ID3D11RenderTargetView*				renderTargetView;
	ID3D11DepthStencilView*				depthStencilView;
	D3D11_VIEWPORT						screenViewport;
	GameTimer							timer;
	bool								paused;
	bool								resizing;
	Camera								camera;
	Floor								floor;
	Walls								walls;
	Lava								lava;
	FirePlinth							firePlinth;
	Lighting							lighting;
	Goblin								goblin;
	PhysicsWorld*						physicsWorld;
	std::unique_ptr<Keyboard>			keyboard;
	std::unique_ptr<
		Keyboard::KeyboardStateTracker>	kbTracker;
	std::unique_ptr<GamePad>			gamePad;
	POINT								lastMousePos;
};

