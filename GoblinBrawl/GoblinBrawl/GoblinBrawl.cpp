// GoblinBrawl.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include <stdio.h>
#include "GoblinBrawl.h"
#include "Game.h"

#define DEBUG_CONSOLE

#define MAX_LOADSTRING 100

int APIENTRY _tWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPTSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
#if defined(DEBUG) | defined(_DEBUG)
#if defined(DEBUG_CONSOLE)
	// redirects sdterr and stdout to another window for debugging
	if( AttachConsole( ATTACH_PARENT_PROCESS )||AllocConsole() ) {
		freopen( "CONOUT$", "w", stdout );
		freopen( "CONOUT$", "w", stderr );
	}
#endif
#endif
	
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

 	Game game( hInstance, 1280, 1024 );
	if( !game.Init() ) {
		return -1;
	}
	return game.Run();
}