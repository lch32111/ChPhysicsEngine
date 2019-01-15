#pragma once

#include "WindowContainer.h"

class Engine : WindowContainer
{
public:
	bool Initialize(HINSTANCE hInstance,
		const std::string& window_title, const std::string& window_class,
		int width, int height);
	bool ProcessMessages();
	void Update();
	void RenderFrame();
};
