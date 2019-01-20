#pragma once

#include "WindowContainer.h"
#include "Timer.h"
#include "GameActor.h"


class Engine : WindowContainer
{
public:
	bool Initialize(HINSTANCE hInstance,
		const std::string& window_title, const std::string& window_class,
		int width, int height);
	bool ProcessMessages();
	void Update();
	void RenderFrame();
private:
	Timer timer;
	actorBox actors[200];
};
