#include "Engine.h"

bool Engine::Initialize(HINSTANCE hInstance, 
	const std::string & window_title, const std::string & window_class, 
	int width, int height)
{
	timer.Start();

	if (!this->render_window.Initialize(this, hInstance, window_title, window_class, width, height))
		return false;
	if (!gfx.Initialize(this->render_window.GetHWND(), width, height))
		return false;

	return true;
}

bool Engine::ProcessMessages()
{
	return this->render_window.ProcessMessages();
}

void Engine::Update()
{
	float dt = (float)timer.GetMilisecondsElapsed();
	timer.Restart();

	while (!keyboard.CharBufferIsEmpty())
	{
		unsigned char ch = keyboard.ReadChar();
	}

	while (!keyboard.KeyBufferIsEmpty())
	{
		KeyboardEvent kbe = keyboard.ReadKey();
		unsigned char keycode = kbe.GetKeyCode();
	}

	const float mouseSpeed = 0.001f * dt;
	while (!mouse.EventBufferIsEmpty())
	{
		MouseEvent me = mouse.ReadEvent();

		if (mouse.IsLeftDown())
		{
			if (me.GetType() == MouseEvent::EventType::RAW_MODE)
			{
				this->gfx.camera.AdjustRotation((float)me.GetPosY() * mouseSpeed, (float)me.GetPosX() * mouseSpeed, 0.0f);
			}
		}
	}

	const float cameraSpeed = 0.005f * dt;
	if (keyboard.KeyIsPressed('W'))
		this->gfx.camera.AdjustPosition(this->gfx.camera.GetForwardVector() * cameraSpeed);
	if (keyboard.KeyIsPressed('A'))
		this->gfx.camera.AdjustPosition(this->gfx.camera.GetLeftVector() * cameraSpeed);
	if (keyboard.KeyIsPressed('S'))
		this->gfx.camera.AdjustPosition(this->gfx.camera.GetBackwardVector() * cameraSpeed);
	if (keyboard.KeyIsPressed('D'))
		this->gfx.camera.AdjustPosition(this->gfx.camera.GetRightVector() * cameraSpeed);
	if (keyboard.KeyIsPressed(VK_SPACE))
		this->gfx.camera.AdjustPosition(0.f, cameraSpeed, 0.f);
	if (keyboard.KeyIsPressed('Z'))
		this->gfx.camera.AdjustPosition(0.f, -cameraSpeed, 0.f);

	physics.step();
}

void Engine::RenderFrame()
{
	Chan::chLiteWorld* w = physics.getWorld();
	
	for (unsigned i = 0; i < w->bodies.size(); ++i)
	{
		actors[i].position = XMFLOAT3(w->bodies[i]->position.x,
									w->bodies[i]->position.y, 0.f);
		actors[i].scale = XMFLOAT3(w->bodies[i]->width.x, w->bodies[i]->width.y, 0.f);
		actors[i].rotation = w->bodies[i]->rotation;
	}

	gfx.RenderFrame(actors, w->bodies.size());
}
