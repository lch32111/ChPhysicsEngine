#include "Engine.h"

bool Engine::Initialize(HINSTANCE hInstance, 
	const std::string & window_title, const std::string & window_class, 
	int width, int height)
{
	timer.Start();
	exit = false;

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

	/*
	while (!keyboard.CharBufferIsEmpty())
	{
		unsigned char ch = keyboard.ReadChar();
	}

	while (!keyboard.KeyBufferIsEmpty())
	{
		KeyboardEvent kbe = keyboard.ReadKey();
		unsigned char keycode = kbe.GetKeyCode();
	}

	const float mouseSpeed = 0.0004f * dt;
	while (!mouse.EventBufferIsEmpty())
	{
		MouseEvent me = mouse.ReadEvent();
		
		if (mouse.IsRightDown())
		{
			if (me.GetType() == MouseEvent::EventType::RAW_MODE)
			{
				this->gfx.camera.AdjustRotation((float)me.GetPosY() * mouseSpeed, (float)me.GetPosX() * mouseSpeed, 0.0f);
			}
		}
	}
	*/
	const float cameraSpeed = 0.01f * dt;
	if (keyboard.KeyIsPressed('W'))
	{	
		// this->gfx.camera.AdjustPosition(this->gfx.camera.GetForwardVector() * cameraSpeed);
		this->gfx.camera.AdjustPosition(0.f, cameraSpeed, 0.f);
	}

	if (keyboard.KeyIsPressed('A'))
	{
		this->gfx.camera.AdjustPosition(this->gfx.camera.GetLeftVector() * cameraSpeed);
	}

	if (keyboard.KeyIsPressed('S'))
	{
		this->gfx.camera.AdjustPosition(this->gfx.camera.GetBackwardVector() * cameraSpeed);
		// this->gfx.camera.AdjustPosition(0.f, -cameraSpeed, 0.f);
	}

	if (keyboard.KeyIsPressed('D'))
	{
		this->gfx.camera.AdjustPosition(this->gfx.camera.GetRightVector() * cameraSpeed);
	}	
	
	
	if (keyboard.KeyIsPressedOnce(VK_SPACE))
	{
		physics.launchBomb();
	}

	if (keyboard.KeyIsPressed(VK_ESCAPE))
	{
		this->exit = true;
	}

	if (keyboard.KeyIsPressedOnce('0'))
	{
		physics.demo0();
	}
	if (keyboard.KeyIsPressedOnce('1'))
	{
		physics.demo1();
	}
	if (keyboard.KeyIsPressedOnce('2'))
	{
		physics.demo2();
	}
	if (keyboard.KeyIsPressedOnce('3'))
	{
		physics.demo3();
	}
	if (keyboard.KeyIsPressedOnce('4'))
	{
		physics.demo4();
	}
	if (keyboard.KeyIsPressedOnce('5'))
	{
		physics.demo5();
	}

	if (keyboard.KeyIsPressedOnce(VK_OEM_PLUS))
		physics.increaseIteration();
	if (keyboard.KeyIsPressedOnce(VK_OEM_MINUS))
		physics.decreaseIteration();

	if (keyboard.KeyIsPressedOnce('M'))
	{
		physics.setWarmStart(!physics.isWarmStart());
	}
	if (keyboard.KeyIsPressedOnce('N'))
	{
		physics.setAccumImpulse(!physics.isAccumImpulse());
	}
	if (keyboard.KeyIsPressedOnce('B'))
	{
		physics.setPosCorrection(!physics.isPosCoorection());
	}

	physics.step();
}

void Engine::RenderFrame()
{
	Chan::chLiteWorld* w = physics.getWorld();
	pData.iterations = w->iterations;
	pData.useWarmStart = w->warmStarting;
	pData.accumulateImpulse = w->accumulateImpulses;
	pData.positionCorrection = w->positionCorrection;
	pData.numActors = w->bodies.size();
	for (unsigned i = 0; i < pData.numActors; ++i)
	{
		pData.actors[i].modelMatrix = XMMatrixScaling(w->bodies[i]->width.x, w->bodies[i]->width.y, 0.f);
		pData.actors[i].modelMatrix *= XMMatrixRotationZ(w->bodies[i]->rotation);
		pData.actors[i].modelMatrix *= XMMatrixTranslation(w->bodies[i]->position.x, w->bodies[i]->position.y, 0.f);
	}

	gfx.RenderFrame(pData);
}
