#pragma once
#ifndef LLISTENER_H
#define LLISTENER_H
#include <Windows.h>
#include <iostream>
#include "Leap.h"

using namespace Leap;

class MyListener : public Leap::Listener
{
public:
	MyListener();
	~MyListener(){
		CloseHandle(m_hFrameMutex);
	}

	Leap::Frame& getLastFrame();

	static Leap::Controller& getController()
	{
		static Leap::Controller s_controller;
		s_controller.setPolicyFlags(Controller::POLICY_BACKGROUND_FRAMES);
		return  s_controller;
	}

	Leap::Vector scalePoint(Leap::Vector v){
		return v * m_fFrameScale;
	}

	virtual void onInit(const Leap::Controller&);
	virtual void onConnect(const Leap::Controller&);
	virtual void onDisconnect(const Leap::Controller&);
	virtual void onExit(const Leap::Controller&);
	virtual void onFrame(const Leap::Controller&);
	virtual void onFocusGained(const Leap::Controller&);
	virtual void onFocusLost(const Leap::Controller&);

	HANDLE						m_hFrameMutex;

private:
	Leap::Frame                 m_lastFrame;
	bool						m_updated;
	float                       m_fFrameScale;
};

#endif