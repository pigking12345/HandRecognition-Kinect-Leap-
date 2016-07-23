#include "LListener.h"

#define FRAME_DELAY 10

MyListener::MyListener():
	m_lastFrame(),
	m_updated(FALSE),
	m_fFrameScale(0.001f),
	m_hFrameMutex(INVALID_HANDLE_VALUE)
{}

void MyListener::onInit(const Controller& controller) {
	m_hFrameMutex = CreateMutex(NULL, FALSE, NULL);
}

void MyListener::onConnect(const Controller& controller) {
}

void MyListener::onDisconnect(const Controller& controller) {
}

void MyListener::onExit(const Controller& controller) {
}

void MyListener::onFrame(const Controller& controller) {
	int frame_index = FRAME_DELAY;
	Leap::Frame frame; 
	frame = controller.frame(frame_index); 
	WaitForSingleObject(m_hFrameMutex, INFINITE);
	m_lastFrame = frame;
	ReleaseMutex(m_hFrameMutex);
}

Frame& MyListener::getLastFrame(){
	return m_lastFrame;
}

void MyListener::onFocusGained(const Controller& controller) {
}

void MyListener::onFocusLost(const Controller& controller) {
}
