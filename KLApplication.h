#ifndef KLAPPLICATION_H
#define KLAPPLICATION_H
#include "KAPI.h"
#include "KHandDetector.h"
#include "LListener.h"
#include "LProcessor.h"
#include "KLCloud.h"

class KLApplication {
public:
	KLApplication(); 
	~KLApplication();

	static KLApplication* getInstance(){
		if(!s_instance){
			s_instance = new KLApplication(); 
		}
		return s_instance; 
	}

	void run(); 

private:
	static KLApplication* s_instance; 

	// Variables about Kinect
	Kinect*								m_pKinect; 
	bool								m_kinectInitialized; 
	BYTE**								m_ppColorBuffer;
	BYTE**								m_ppDepthBuffer; 
	NUI_SKELETON_FRAME**				m_ppSkeletonFrame;
	LONG**								m_ppColorCoordinates;

	// Variables about Kinect Hand Detector
	KHandDetector*						m_pKHandDetector; 

	// Variables about Leap Motion Listener
	MyListener*							m_pListener; 

	// Variables about Leap Motion Processor
	LProcessor*							m_pLProcessor; 

	// Variables about Kinect Leap Motion Cloud
	KLCloud*							m_pKLCloud; 

	int									m_currentState;
	
};
#endif
