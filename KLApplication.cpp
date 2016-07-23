#include "KLApplication.h"

using namespace std; 
using namespace cv; 

KLApplication* KLApplication::s_instance = NULL; 

KLApplication::KLApplication(){
	m_ppColorBuffer = NULL; 
	m_ppDepthBuffer = NULL; 
	m_ppSkeletonFrame = NULL; 
	m_ppColorCoordinates = NULL; 
	m_pListener = NULL; 
	m_pKHandDetector = NULL; 
	m_pLProcessor = NULL; 
	m_pKLCloud = NULL; 
	
	m_currentState = 0; 

	// Initialize Kinect
	m_kinectInitialized = false; 
	m_pKinect = new Kinect(); 
	HRESULT hr = m_pKinect->createFirstConnected(); 
	if(FAILED(hr)){
		cout<<"Kinect connection failed. "<<endl; 
		exit(-1); 
	}	
	else{
		m_pKinect->getBuffers(m_ppColorBuffer, m_ppDepthBuffer, m_ppSkeletonFrame); 
		m_ppColorCoordinates = new LONG*(); 
		*m_ppColorCoordinates = new LONG[m_pKinect->getColorHeight()*m_pKinect->getColorWidth()*2];
		m_kinectInitialized = true; 
	}

	// Initialize Kinect Hand Detector
	KHandDetector::createControlWnd(); 
	m_pKHandDetector = new KHandDetector(m_pKinect, m_ppColorBuffer, m_ppDepthBuffer, m_ppSkeletonFrame, m_ppColorCoordinates); 

	// Initialize Leap Motion Listener
	m_pListener = new MyListener(); 
	MyListener::getController().addListener(*m_pListener); 

	// Initialize Leap Motion Processor
	m_pLProcessor = new LProcessor(m_pListener); 
}

KLApplication::~KLApplication(){
	// Delete Kinect Variables
	if(m_ppColorCoordinates){
		delete [] *m_ppColorCoordinates; 
		delete m_ppColorCoordinates; 
		m_ppColorCoordinates = NULL; 
	}

	if(m_pKHandDetector)
		delete m_pKHandDetector; 

	if(m_pListener){
		MyListener::getController().removeListener(*m_pListener); 
		delete m_pListener; 
	}

	if(m_pLProcessor)
		delete m_pLProcessor; 

	if(m_pKLCloud)
		delete m_pKLCloud; 
}

void KLApplication::run(){
	while(true){
		while(!m_pKinect->isUpdated()){
			waitKey(40); 
			m_pKinect->fetchData(); 
		}

		if(!m_pKHandDetector->runOnce(m_currentState)){
			continue; 
		}

		if(!m_pLProcessor->storeHandData()){
			continue;
		}

		if(!m_pLProcessor->matching2KinectData(m_pKHandDetector->getPalms(), m_pKHandDetector->getFingerTips())){
			continue;
		}

		m_currentState = 1; 

		m_pKHandDetector->drawLeapMotionPoint(m_pLProcessor->getTransPalmPos(), m_pLProcessor->getTransFingers());


		// Initialize KL Cloud
		if(!m_pKLCloud)
			m_pKLCloud = new KLCloud(m_ppColorBuffer, m_ppDepthBuffer, m_ppColorCoordinates, m_pKinect->getColorWidth(), m_pKinect->getColorHeight()); 

		if(!m_pKLCloud->updatePointCloud(m_pLProcessor->getTransPalmPos(), m_pLProcessor->getTransFingers()))
			break; 
	}
}

int main(){
	KLApplication* pKLApp = KLApplication::getInstance(); 
	pKLApp->run(); 
	delete pKLApp; 
	return 0; 
}