#ifndef KAPI_H
#define KAPI_H
#include <Windows.h>
#include <iostream>
#include "NuiApi.h"


class Kinect{
public: 
	Kinect();
	virtual ~Kinect();

	HRESULT createFirstConnected();
	HRESULT fetchData();
	int getColorWidth(){ return 640; }
	int getColorHeight(){ return 480; }

	BYTE** getColorBuffer(){ return m_pColorBuffer; }
	BYTE** getDepthBuffer(){ return m_pDepthBuffer; }
	NUI_SKELETON_FRAME** getSkeletonFrame(){ return m_pSkeletonFrame; }

	int getColorBufferSize(){ return m_colorBufferSize; }
	int getDepthBufferSize(){ return m_depthBufferSize; }

	void getBuffers(BYTE**& colorBuf, BYTE**& depthBuf, NUI_SKELETON_FRAME**& skeletonFrame){
		colorBuf = m_pColorBuffer; 
		depthBuf = m_pDepthBuffer; 
		skeletonFrame = m_pSkeletonFrame; 
		return; 
	}

	INT getColorPitch(){ return m_colorBufferPitch; }
	INT getDepthPitch(){ return m_depthBufferPitch; }
	INuiSensor* getSensor(){return m_pNuiSensor; }

	bool isUpdated(){
		if(m_depthUpdated&&m_colorUpdated){
			m_depthUpdated = m_colorUpdated = false;
			return true;
		}
		return false;
	}

private:
	void unInitialize();
	bool					m_bInitialized;
	INuiSensor*				m_pNuiSensor;
	HANDLE					m_hColorStreamHandle;
	HANDLE					m_hNextColorFrameEvent;
	HANDLE                  m_hDepthStreamHandle;
	HANDLE                  m_hNextDepthFrameEvent;
	HANDLE					m_hNextSkeletonEvent;

	BYTE**					m_pColorBuffer;
	INT						m_colorBufferSize;
	INT						m_colorBufferPitch;
	BYTE**					m_pDepthBuffer;
	INT						m_depthBufferSize;
	INT						m_depthBufferPitch;
	NUI_SKELETON_FRAME**	m_pSkeletonFrame;
	INT						m_skeletonBufferSize;

	bool					m_depthUpdated;
	bool					m_colorUpdated;
};
#endif
