#include "KAPI.h"

#define WIDTH 640
#define HEIGHT 480

Kinect::Kinect(){
	m_bInitialized = false;
	m_pNuiSensor = NULL;
	m_hColorStreamHandle = INVALID_HANDLE_VALUE;
	m_hNextColorFrameEvent = INVALID_HANDLE_VALUE;
	m_hDepthStreamHandle = INVALID_HANDLE_VALUE;
	m_hNextDepthFrameEvent = INVALID_HANDLE_VALUE;
	m_hNextSkeletonEvent = INVALID_HANDLE_VALUE; 
	m_pColorBuffer = new BYTE*();
	m_colorBufferSize = 0;
	m_colorBufferPitch = 0;
	m_pDepthBuffer = new BYTE*();
	m_depthBufferSize = 0;
	m_depthBufferPitch = 0;
	m_pSkeletonFrame = new NUI_SKELETON_FRAME*();
	*m_pSkeletonFrame = new NUI_SKELETON_FRAME(); 
	m_skeletonBufferSize = 0;
	m_depthUpdated = m_colorUpdated = false; 
}

Kinect::~Kinect(){
	unInitialize();
	if(m_pColorBuffer){
		delete [] *m_pColorBuffer;
		delete m_pColorBuffer;
		m_pColorBuffer = NULL; 
	}
	if(m_pDepthBuffer){
		delete [] *m_pDepthBuffer;
		delete m_pDepthBuffer;
		m_pDepthBuffer = NULL; 
	}
	if(m_pSkeletonFrame){
		delete *m_pSkeletonFrame;
		delete m_pSkeletonFrame;
		m_pSkeletonFrame = NULL; 
	}
}

void Kinect::unInitialize(){
	// Close Kinect
	if (m_pNuiSensor)
	{
		m_pNuiSensor->NuiShutdown();
		m_pNuiSensor = NULL;
	}

	// Close handles for created events
	if (m_hNextColorFrameEvent && (m_hNextColorFrameEvent != INVALID_HANDLE_VALUE))
	{
		CloseHandle(m_hNextColorFrameEvent);
		m_hNextColorFrameEvent = NULL;
	}

	if (m_hNextDepthFrameEvent && (m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE))
	{
		CloseHandle(m_hNextDepthFrameEvent);
		m_hNextDepthFrameEvent = NULL;
	}

	if (m_hNextSkeletonEvent && (m_hNextSkeletonEvent != INVALID_HANDLE_VALUE))
	{
		CloseHandle(m_hNextSkeletonEvent);
		m_hNextSkeletonEvent = NULL;
	}
}

// In current version, we can't set the resolution
// All sensors of Kinect is activated. 
HRESULT Kinect::createFirstConnected(){
	INuiSensor * pNuiSensor;
	HRESULT hr;

	int iSensorCount = 0;
	hr = NuiGetSensorCount(&iSensorCount);
	if (FAILED(hr))
	{
		return hr;
	}

	// Look at each Kinect sensor
	for (int i = 0; i < iSensorCount; ++i)
	{
		// Create the sensor so we can check status, if we can't create it, move on to the next
		hr = NuiCreateSensorByIndex(i, &pNuiSensor);
		if (FAILED(hr))
		{
			continue;
		}

		// Get the status of the sensor, and if connected, then we can initialize it
		hr = pNuiSensor->NuiStatus();
		if (S_OK == hr)
		{
			m_pNuiSensor = pNuiSensor;
			break;
		}

		// This sensor wasn't OK, so release it since we're not using it
		pNuiSensor->Release();
	}

	if (NULL != m_pNuiSensor)
	{
		// Initialize the Kinect and specify that we'll be using color, depth and skeletion 
		hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | 
			NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX |
			NUI_INITIALIZE_FLAG_USES_SKELETON);
		if (SUCCEEDED(hr))
		{
			// Create an event that will be signaled when depth data is available
			m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

			// Open a depth image stream to receive depth frames
			hr = m_pNuiSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
				NUI_IMAGE_RESOLUTION_640x480,
				NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE|NUI_IMAGE_STREAM_FLAG_DISTINCT_OVERFLOW_DEPTH_VALUES,
				//NUI_IMAGE_STREAM_FRAME_LIMIT_MAXIMUM,
				2,
				m_hNextDepthFrameEvent,
				&m_hDepthStreamHandle);
			//m_pNuiSensor->NuiImageStreamSetImageFrameFlags(&m_hDepthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE|NUI_IMAGE_STREAM_FLAG_DISTINCT_OVERFLOW_DEPTH_VALUES);

		}

		if (SUCCEEDED(hr))
		{
			// Open a color image stream to receive depth frames
			m_hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

			hr = m_pNuiSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_COLOR,
				NUI_IMAGE_RESOLUTION_640x480,
				0,
				2,
				m_hNextColorFrameEvent,
				&m_hColorStreamHandle);
		}
		if (SUCCEEDED(hr)){
			// Enable the skeleton tracking
			m_hNextSkeletonEvent = CreateEventW(NULL, TRUE, FALSE, NULL);
			//hr = m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonEvent, 0); 
			hr = m_pNuiSensor->NuiSkeletonTrackingEnable(
				m_hNextSkeletonEvent,  NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE | NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT);
		}
	}

	if (NULL == m_pNuiSensor || FAILED(hr))
	{
		return E_FAIL;
	}
	m_bInitialized = true;
	return hr;
}

HRESULT Kinect::fetchData(){
	if (!m_bInitialized)
		return E_NUI_DEVICE_NOT_READY;

	// Fail if Kinect is not initialized
	if (!m_pNuiSensor)
	{
		return E_NUI_STREAM_NOT_ENABLED;
	}

	// Get next image stream frame
	NUI_IMAGE_FRAME imageFrame;
	HRESULT hr = E_NUI_STREAM_NOT_ENABLED;

	m_colorUpdated = false; 
	m_depthUpdated = false; 


	if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0) )
	{

		hr = m_pNuiSensor->NuiImageStreamGetNextFrame(
			m_hDepthStreamHandle,
			0,
			&imageFrame);
		if (FAILED(hr))
		{
			return hr;
		}

		// Lock frame texture to allow for copy
		INuiFrameTexture* pTexture = imageFrame.pFrameTexture;
		NUI_LOCKED_RECT lockedRect;
		pTexture->LockRect(0, &lockedRect, NULL, 0);

		// Check if image is valid
		if (lockedRect.Pitch != 0)
		{
			// Copy image information into buffer
			BYTE* pBuffer = lockedRect.pBits;
			INT size = lockedRect.size;
			INT pitch = lockedRect.Pitch;

			// Only reallocate memory if the buffer size has changed
			if (size != m_depthBufferSize)
			{
				delete[] (*m_pDepthBuffer);
				*m_pDepthBuffer = new BYTE[size];
				m_depthBufferSize = size;
			}
			memcpy_s(*m_pDepthBuffer, size, pBuffer, size);

			m_depthBufferPitch = pitch;
			m_depthUpdated = true;
		}

		// Unlock texture
		pTexture->UnlockRect(0);

		// Release image stream frame
		hr = m_pNuiSensor->NuiImageStreamReleaseFrame(m_hDepthStreamHandle, &imageFrame);
	}

	if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextColorFrameEvent, 0) ){
		hr = m_pNuiSensor->NuiImageStreamGetNextFrame(
			m_hColorStreamHandle,
			0,
			&imageFrame);
		if (FAILED(hr))
		{
			return hr;
		}

		// Lock frame texture to allow for copy
		INuiFrameTexture* pTexture = imageFrame.pFrameTexture;
		NUI_LOCKED_RECT lockedRect;
		pTexture->LockRect(0, &lockedRect, NULL, 0);

		// Check if image is valid
		if (lockedRect.Pitch != 0)
		{
			// Copy image information into buffer so it doesn't get overwritten later
			BYTE* pBuffer = lockedRect.pBits;
			INT size = lockedRect.size;
			INT pitch = lockedRect.Pitch;

			// Only reallocate memory if the buffer size has changed
			if (size != m_colorBufferSize)
			{
				delete[] (*m_pColorBuffer);
				*m_pColorBuffer = new BYTE[size];
				m_colorBufferSize = size;
			}
			memcpy_s(*m_pColorBuffer, size, pBuffer, size);


			m_colorBufferPitch = pitch;
		}

		// Unlock texture

		pTexture->UnlockRect(0);
		hr = m_pNuiSensor->NuiImageStreamReleaseFrame(m_hColorStreamHandle, &imageFrame);
		if (FAILED(hr)){
			return hr;
		}
		m_colorUpdated = true;
	}

	// Process skeleton
	if(m_colorUpdated&&m_depthUpdated){
		if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextSkeletonEvent, 0) )
		{
			hr = m_pNuiSensor->NuiSkeletonGetNextFrame(0, *m_pSkeletonFrame);
			if ( FAILED(hr) )
			{
				return hr;
			}
		}
	}
	return hr;
}