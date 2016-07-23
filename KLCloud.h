#pragma once
#ifndef KLCLOUD_H
#define KLCLOUD_H

#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core/core.hpp>

typedef pcl::PointXYZRGB PointType;

class KLCloud {
public:
	KLCloud(BYTE**, BYTE**, LONG**, int, int); 
	~KLCloud(); 

	bool clearPrevDrawing(); 

	bool drawKCLoud(); 

	bool drawLHand(const std::vector<cv::Vec3d>& palms, const std::vector<std::vector<cv::Vec3d>>& fingers); 

	bool updatePointCloud(const std::vector<cv::Vec3d>& palms, const std::vector<std::vector<cv::Vec3d>>& fingers); 

private:
	const static float											badFloat; 

	boost::shared_ptr<pcl::visualization::PCLVisualizer>		m_pViewer;
	pcl::PointCloud<PointType>::Ptr								m_pCloud;


	// Variables for Kinect
	BYTE**														m_ppColorBuffer; 
	BYTE**														m_ppDepthBuffer;
	LONG**														m_ppColorCoordinates;
	int															m_width; 
	int															m_height; 

	float														m_maxZ; // Points with Z>m_maxZ will be extracted

};

#endif