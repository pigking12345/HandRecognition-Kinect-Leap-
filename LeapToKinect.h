#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <iostream>

using namespace cv; 
using namespace std;

#pragma once

void leapToKinectRT(const vector<Vec3d>& l_data, vector<Vec3d>& k_data, Mat& R, Mat& T){
	int num_of_p = (int)l_data.size(); 

	// get mean of each dimension for both leap and kinect 
	Mat l_mean(3,1,CV_64F,0.0f); 
	Mat k_mean(3,1,CV_64F,0.0f); 
	for(int i=0; i<num_of_p; i++){
		for (int j=0;j<3;j++){
			l_mean.at<double>(j,0) += l_data[i][j];
			k_mean.at<double>(j,0) += k_data[i][j]; 
		}
	}
	l_mean = (1.0/num_of_p)*l_mean; 
	k_mean = (1.0/num_of_p)*k_mean;
	cout<<"l_mean:"<<endl<<l_mean<<endl;
	cout<<"k_mean:"<<endl<<k_mean<<endl; 

	vector<Mat> norm_l_data(num_of_p); 
	vector<Mat> norm_k_data(num_of_p);
	for(int i=0; i<num_of_p; i++){
		norm_l_data[i] = Mat(l_data[i]) - l_mean; 
		norm_k_data[i] = Mat(k_data[i]) - k_mean; 
	}

	Mat M = Mat::zeros(3,3,CV_64F); 
	for(int i=0; i<num_of_p; i++){
		M = M + norm_l_data[i]*norm_k_data[i].t(); 
	}

	cout<<"M = "<<endl<<M<<endl<<endl; 

	Mat eigen_val=Mat::zeros(3,3,CV_64F); 
	Mat eigen_vec=Mat::zeros(3,3,CV_64F); 
	Mat eigen_vec_inv=Mat::zeros(3,3,CV_64F); 
	Mat S = Mat::zeros(3,3,CV_64F);

	Mat M_M = M.t()*M; 
	cout<<"M'*M = "<<endl<<M_M<<endl<<endl; 

	eigen(M_M, eigen_val, eigen_vec); 
	cout<<"V="<<endl<<eigen_vec<<endl<<endl; 
	cout<<"D="<<endl<<eigen_val<<endl<<endl; 

	eigen_vec = eigen_vec.t();

	for (int i=0; i<3; i++)
		S.at<double>(i,i) = sqrt(eigen_val.at<double>(i,0)); 

	cout<<"S="<<endl<<S<<endl<<endl; 
	invert(eigen_vec, eigen_vec_inv); 
	Mat M_M_sqrt = Mat::zeros(3,3,CV_64F); 
	M_M_sqrt = eigen_vec*S*eigen_vec_inv; 
	cout<<"(M'*M)^(0.5) = "<<endl<<M_M_sqrt<<endl<<endl; 

	Mat M_M_sqrt_inv = Mat::zeros(3,3,CV_64F); 
	invert(M_M_sqrt, M_M_sqrt_inv,DECOMP_CHOLESKY); 
	cout<<"(M'*M)^(-0.5)="<<endl<<M_M_sqrt_inv<<endl<<endl; 

	R = M*M_M_sqrt_inv; 
	T = k_mean - R*l_mean; 
}

void leapToKinectRT(const vector<vector<float>>& l_data, const vector<vector<float>>& k_data, Mat& R, Mat& T){
	int num_of_p = (int)l_data.size(); 
	vector<Vec3d> l_data_vec;
	vector<Vec3d> k_data_vec;

	for (int i=0; i<num_of_p; i++){
		l_data_vec.push_back( Vec3d(l_data[i][0], l_data[i][1], l_data[i][2]));
		k_data_vec.push_back( Vec3d(k_data[i][0], k_data[i][1], k_data[i][2]));
	}

	leapToKinectRT(l_data_vec, k_data_vec, R, T); 
}

void transformLeap(const vector<Vec3d>& l_data, vector<Vec3d>& l_new_data, const Mat& R, const Mat& T){
	int num_of_p = (int)l_data.size(); 

	if(l_new_data.size()!=num_of_p)
		l_new_data.resize(num_of_p); 

	for(int i=0; i<num_of_p; i++){
		Mat temp = R*Mat(l_data[i])+T; 
		for(int j=0; j<3; j++){
			l_new_data[i][j] = temp.at<double>(j,0); 
		}
	}
}

void transformLeap(const Vec3d& l_data, Vec3d& l_new_data, const Mat& R, const Mat& T){
	Mat temp = R*Mat(l_data)+T; 
	for(int j=0; j<3; j++){
		l_new_data[j] = temp.at<double>(j,0); 
	}
}