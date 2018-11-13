#ifndef CANNYBOUND_H
#define CANNYBOUND_H

#include "core/core.hpp"  
#include "highgui/highgui.hpp"  
#include "imgproc/imgproc.hpp"  
#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;

class canny_bound{

public:

	canny_bound();
	virtual ~canny_bound();

	//******************高斯卷积核生成函数*************************
	//第一个参数gaus是一个指向含有N个double类型数组的指针；
	//第二个参数size是高斯卷积核的尺寸大小；
	//第三个参数sigma是卷积核的标准差
	//*************************************************************
	void GetGaussianKernel(double **gaus, const int size, const double sigma);

	//******************灰度转换函数*************************
	//第一个参数image输入的彩色RGB图像；
	//第二个参数imageGray是转换后输出的灰度图像；
	//*************************************************************
	void ConvertRGB2GRAY(const Mat &image, Mat &imageGray);

	//******************高斯滤波*************************
	//第一个参数imageSource是待滤波原始图像；
	//第二个参数imageGaussian是滤波后输出图像；
	//第三个参数gaus是一个指向含有N个double类型数组的指针；
	//第四个参数size是滤波核的尺寸
	//*************************************************************
	void GaussianFilter(const Mat imageSource, Mat &imageGaussian, double **gaus, int size);
	//******************Sobel算子计算X、Y方向梯度和梯度方向角********************
	//第一个参数imageSourc原始灰度图像；
	//第二个参数imageSobelX是X方向梯度图像；
	//第三个参数imageSobelY是Y方向梯度图像；
	//第四个参数pointDrection是梯度方向角数组指针
	//*************************************************************
	void SobelGradDirction(const Mat imageSource, Mat &imageSobelX, Mat &imageSobelY, double *&pointDrection);
	//******************计算Sobel的X和Y方向梯度幅值*************************
	//第一个参数imageGradX是X方向梯度图像；
	//第二个参数imageGradY是Y方向梯度图像；
	//第三个参数SobelAmpXY是输出的X、Y方向梯度图像幅值
	//*************************************************************
	void SobelAmplitude(const Mat imageGradX, const Mat imageGradY, Mat &SobelAmpXY);
	//******************局部极大值抑制*************************
	//第一个参数imageInput输入的Sobel梯度图像；
	//第二个参数imageOutPut是输出的局部极大值抑制图像；
	//第三个参数pointDrection是图像上每个点的梯度方向数组指针
	//*************************************************************
	void LocalMaxValue(const Mat imageInput, Mat &imageOutput, double *pointDrection);

	//******************双阈值处理*************************
	//第一个参数imageInput输入和输出的的Sobel梯度幅值图像；
	//第二个参数lowThreshold是低阈值
	//第三个参数highThreshold是高阈值
	//******************************************************
	void DoubleThreshold(Mat &imageIput, double lowThreshold, double highThreshold);
	//******************双阈值中间像素连接处理*********************
	//第一个参数imageInput输入和输出的的Sobel梯度幅值图像；
	//第二个参数lowThreshold是低阈值
	//第三个参数highThreshold是高阈值
	//*************************************************************
	void DoubleThresholdLink(Mat &imageInput, double lowThreshold, double highThreshold);
};
#endif
