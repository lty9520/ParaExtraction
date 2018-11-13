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

	//******************��˹��������ɺ���*************************
	//��һ������gaus��һ��ָ����N��double���������ָ�룻
	//�ڶ�������size�Ǹ�˹����˵ĳߴ��С��
	//����������sigma�Ǿ���˵ı�׼��
	//*************************************************************
	void GetGaussianKernel(double **gaus, const int size, const double sigma);

	//******************�Ҷ�ת������*************************
	//��һ������image����Ĳ�ɫRGBͼ��
	//�ڶ�������imageGray��ת��������ĻҶ�ͼ��
	//*************************************************************
	void ConvertRGB2GRAY(const Mat &image, Mat &imageGray);

	//******************��˹�˲�*************************
	//��һ������imageSource�Ǵ��˲�ԭʼͼ��
	//�ڶ�������imageGaussian���˲������ͼ��
	//����������gaus��һ��ָ����N��double���������ָ�룻
	//���ĸ�����size���˲��˵ĳߴ�
	//*************************************************************
	void GaussianFilter(const Mat imageSource, Mat &imageGaussian, double **gaus, int size);
	//******************Sobel���Ӽ���X��Y�����ݶȺ��ݶȷ����********************
	//��һ������imageSourcԭʼ�Ҷ�ͼ��
	//�ڶ�������imageSobelX��X�����ݶ�ͼ��
	//����������imageSobelY��Y�����ݶ�ͼ��
	//���ĸ�����pointDrection���ݶȷ��������ָ��
	//*************************************************************
	void SobelGradDirction(const Mat imageSource, Mat &imageSobelX, Mat &imageSobelY, double *&pointDrection);
	//******************����Sobel��X��Y�����ݶȷ�ֵ*************************
	//��һ������imageGradX��X�����ݶ�ͼ��
	//�ڶ�������imageGradY��Y�����ݶ�ͼ��
	//����������SobelAmpXY�������X��Y�����ݶ�ͼ���ֵ
	//*************************************************************
	void SobelAmplitude(const Mat imageGradX, const Mat imageGradY, Mat &SobelAmpXY);
	//******************�ֲ�����ֵ����*************************
	//��һ������imageInput�����Sobel�ݶ�ͼ��
	//�ڶ�������imageOutPut������ľֲ�����ֵ����ͼ��
	//����������pointDrection��ͼ����ÿ������ݶȷ�������ָ��
	//*************************************************************
	void LocalMaxValue(const Mat imageInput, Mat &imageOutput, double *pointDrection);

	//******************˫��ֵ����*************************
	//��һ������imageInput���������ĵ�Sobel�ݶȷ�ֵͼ��
	//�ڶ�������lowThreshold�ǵ���ֵ
	//����������highThreshold�Ǹ���ֵ
	//******************************************************
	void DoubleThreshold(Mat &imageIput, double lowThreshold, double highThreshold);
	//******************˫��ֵ�м��������Ӵ���*********************
	//��һ������imageInput���������ĵ�Sobel�ݶȷ�ֵͼ��
	//�ڶ�������lowThreshold�ǵ���ֵ
	//����������highThreshold�Ǹ���ֵ
	//*************************************************************
	void DoubleThresholdLink(Mat &imageInput, double lowThreshold, double highThreshold);
};
#endif
