#ifndef CALCLOD1_H
#define CALCLOD1_H


#include "Tools.h"

using namespace std;
using namespace pcl;



class calcLOD1{

private:

	Tools tool;


public:

	calcLOD1();
	virtual ~calcLOD1();



	/*
	*Name:		calcLOD1
	*Function:	����LOD1�Ĳ�������b�����t���߶�h������ʾ
	*Input:		cloud	�������
	*			size	���ƴ�С
	*			x		����x��������
	*			y		����y��������
	*			z		����z��������
	*Output:	b		ģ�ͳ���
	*			t		ģ�Ϳ��
	*			h		ģ�͸߶�
	*Return:	NULL
	*/
	void calc(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int size, double* x, double* y, double* z);

};

#endif	// CALCLOD1_H
