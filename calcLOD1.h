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
	*Function:	计算LOD1的参数长度b，宽度t，高度h，并显示
	*Input:		cloud	输入点云
	*			size	点云大小
	*			x		点云x坐标数组
	*			y		点云y坐标数组
	*			z		点云z坐标数组
	*Output:	b		模型长度
	*			t		模型宽度
	*			h		模型高度
	*Return:	NULL
	*/
	void calc(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int size, double* x, double* y, double* z);

};

#endif	// CALCLOD1_H
