#ifndef TOOLS_H
#define TOOLS_H

#include <iostream>
#include <vector>

#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>

using namespace std;
using namespace pcl;

class Tools
{

public:

	Tools();
	virtual ~Tools();

	//求数组最大值
	double
		max_array(double *array, int n);//array 为int类型的数组的指针 ，n为数组元素个数

	//求数组最小值
	double
		min_array(double *array, int n);//array 为int类型的数组的指针 ，n为数组元素个数


	//计算点云密度（点云间平均间距）
	double
		computeCloudResolution(const pcl::PointCloud<PointXYZ>::ConstPtr &cloud);


	//************************************
	// Method:    boundaryEstimation
	// FullName:  Tools::boundaryEstimation
	// Access:    public 
	// Returns:   void
	// Qualifier:
	// Parameter: pcl::PointCloud<pcl::Boundary> & boundaries
	// Parameter: pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_cuted
	// Parameter: double res_cloud
	// Parameter: pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_boundaries
	//************************************
	void boundaryEstimation(pcl::PointCloud<pcl::Boundary>& boundaries,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_cuted,
		double res_cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_boundaries);


	//************************************
	// Method:    normalEstimation
	// FullName:  Tools::normalEstimation
	// Access:    public 
	// Returns:   void
	// Qualifier:
	// Parameter: pcl::PointCloud<pcl::Normal>::Ptr & cloud_normal
	// Parameter: pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_input
	//************************************
	void normalEstimation(pcl::PointCloud<pcl::Normal>::Ptr& cloud_normal,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_input);
};


#endif // TOOLS_H