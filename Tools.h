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

	//���������ֵ
	double
		max_array(double *array, int n);//array Ϊint���͵������ָ�� ��nΪ����Ԫ�ظ���

	//��������Сֵ
	double
		min_array(double *array, int n);//array Ϊint���͵������ָ�� ��nΪ����Ԫ�ظ���


	//��������ܶȣ����Ƽ�ƽ����ࣩ
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