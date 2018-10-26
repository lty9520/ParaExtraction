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
	//求数组最大值
	double
		max_array(double *array, int n)//array 为int类型的数组的指针 ，n为数组元素个数
	{
		double max = array[0];
		for (int i = 0; i < n; i++)
		{
			if (max < array[i])
			{
				max = array[i];
			}
		}
		return max;
	}

	//求数组最小值
	double
		min_array(double *array, int n)//array 为int类型的数组的指针 ，n为数组元素个数
	{
		double min = array[0];
		for (int i = 0; i< n; i++)
		{
			if (min > array[i])
			{
				min = array[i];
			}
		}
		return min;
	}


	//计算点云密度（点云间平均间距）
	double
		computeCloudResolution(const pcl::PointCloud<PointXYZ>::ConstPtr &cloud)
	{
		double res = 0.0;
		int n_points = 0;
		int nres;
		std::vector<int> indices(2);
		std::vector<float> sqr_distances(2);
		pcl::search::KdTree<PointXYZ> tree;
		tree.setInputCloud(cloud);

		for (size_t i = 0; i < cloud->size(); ++i)
		{
			if (!pcl_isfinite((*cloud)[i].x))
			{
				continue;
			}
			//Considering the second neighbor since the first is the point itself.
			nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
			if (nres == 2)
			{
				res += sqrt(sqr_distances[1]);
				++n_points;
			}
		}
		if (n_points != 0)
		{
			res /= n_points;
		}
		return res;

	}


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
		pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_boundaries){

		//BoundaryEstimation part test
		/*
		2.9  π/2		接缝角点
		7 3.8 π/2		小弧角点
		3 2.5 π/2		忽略小弧角点 缺一点

		zuizhong
		3.5		3		1.png + 2.png
		3.3		3		2.png 绿
		3		2.8		1.png lan
		2.8		2.8		1.png hui
		2.6		3		1.png cheng
		2.6		3.2		1.png hei
		*/



		pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normEst;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud_cuted));
		normEst.setNumberOfThreads(50);
		normEst.setRadiusSearch(res_cloud * 1);
		normEst.compute(*normals);

		boundEst.setInputCloud(cloud_cuted);
		boundEst.setInputNormals(normals);
		boundEst.setRadiusSearch(res_cloud * 2);
		boundEst.setAngleThreshold(M_PI / 4);
		boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
		boundEst.compute(boundaries);

		//cout << "boundaries = " << boundaries << endl;
		//cout << "test = " << boundaries.points[0].boundary_point << endl;


		int countBoundaries = 0;
		for (int i = 0; i < cloud_cuted->size(); i++){
			int j = 0;
			uint8_t x = (boundaries.points[i].boundary_point);
			int a = static_cast<int> (x);
			if (a == 1){
				countBoundaries++;

			}
		}

		//write boundaries cloud 

		cloud_boundaries->width = static_cast<int>(cloud_cuted->points.size());
		cloud_boundaries->height = 1;
		cloud_boundaries->points.resize(cloud_boundaries->width * cloud_cuted->height);
		cloud_boundaries->is_dense = true;
		int j = 0;
		for (int i = 0; i < static_cast<int>(cloud_cuted->points.size()); i++){
			if (boundaries.points[i].boundary_point != 0){
				cloud_boundaries->points[j].x = cloud_cuted->points[i].x;
				cloud_boundaries->points[j].y = cloud_cuted->points[i].y;
				cloud_boundaries->points[j].z = cloud_cuted->points[i].z;
				j++;
			}
			continue;
		}
		cloud_boundaries->width = j;
		cloud_boundaries->points.resize(cloud_boundaries->width * cloud_boundaries->height);

		cout << "point cloud boundary has:" << cloud_boundaries->points.size() << "data points" << endl;


		cout << "boundaries size = " << countBoundaries << endl;
	}


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
		pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_input){

		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>nor;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		nor.setInputCloud(cloud_input);
		nor.setNumberOfThreads(50);
		nor.setRadiusSearch(0.01);
		nor.setSearchMethod(tree);
		cout << "wojinliale" << endl;
		nor.compute(*cloud_normal);
	}
};


#endif // TOOLS_H