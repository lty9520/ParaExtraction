#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/segmentation/sac_segmentation.h> 



int
main(/*int argc, char** argv*/)
{
	// initialize PointClouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);


	//line1
	double line1[15] = { 0.471757, 0.542008, 0.167172, 
						0.471549, 0.546456, 0.167409, 
						0.471557, 0.550007, 0.16764, 
						0.471575, 0.555676, 0.167184, 
						0.471595, 0.561717, 0.167541 };

	int i, j = 0;
	int num_1 = 0;
	std::vector<std::vector<double>> l1(5);

	for (int i = 0; i < l1.size(); i++){
		l1[i].resize(3);
	}

	for (int i = 0; i < l1.size(); i++)
	{
		for (int j = 0; j < l1[0].size(); j++)
		{
			l1[i][j] = line1[num_1];
			num_1++;
		}
	}

	//test input
	/*
	for (int i = 0; i < l1.size(); i++)
	{
		for (int j = 0; j < l1[0].size(); j++)
		{
			cout << l1[i][j];
			cout << "\t";
		}
		cout << "\n";
	}
*/

	

	// populate our PointCloud with points
	cloud->width = l1.size();
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);
	for (int i = 0; i < l1.size(); i++)
	{
		/*
		if (pcl::console::find_argument(argc, argv, "-s") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0)
		{
			cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
			cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
			if (i % 5 == 0)
				cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
			else if (i % 2 == 0)
				cloud->points[i].z = sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
				- (cloud->points[i].y * cloud->points[i].y));
			else
				cloud->points[i].z = -sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
				- (cloud->points[i].y * cloud->points[i].y));
		}
		else
		{
			cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
			cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
			if (i % 2 == 0)
				cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
			else
				cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
		}
		*/
		
		
		cloud->points[i].x = l1[i][0];
		cloud->points[i].y = l1[i][1];
		cloud->points[i].z = l1[i][2];
		
	}

	for (int i = 0; i < cloud->points.size(); i++)
	{
		cout << cloud->points[i].x;
		cout << cloud->points[i].y;
		cout << cloud->points[i].z;
		cout << "\n";
	}

	//std::vector<int> inliers;



	//创建一个模型参数对象，用于记录结果
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//inliers表示误差能容忍的点 记录的是点云的序号
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// 创建一个分割器
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory-设置目标几何形状
	seg.setModelType(pcl::SACMODEL_LINE);
	//分割方法：随机采样法
	seg.setMethodType(pcl::SAC_RANSAC);
	//设置误差容忍范围
	seg.setDistanceThreshold(0.01);
	//输入点云
	seg.setInputCloud(cloud);
	//分割点云
	seg.segment(*inliers, *coefficients);



	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//viewer->addCoordinateSystem (1.0, "global");
	viewer->initCameraParameters();
	viewer->addLine(*coefficients);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(1);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	/*
	// created RandomSampleConsensus object and compute the appropriated model
	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
		model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
	if (pcl::console::find_argument(argc, argv, "-f") >= 0)
	{
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
		ransac.setDistanceThreshold(.01);
		ransac.computeModel();
		ransac.getInliers(inliers);
	}
	else if (pcl::console::find_argument(argc, argv, "-sf") >= 0)
	{
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
		ransac.setDistanceThreshold(.01);
		ransac.computeModel();
		ransac.getInliers(inliers);
	}

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

	// creates the visualization object and adds either our orignial cloud or all of the inliers
	// depending on the command line arguments specified.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	if (pcl::console::find_argument(argc, argv, "-f") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0)
		viewer = simpleVis(final);
	else
		viewer = simpleVis(cloud);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	*/
	
	return 0;
}