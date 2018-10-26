#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/boundary.h>	
#include <pcl/features/normal_3d_omp.h>


//计算点云密度（点云间平均间距）
double
computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
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
	normEst.setRadiusSearch(res_cloud * 15);
	normEst.compute(*normals);

	boundEst.setInputCloud(cloud_cuted);
	boundEst.setInputNormals(normals);
	boundEst.setRadiusSearch(res_cloud * 10);
	boundEst.setAngleThreshold(M_PI / 4);
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
	boundEst.compute(boundaries);

	//cout << "boundaries = " << boundaries << endl;
	//cout << "test = " << boundaries.points[0].boundary_point << endl;


	int countBoundaries = 0;
#pragma omp parallel for
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
#pragma omp parallel for
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


int
main(int argc, char** argv)
{
	
	//初始化PointCloud对象
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//加载点云文件
	pcl::io::loadPCDFile("quetidian33.pcd", *cloud);
	//std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;
	std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;


	
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rol(new pcl::PointCloud<pcl::PointXYZ>);

	//初始化边界对象
	pcl::PointCloud<pcl::Boundary> boundaries;
	//初始化存储边界对象的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundaries(new pcl::PointCloud <pcl::PointXYZ>);


	// Fill in the cloud data
	//cloud->width = 5;
	//cloud->height = 1;
	//cloud->points.resize(cloud->width * cloud->height);
	/*
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	*/



	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(100);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_sor);



	
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	// build the filter
	outrem.setInputCloud(cloud_sor);
	outrem.setRadiusSearch(0.2);
	outrem.setMinNeighborsInRadius(1);
	// apply filter
	outrem.filter(*cloud_rol);


	double res_cloud = computeCloudResolution(cloud_rol);

	boundaryEstimation(boundaries, cloud_rol, res_cloud, cloud_boundaries);


	
	std::cerr << "Cloud before filtering: " << cloud->points.size() << std::endl;
	
	std::cerr << "Cloud after StatisticalOutlierRemoval filtering: " << cloud_sor->points.size() << std::endl;

	std::cerr << "Cloud after RadiusOutlierRemoval filtering: " << cloud_rol->points.size() << std::endl;

	std::cerr << "Cloud after BoundaryEstimation: " << cloud_boundaries->points.size() << std::endl;
	
	pcl::io::savePCDFileASCII("quetidian_linshi.pcd", *cloud_rol);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ori(new pcl::visualization::PCLVisualizer("原始点云"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> g(cloud, 0, 255, 0);
	viewer_ori->addPointCloud(cloud, g, "cc");
	viewer_ori->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cc");
	viewer_ori->addCoordinateSystem(1);

	while (!viewer_ori->wasStopped())
	{

		viewer_ori->spinOnce(1);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_sor(new pcl::visualization::PCLVisualizer("SOR点云"));
	//viewer_ori->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	viewer_sor->setBackgroundColor(255, 255, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gr(cloud_sor, 255, 0, 0);
	viewer_sor->addPointCloud(cloud_sor, gr, "cloud sor");
	viewer_sor->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud sor");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> boundaryoints_color_handler(cloud_boundaries, 0, 255, 0);
	//viewer_ori->addPointCloud(cloud_boundaries, boundaryoints_color_handler, "boundary");
	//viewer_ori->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "boundary");
	viewer_sor->addCoordinateSystem(1);
	//std::cerr << "Point cloud filtered data: " << cloud->points.size() << " points" << std::endl;
	cout << "sor cloud visulization finish" << endl;
	//pcl::io::savePCDFileASCII("queti111111.pcd", *cloud_filtered); //将点云保存到PCD文件中
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normal, 20, 0.03, "normals");
	while (!viewer_sor->wasStopped())
	{

		viewer_sor->spinOnce(1);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}



	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_rol(new pcl::visualization::PCLVisualizer("ROL点云"));
	//viewer_ori->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	viewer_rol->setBackgroundColor(255, 255, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gre(cloud_rol, 255, 0, 0);
	viewer_rol->addPointCloud(cloud_rol, gre, "cloud rol");
	viewer_rol->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud rol");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> boundaryoints_color_handler(cloud_boundaries, 0, 255, 0);
	//viewer_ori->addPointCloud(cloud_boundaries, boundaryoints_color_handler, "boundary");
	//viewer_ori->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "boundary");
	viewer_rol->addCoordinateSystem(1);
	//std::cerr << "Point cloud filtered data: " << cloud->points.size() << " points" << std::endl;
	cout << "rol cloud visulization finish" << endl;
	//pcl::io::savePCDFileASCII("queti111111.pcd", *cloud_filtered); //将点云保存到PCD文件中
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normal, 20, 0.03, "normals");
	while (!viewer_rol->wasStopped())
	{

		viewer_rol->spinOnce(1);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_be(new pcl::visualization::PCLVisualizer("Boundary点云"));
	//viewer_ori->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	viewer_be->setBackgroundColor(255, 255, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gree(cloud_boundaries, 255, 0, 0);
	viewer_be->addPointCloud(cloud_boundaries, gree, "cloud be");
	viewer_be->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud be");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> boundaryoints_color_handler(cloud_boundaries, 0, 255, 0);
	//viewer_ori->addPointCloud(cloud_boundaries, boundaryoints_color_handler, "boundary");
	//viewer_ori->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "boundary");
	viewer_be->addCoordinateSystem(1);
	//std::cerr << "Point cloud filtered data: " << cloud->points.size() << " points" << std::endl;
	cout << "rol cloud visulization finish" << endl;
	//pcl::io::savePCDFileASCII("queti111111.pcd", *cloud_filtered); //将点云保存到PCD文件中
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normal, 20, 0.03, "normals");
	while (!viewer_be->wasStopped())
	{

		viewer_be->spinOnce(1);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	system("pause");
	return (0);
}