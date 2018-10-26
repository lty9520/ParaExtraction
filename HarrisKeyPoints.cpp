//#include <pcl/keypoints/harris_keypoint3D.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/harris_2d.h>
#include <pcl/keypoints/harris_6d.h>
#include <iostream>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>		//PCL的PCD格式文件的输入输出头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/common/impl/io.hpp>
#include <pcl/features/normal_3d_omp.h>

using namespace pcl;
using namespace std;
// -----Main-----
int
main(int argc, char** argv)

{

	DWORD start_time = GetTickCount(); //开始计时
	double search_radius = 0;
	double thresh = 0;
	// 自行创建一随机点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);      //输入点云
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_Harris(new pcl::PointCloud<pcl::PointXYZI>);  //输出点云
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_Harris2D(new pcl::PointCloud<pcl::PointXYZI>);  //输出点云
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_corners(new pcl::PointCloud<pcl::PointXYZI>);  //角点点云
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);		//点云法线
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_resort(new pcl::PointCloud<pcl::PointXYZ>);      //重新排列点云
	pcl::io::loadPCDFile("zhengti-bd.pcd", *point_cloud);                  //点云读入
	cout << "Please intput the searching radius:" << endl;
	//std::cin >> search_radius;
	cout << "Please intput the Threshold:" << endl;
	//std::cin >> thresh;


	cloud_corners->width = static_cast<int>(point_cloud->points.size());
	cloud_corners->height = 1;
	cloud_corners->points.resize(point_cloud->width * point_cloud->height);
	cloud_corners->is_dense = true;



	
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>nor;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	nor.setInputCloud(point_cloud);
	nor.setNumberOfThreads(50);
	nor.setRadiusSearch(0.8);
	nor.setSearchMethod(tree);
	//cout << "wojinliale" << endl;
	nor.compute(*cloud_normal);


	pcl::HarrisKeypoint3D< pcl::PointXYZ, pcl::PointXYZI, pcl::Normal > ne;  //创建对象
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setMethod(ne.HARRIS);
	ne.setNumberOfThreads(50);
	ne.setInputCloud(point_cloud);
	ne.setNormals(cloud_normal);
	ne.setSearchMethod(tree);                   //近邻搜索方法
	ne.setRadiusSearch(8.0f);      //r=search_radiusmm			15.0出2号点
	ne.setThreshold(0.0000f);                       //兴趣阈值
	ne.setNonMaxSupression(true);            //setThreshold前要激活它
	ne.setRefine(false);
	ne.compute(*cloud_Harris);                     //计算获取关键点          //经调试，运行至此部分出现错误

	/*
	pcl::HarrisKeypoint2D<pcl::PointXY, pcl::PointXYZI, pcl::Normal> harris2d;
	harris2d.setMethod(harris2d.HARRIS);
	harris2d.setNumberOfThreads(50);
	harris2d.setWindowHeight(1);
	harris2d.setSkippedPixels(1);
	harris2d.setWindowWidth(1);
	harris2d.setNonMaxSupression(true);
	harris2d.setRefine(true);
	harris2d.setThreshold(0.0000f);
	harris2d.setRadiusSearch(30.0f);
	harris2d.compute(*cloud_Harris2D);
	*/


	/*
	pcl::HarrisKeypoint6D<pcl::PointXYZRGB, PointXYZI, pcl::Normal>harris6d;
	harris6d.setNonMaxSupression(true);
	harris6d.setNumberOfThreads(50);
	harris6d.setRadius(0.1);
	harris6d.setRefine(true);
	harris6d.setThreshold(0.000f);
	harris6d.compute(*cloud_Harris2D);

	*/



	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	kdtree.setInputCloud(point_cloud);

	
	//kdtree.setInputCloud(point_cloud);


	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	float radius = 20.0f;



	for (int i = 0; i < point_cloud->points.size(); i++)
	{
		if (kdtree.radiusSearch(point_cloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j){

				if ((float)cloud_normal->points[i].normal_x * (float)cloud_normal->points[pointIdxRadiusSearch[j]].normal_x > 0)
				{
					cloud_corners->points[i].x = point_cloud->points[i].x;
					cloud_corners->points[i].y = point_cloud->points[i].y;
					cloud_corners->points[i].z = point_cloud->points[i].z;
				}


			}
			/*std::cout << "    " << point_cloud->points[pointIdxRadiusSearch[j]].x
			<< " " << point_cloud->points[pointIdxRadiusSearch[j]].y
			<< " " << point_cloud->points[pointIdxRadiusSearch[j]].z
			<< " (squared distance: " << pointRadiusSquaredDistance[j] << ")" << std::endl;*/
		}
	}





	std::cerr << "Harris Point cloud data: " << cloud_Harris->points.size() << " points" << std::endl;



	//pcl::PointCloud<pcl::PointXYZI>::Ptr test(new pcl::PointCloud<pcl::PointXYZI>);      //输入点云
	//pcl::io::loadPCDFile("queti_lable.pcd", *test);

	//pcl::io::savePCDFileASCII("normalzhuzi.pcd", *cloud_Harris); //将点云保存到PCD文件中
	//创建viewer对象
	//原始点云显示窗口
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ori(new pcl::visualization::PCLVisualizer("原始点云"));
	//viewer_ori->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ori_color_handler(point_cloud, 128, 138, 135);
	viewer_ori->addPointCloud(point_cloud, ori_color_handler, "point_cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_Harris_color_handler(cloud_Harris, 255, 0, 0);
	viewer_ori->addPointCloud(cloud_Harris, cloud_Harris_color_handler, "Harris");
	viewer_ori->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Harris");
	viewer_ori->addCoordinateSystem(1);
	//viewer_ori->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(point_cloud, cloud_normal, 1, 30, "normals");
	viewer_ori->setBackgroundColor(0, 0, 0);
	cout << "original cloud visulization finish" << endl;


	
	for (int i = 0; i < cloud_Harris->points.size(); i++){
		viewer_ori->addText3D(to_string(i), cloud_Harris->points[i], 1.0, 0.0, 0.0, 1.0);
	}

	//pcl::io::savePCDFileASCII("queti_lable.pcd", *cloud_Harris);

	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normal, 20, 0.03, "normals");
	while (!viewer_ori->wasStopped())
	{
		viewer_ori->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	int n;
	//int pid[100];
	int P_num;

	vector<int> p_id;

	cout << "输入要提取的点的个数：" << endl;
	cin >> P_num;
	cout << "输入要提取的点的点号" << endl;
	/*
	for (int i = 0; i < P_num; i ++)
	{
		cin >> n;
		if (n > cloud_Harris->points.size())
		{
			cout << "输入错误，最大点号为" << cloud_Harris->points.size() << "，请重新输入" << endl;
			cin >> n;
			p_id.push_back(n);
		} 
		else
		{
			
			p_id.push_back(n);
		}
		
	}

	cloud_resort->width = P_num;
	cloud_resort->height = 1;
	cloud_resort->points.resize(cloud_resort->width * cloud_resort->height);
	cloud_resort->is_dense = true;


	cout << "提取完成" << endl;

	
	for (int i = 0; i < P_num; i++)
	{
		cout << "P_id[" << i << "] = " << p_id[i] << endl;
	}

	for (int i = 0; i < P_num; i++)
	{
		cloud_resort->points[i].x = cloud_Harris->points[p_id[i]].x;
		cloud_resort->points[i].y = cloud_Harris->points[p_id[i]].y;
		cloud_resort->points[i].z = cloud_Harris->points[p_id[i]].z;
	}


	
	cout << "cloud_Harris" << endl;
	for (int i = 0; i < cloud_Harris->points.size(); i++)
	{
		
		cout << i << "\t" << cloud_Harris->points[i].x << endl;
		cout << i << "\t" << cloud_Harris->points[i].y << endl;
		cout << i << "\t" << cloud_Harris->points[i].z << endl;
	}


	cout << "cloud_resort" << endl;
	for (int i = 0; i < cloud_resort->points.size(); i++)
	{
		
		cout << i << "\t" << cloud_resort->points[i].x ;
		cout << i << "\t" << cloud_resort->points[i].y ;
		cout << i << "\t" << cloud_resort->points[i].z << endl;
	}
	
	pcl::io::savePCDFileASCII("cloud_corner.pcd", *cloud_resort);
	*/

	ofstream fout("queti.txt");
	fout << "id \t\t x \t\t y \t\t z \t\t\n";
	for (int i = 0; i < P_num; i++){
		cin >> n;
		fout << n;
		fout << "\t\t";
		fout << cloud_Harris->points[n].x;
		fout << "\t\t";
		fout << cloud_Harris->points[n].y;
		fout << "\t\t";
		fout << cloud_Harris->points[n].z;
		fout << "\n";
	}

	fout.close();



	//特征点云显示窗口
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_cor(new pcl::visualization::PCLVisualizer("角点点云"));
	//viewer_ori->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gray(point_cloud, 128, 138, 135);
	viewer_cor->addPointCloud(point_cloud, gray, "ori");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> red(cloud_Harris2D, 255, 0, 0);
	viewer_cor->addPointCloud(cloud_Harris2D, red, "cor");
	viewer_cor->setBackgroundColor(255, 255, 255);
	viewer_cor->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cor");
	cout << "original cloud visulization finish" << endl;

	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normal, 20, 0.03, "normals");
	while (!viewer_cor->wasStopped())
	{
		viewer_cor->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}

