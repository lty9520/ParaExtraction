#pragma once
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>		//PCL的PCD格式文件的输入输出头文件
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/sample_consensus/method_types.h>   //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割的类的头文件
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>			//边界提取头文件	
#include <vector>
#include <stdio.h>  
#include <math.h>  
#include <stdlib.h> 
#include <cmath>



//弧段需要点个数
#define PATCH_NUM 3
#define FIND_THRES 0.1
#define FIND_POINT_NUM PATCH_NUM + 2
#define ARC_NUM 6
#define MAXN 3274

//红绿蓝 xyz
using namespace std;
using namespace pcl;


struct Point
{
	float x, y;
};
//叉积
float multiply(Point p1, Point p2, Point p0)
{
	return((p1.x - p0.x)*(p2.y - p0.y) - (p2.
		x - p0.x)*(p1.y - p0.y));
}
float dis(Point p1, Point p2)
{
	return(sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y)));
}

//************************************/
// Method:    Graham_scan
// FullName:  Graham_scan
// Access:    public 
// Author:    
// Explain:   calculate the convecx hull of the input array
// Returns:   void
// Qualifier:
// Parameter: Point PointSet[]	:input data array
// Parameter: Point ch[]		:convex hull result array
// Parameter: int n				:input data number
// Parameter: float & len		:point number of result convex hull 
//************************************/
void Graham_scan(Point PointSet[], Point ch[], int n, float &len)
{
	int i, j, k = 0, top = 2;
	Point tmp;
	//find start point(y minest point)
	for (i = 1; i < n; i++)
		if ((PointSet[i].y < PointSet[k].y) || ((PointSet[i].y == PointSet[k].y) && (PointSet[i].x < PointSet[k].x)))
			k = i;
	//cout << "min" << k << endl;
	//make start point P[0]
	tmp = PointSet[0];
	PointSet[0] = PointSet[k];
	PointSet[k] = tmp;

	for (i = 1; i < n - 1; i++)
	{
		k = i;
		for (j = i + 1; j < n; j++)
			if ((multiply(PointSet[j], PointSet[k], PointSet[0]) > 0)
				|| ((multiply(PointSet[j], PointSet[k], PointSet[0]) == 0)
					&& (dis(PointSet[0], PointSet[j]) < dis(PointSet[0], PointSet[k]))))
				k = j;
		tmp = PointSet[i];
		PointSet[i] = PointSet[k];
		PointSet[k] = tmp;
	}
	//give iniatial data to result array ch[]
	ch[0] = PointSet[0];
	ch[1] = PointSet[1];
	ch[2] = PointSet[2];
	for (i = 3; i < n; i++)
	{
		while (multiply(PointSet[i], ch[top], ch[top - 1]) >= 0)
			top--;
		ch[++top] = PointSet[i];
	}
	len = top + 1;
}



//求最大值(Z轴方向）(Y方向作为判断阈值）
void 
getMaxZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
		vector<double> &tiqu_axis)
{
	double result = 0.0;
	for (int n = 0; n < PATCH_NUM; n++)
	{
		vector<double> temp_z;
		for (int i = 0; i < cloud->points.size(); i++)
		{

			if (abs(cloud->points[i].y - tiqu_axis[n]) < 1)
				temp_z.push_back(abs(cloud->points[i].z));


		}
		vector<double>::iterator maxPosition = max_element(temp_z.begin(), temp_z.end());

		cout << "p[" << n << "].z = " << *maxPosition << endl;
		result = *maxPosition;
		//cout << "result = " << result << endl;
		//return result;
	}
	
}

//求最大值(Y轴方向）(Z方向作为判断阈值）
void
getMaxY(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
		vector<double> &tiqu_axis)
{
	double result = 0.0;
	for (int n = 0; n < PATCH_NUM; n++)
	{
		vector<double> temp_y;
		for (int i = 0; i < cloud->points.size(); i++)
		{

			if (abs(cloud->points[i].z - tiqu_axis[n]) < 1)
				temp_y.push_back(abs(cloud->points[i].y));


		}
		vector<double>::iterator maxPosition = max_element(temp_y.begin(), temp_y.end());

		cout << "p[" << n << "].y = " << *maxPosition << endl;
		result = *maxPosition;
		//return result;
	}
}


//求拟合点
void
fitPointCalc(double dy, 
			double dz, 
			double patch, 
			pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
			int p1, 
			int p2, 
			vector<int> &tiqu)
{

	vector<double> tiqu_x;
	vector<double> tiqu_y;
	vector<double> tiqu_z;
	
	int n = 1;
	if (dy > dz)
	{
		cout << ">" << endl;
		patch = dy / (PATCH_NUM + 1);
		cout << "patch = " << patch << endl;
		for (n; n < PATCH_NUM + 1; n++)
		{
			//cout << n << endl;
			
			std::vector<double> temp;
			//cout << "size = " << temp.size() << endl;
			std::vector<int> tempId;
			for (int i = 0; i < cloud->points.size(); i++)
			{


				if (//abs(cloud->points[p1].x - cloud->points[i].x) < 0.01
					abs((cloud->points[p1].y - patch * n) - cloud->points[i].y) < FIND_THRES)
					//&& abs(cloud->points[p1].z - cloud->points[i].z) < 0.01)
				{
					//cout << "11" << endl;
					double t = cloud->points[i].y - (cloud->points[p1].y - patch * n);
					//cout << "t = " << t << endl;
					temp.push_back(t);
					tempId.push_back(i);
					//tiqu[n] = i;
					//cout << t << endl;
					//cout << i << "x = " << cloud->points[i].x << endl;
					//cout << i << "y = " << cloud->points[i].y << endl;
					//cout << i << "z = " << cloud->points[i].z << endl;
				}
				//cout << "22" << endl;


			}

			int num = 0;
			double temp_min = temp[0];
			int temp_id = 0;
			vector<double>::iterator it;
			for (int t = 0; t < temp.size(); t++)
			{
				//cout << "temp = " << temp[i] << endl;
				//cout << "tempId = " << tempId[i] << endl;

				if (abs(temp[t]) <= abs(temp_min))
				{
					temp_min = temp[t];
				}



			}

			for (int t = 0; t < temp.size(); t++)
			{
				//cout << "temp = " << temp[i] << endl;
				//cout << "tempId = " << tempId[i] << endl;
				if (abs(temp[t]) == abs(temp_min))
				{
					temp_id = t;
				}



			}
			num = temp_id;


			//cout << num << endl;
			tiqu.push_back(tempId[num]);
			//cout << "33" << endl;
		}
		for (int i = 0; i < PATCH_NUM; i++)
		{
			tiqu_x.push_back(cloud->points[tiqu[i]].x);
			tiqu_y.push_back(cloud->points[tiqu[i]].y);
			tiqu_z.push_back(cloud->points[tiqu[i]].z);
		}
		getMaxZ(cloud, tiqu_y);
	}
	else
	{
		cout << "<" << endl;
		patch = dz / (PATCH_NUM + 1);
		cout << "patch = " << patch << endl;
		for (n; n < PATCH_NUM + 1; n++)
		{

			//cout << n << endl;
			//cout << "!!!" << endl;
			std::vector<double> temp;
			//cout << "size = " << temp.size() << endl;
			std::vector<int> tempId;
			for (int i = 0; i < cloud->points.size(); i++)
			{


				if (//abs(cloud->points[p1].x - cloud->points[i].x) < 0.01
					//abs((cloud->points[p1].y - patch * n) - cloud->points[i].y) < 0.1)
					abs((cloud->points[p1].z + patch * n) - cloud->points[i].z) < FIND_THRES)
				{
					//cout << "11" << endl;
					double t = cloud->points[i].z - (cloud->points[p1].z - patch * n);
					//cout << "t = " << t << endl;
					temp.push_back(t);
					tempId.push_back(i);
					//tiqu[n] = i;
					//cout << t << endl;
					//cout << i << "x = " << cloud->points[i].x << endl;
					//cout << i << "y = " << cloud->points[i].y << endl;
					//cout << i << "z = " << cloud->points[i].z << endl;
				}
				//


			}
			
			
			int num = 0;
			
			double temp_min = temp[0];
			int temp_id = 0;
			vector<double>::iterator it;
			for (int t = 0; t < temp.size(); t++)
			{
				//cout << "temp = " << temp[i] << endl;
				//cout << "tempId = " << tempId[i] << endl;

				//cout << "22" << endl;
				if (abs(temp[t]) <= abs(temp_min))
				{
					temp_min = temp[t];
				}



			}

			for (int t = 0; t < temp.size(); t++)
			{
				//cout << "temp = " << temp[i] << endl;
				//cout << "tempId = " << tempId[i] << endl;
				if (abs(temp[t]) == abs(temp_min))
				{
					temp_id = t;
				}



			}
			num = temp_id;


			//cout << num << endl;
			tiqu.push_back(tempId[num]);
			//cout << "33" << endl;
		}
		for (int i = 0; i < PATCH_NUM; i++)
		{
			tiqu_x.push_back(cloud->points[tiqu[i]].x);
			tiqu_y.push_back(cloud->points[tiqu[i]].y);
			tiqu_z.push_back(cloud->points[tiqu[i]].z);
		}
		getMaxY(cloud, tiqu_z);
	}

	for (int i = 0; i < PATCH_NUM; i++)
	{
		cout << "tiqu[" << i << "] = " << tiqu[i] << endl;
	}

}


//匹配提取点到点云中
void
findPointInCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& keyPoint, 
				pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
				vector<int> &pid)
{
	
	int j = 0;
	for (j; j < ARC_NUM; j++)
	{
		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (abs(keyPoint->points[j].x - cloud->points[i].x) < FIND_THRES &&
				abs(keyPoint->points[j].y - cloud->points[i].y) < FIND_THRES &&
				abs(keyPoint->points[j].z - cloud->points[i].z) < FIND_THRES)
			{
				pid.push_back(i);
				
			}

		}
		
	}

	for (int i = 0; i < ARC_NUM; i++)
	{
		cout << "pid[" << i << "] = " << pid[i] << endl;
	}


	
}




//求数组最小值
double
min_array(float *array, 
		int n)//array 为int类型的数组的指针 ，n为数组元素个数
{
	double min = array[0];
	for (int i = 0; i < n; i++)
	{
		if (min > array[i])
		{
			min = array[i];
		}
	}
	return min;
}

//求数组最大值
double
max_array(float *array, 
		int n)//array 为int类型的数组的指针 ，n为数组元素个数
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

int main()
{

	/*
	//获取当前路径
	TCHAR path[100];
	GetCurrentDirectory(MAX_PATH, path);
	cout << path << "\n" << endl;

	string a;
	a.assign(path);

	a.append("\\0606.obj");

	cout << a << "\n" << endl;
	*/

	DWORD start_time = GetTickCount(); //开始计时
	//初始化PointCloud对象
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pushback
	cloud->push_back(PointXYZ(0, 0, 0));
	/*
	//创建mesh对象
	pcl::PolygonMesh mesh;
	//读取polygon文件，obj格式读取为mesh
	pcl::io::loadPolygonFile("bdl zt 2 wc.obj", mesh);

	//初始化结果存储点云final
	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
	//将mesh格式转换为PointCloud格式 方便读取
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	//转存为可读取的PCD文件格式
	pcl::io::savePCDFileASCII("bdl zt 2 wc.pcd", *cloud);

	//可输出点的数量
	cout << cloud->size() << endl;
	cout << "OK!";
	*/

	//加载点云文件
	pcl::io::loadPCDFile("zhengti-dy.pcd", *cloud);
	//std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;
	std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cross_point(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tubao(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr fenduan_point(new pcl::PointCloud<pcl::PointXYZ>);

	cross_point->width = 35;
	cross_point->height = 1;
	cross_point->points.resize(cross_point->width * cross_point->height);
	cross_point->is_dense = true;

	cloud_tubao->width = MAXN;
	cloud_tubao->height = 1;
	cloud_tubao->points.resize(cloud_tubao->width * cloud_tubao->height);
	cloud_tubao->is_dense = true;

	//fenduan_point->width = 40;
	//fenduan_point->height = 1;
	//fenduan_point->points.resize(fenduan_point->width * fenduan_point->height);
	//fenduan_point->is_dense = true;

	//for (int i = 0; i < cloud->points.size(); i++)
	//{
	//	cloud->points[i].x = -61.365341;
	//}

	

	pcl::io::loadPCDFile("cloud_corner.pcd", *fenduan_point);
	
	//pcl::io::savePCDFileASCII("quetidian33.pcd", *cloud);
	//cout << "1" << endl;
/*	
	#pragma omp parallel for
	for (int i = 0; i < cross_point->points.size(); i++){

	cross_point->points[i].x = x_cuted[k];
	cross_point->points[i].y = y_cuted[i];
	cross_point->points[i].z = z_cuted[i];



	}
*/	
	//1X2
	cross_point->points[0].x = -36.2466;	//0.464723
	cross_point->points[0].y = 937.78;
	cross_point->points[0].z = 125.975;	// 0.0817564
	
	//1X2qian
	cross_point->points[1].x = -56.6656;
	cross_point->points[1].y = 880.742;
	cross_point->points[1].z = 126.95;
	
	//6X7
	cross_point->points[2].x = -61.3238;
	cross_point->points[2].y = 776.543;
	cross_point->points[2].z = 163.753;
	//8X9
	cross_point->points[3].x = -63.8391;
	cross_point->points[3].y = 733.221;
	cross_point->points[3].z = 186.147;
	//6X13
	cross_point->points[4].x = -35.2599;
	cross_point->points[4].y = 658.131;
	cross_point->points[4].z = 214.418;
	//9X14
	cross_point->points[5].x = -32.0161;
	cross_point->points[5].y = 648.211;
	cross_point->points[5].z = 265.67;
	//12X15

	cross_point->points[6].x = -56.6656;
	cross_point->points[6].y = 859.902;
	cross_point->points[6].z = 138.228;	//134.3106
	//4X5
	cross_point->points[7].x = -56.6656;	//0.464723
	cross_point->points[7].y = 839.062;
	cross_point->points[7].z = 147.678;
	//4X5qian
	cross_point->points[8].x = -56.6656;
	cross_point->points[8].y = 818.222;
	cross_point->points[8].z = 157.128;
	//11X15
	cross_point->points[9].x = -56.6656;
	cross_point->points[9].y = 797.382;
	cross_point->points[9].z = 162.823;

	cross_point->points[10].x = -56.6656;
	cross_point->points[10].y = 926.372;
	cross_point->points[10].z = 125.53; 
	//4X5
	cross_point->points[11].x = -56.6656;
	cross_point->points[11].y = 914.963;
	cross_point->points[11].z = 123.23;
	//4X5qian
	cross_point->points[12].x = -56.6656;
	cross_point->points[12].y = 903.556;
	cross_point->points[12].z = 122.361;
	//11X15
	cross_point->points[13].x = -56.6656;
	cross_point->points[13].y = 892.15;
	cross_point->points[13].z = 124.59;

	cross_point->points[14].x = -56.6656;
	cross_point->points[14].y = 767.879;
	cross_point->points[14].z = 172.569;
	//4X5
	cross_point->points[15].x = -56.6656;
	cross_point->points[15].y = 759.214;
	cross_point->points[15].z = 180.063;
	//4X5qian
	cross_point->points[16].x = -56.6656;
	cross_point->points[16].y = 750.55;
	cross_point->points[16].z = 184.165;
	//11X15
	cross_point->points[17].x = -56.6656;
	cross_point->points[17].y = 741.885;
	cross_point->points[17].z = 186.899;

	cross_point->points[18].x = -56.6656;
	cross_point->points[18].y = 718.203;
	cross_point->points[18].z = 200.463;
	//4X5
	cross_point->points[19].x = -56.6656;
	cross_point->points[19].y = 703.184;
	cross_point->points[19].z = 208.081;
	//4X5qian			
	cross_point->points[20].x = -56.6656;
	cross_point->points[20].y = 688.167;
	cross_point->points[20].z = 211.753;
	//11X15
	cross_point->points[21].x = -56.6656;
	cross_point->points[21].y = 673.146;
	cross_point->points[21].z = 211.376;

	cross_point->points[22].x = -56.6656;
	cross_point->points[22].y = 660.117;
	cross_point->points[22].z = 224.662;
	//4X5
	cross_point->points[23].x = -56.6656;
	cross_point->points[23].y = 661.633;
	cross_point->points[23].z = 234.911;
	//4X5qian			
	cross_point->points[24].x = -56.6656;
	cross_point->points[24].y = 660.282;
	cross_point->points[24].z = 245.16;
	//11X15
	cross_point->points[25].x = -56.6656;
	cross_point->points[25].y = 653.568;
	cross_point->points[25].z = 255.411;
	/*
	cross_point->points[10].x = -275.384;
	cross_point->points[10].y = 517.489;
	cross_point->points[10].z = 184.448;

	cross_point->points[11].x = -450.719;
	cross_point->points[11].y = 580;
	cross_point->points[11].z = 8352.5;

	cross_point->points[12].x = -450.75;
	cross_point->points[12].y = 1795.63;
	cross_point->points[12].z = -9871;

	cross_point->points[13].x = -378.654;
	cross_point->points[13].y = 3029.24;
	cross_point->points[13].z = -40.1604;*/

	cout << "2" << endl;


	cross_point->width = 35;
	cross_point->points.resize(cross_point->width * cross_point->height);

	
	cout << "3" << endl;
	/*
	cloud->width = cloud->points.size() + 1;
	cloud->width = 1;
	cloud->points.resize(cloud->width * cloud->width);
	cloud->is_dense = true;
	cloud->points[cloud->points.size() + 1].x = 0.471670;
	cloud->points[cloud->points.size() + 1].y = 0.561969;
	cloud->points[cloud->points.size() + 1].z = 0.0817564;
*/
	/*
	int size = cloud->points.size();
	float* x = new float[size];

	#pragma omp parallel for
	for (int i = 0; i < cloud->points.size(); i++){

		x[i] = cloud->points[i].x;
		//fout << x[i];
		//fout << "\n";
		//fout << flush;
	}

	double max_x = 0.0;
	double min_x = 0.0;

	max_x = max_array(x, size);
	min_x = min_array(x, size);

	cout << "min_x = " << min_x;
	cout << "max_x = " << max_x;

*/



	//求解凸包
	/*
	Point PointSet[MAXN];
	Point ch[MAXN];
	float len;
	int i;
	int sumx, sumy;
	float  zhongx, zhongy;
	for (int i = 0; i < MAXN; i++)
	{
		PointSet[i].x = cloud->points[i].y;
		PointSet[i].y = cloud->points[i].z;
	}
	Graham_scan(PointSet, ch, MAXN, len);
	for (int i = 0; i < len; i++)
	{
		cloud_tubao->points[i].y = ch[i].x;
		cloud_tubao->points[i].z = ch[i].y;
	}

	cout << len << endl;
	

	cout << "4" << endl;
*/

	//原始点云显示窗口
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ori(new pcl::visualization::PCLVisualizer("原始点云"));
	//viewer_ori->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud, 0, 255, 0);
	viewer_ori->addPointCloud(cloud, red, "cloud");
	viewer_ori->addPointCloud(cloud_tubao, green, "cross");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> boundaryoints_color_handler(cloud_boundaries, 0, 255, 0);
	//viewer_ori->addPointCloud(cloud_boundaries, boundaryoints_color_handler, "boundary");
	viewer_ori->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cross");
	viewer_ori->addCoordinateSystem(1);
	
	/*
	cloud_tubao->width = len;
	cloud_tubao->height = 1;
	cloud_tubao->points.resize(cloud_tubao->width * cloud_tubao->height);
	cloud_tubao->is_dense = true;
*/
	cout << "5" << endl;
	/*
	for (int i = 0; i < cloud_tubao->points.size(); i++)
	{
		viewer_ori->addText3D(to_string(i), cloud_tubao->points[i], 10.0, 0.0, 0.0, 1.0);
	}*/
	//cout << "6" << endl;

		//int j = 0;
	/*
	double temp1 = 0;
	temp1 = cross_point->points[0].x - cloud->points[257816].x;
	cout << "x = " << temp1 << endl;
	double temp2 = 0;
	temp2 = cross_point->points[0].y - cloud->points[257816].y;
	cout << "y = " << temp2 << endl;
	double temp3 = 0;
	temp3 = cross_point->points[0].z - cloud->points[257816].z;
	cout << "z = " << temp3 << endl;



	for (j; j < 6; j++)
	{
		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (   abs(cross_point->points[j].x - cloud->points[i].x) < 0.01
				&& abs(cross_point->points[j].y - cloud->points[i].y) < 0.01
				&& abs(cross_point->points[j].z - cloud->points[i].z) < 0.01)
			{
				pid[j] = i;
			}

		}
	}
*/
	//*****************
	//*****************
	//int pid[FIND_POINT_NUM];
	vector<int> pid;

	//findPointInCloud(cross_point, cloud, pid);
	
	
	cout << "6" << endl;


	
/*
	double dy[3];
	double dz[3];
	
	//123
	for (int i = 1; i < 3; i++)
	{
		dy[i] = cloud->points[pid[i]].y - cloud->points[pid[i + 1]].y;
		dz[i] = cloud->points[pid[i]].z - cloud->points[pid[i + 1]].z;
	}


	double dy_12 = cloud->points[pid[0]].y - cloud->points[pid[1]].y;
	double dz_12 = cloud->points[pid[0]].z - cloud->points[pid[1]].z;
*/	

	int n = 1;

	//cout << "*****15184:" << cloud->points[15184].x << "\t" << cloud->points[15184].y << "\t" << cloud->points[15184].z << endl;
	//cout << "*****15128:" << cloud->points[15128].x << "\t" << cloud->points[15128].y << "\t" << cloud->points[15128].z << endl;
	//cout << "*****234615:" << cloud->points[234615].x << "\t" << cloud->points[234615].y << "\t" << cloud->points[234615].z << endl;
	//cout << "*****204068:" << cloud->points[204068].x << "\t" << cloud->points[204068].y << "\t" << cloud->points[204068].z << endl;
	//cout << "*****171547:" << cloud->points[171547].x << "\t" << cloud->points[171547].y << "\t" << cloud->points[171547].z << endl;



	/*
	cout << "arc1" << endl;
	cout << "*****253325:" << cloud->points[253325].x << "\t" << cloud->points[253325].y << "\t" << cloud->points[253325].z << endl;
	cout << "*****201879:" << cloud->points[201879].x << "\t" << cloud->points[201879].y << "\t" << cloud->points[201879].z << endl;
	cout << "*****201767:" << cloud->points[201767].x << "\t" << cloud->points[201767].y << "\t" << cloud->points[201767].z << endl;
	cout << "*****215353:" << cloud->points[215353].x << "\t" << cloud->points[215353].y << "\t" << cloud->points[215353].z << endl;
	cout << "arc3" << endl;
	cout << "*****219477:" << cloud->points[219477].x << "\t" << cloud->points[219477].y << "\t" << cloud->points[219477].z << endl;
	cout << "*****206463:" << cloud->points[206463].x << "\t" << cloud->points[206463].y << "\t" << cloud->points[206463].z << endl;
	cout << "*****221540:" << cloud->points[221540].x << "\t" << cloud->points[221540].y << "\t" << cloud->points[221540].z << endl;
	cout << "*****202428:" << cloud->points[202428].x << "\t" << cloud->points[202428].y << "\t" << cloud->points[202428].z << endl;
	cout << "arc4" << endl;
	cout << "*****169679:" << cloud->points[169679].x << "\t" << cloud->points[169679].y << "\t" << cloud->points[169679].z << endl;
	cout << "*****99850:" << cloud->points[99850].x << "\t" << cloud->points[99850].y << "\t" << cloud->points[99850].z << endl;
	cout << "*****176202:" << cloud->points[176202].x << "\t" << cloud->points[176202].y << "\t" << cloud->points[176202].z << endl;
	cout << "*****128922:" << cloud->points[128922].x << "\t" << cloud->points[128922].y << "\t" << cloud->points[128922].z << endl;
	cout << "arc5" << endl;
	cout << "*****41234:" << cloud->points[41234].x << "\t" << cloud->points[41234].y << "\t" << cloud->points[41234].z << endl;
	cout << "*****41165:" << cloud->points[41165].x << "\t" << cloud->points[41165].y << "\t" << cloud->points[41165].z << endl;
	cout << "*****131856:" << cloud->points[131856].x << "\t" << cloud->points[131856].y << "\t" << cloud->points[131856].z << endl;
	cout << "*****133367:" << cloud->points[133367].x << "\t" << cloud->points[133367].y << "\t" << cloud->points[133367].z << endl;
	cout << "4 5" << endl;
	cout << "*****4:" << cross_point->points[4].x << "\t" << cross_point->points[4].y << "\t" << cross_point->points[4].z << endl;
	cout << "*****5:" << cross_point->points[5].x << "\t" << cross_point->points[5].y << "\t" << cross_point->points[5].z << endl;
	*/




	//int p1 = pid[1];
	//int p2 = pid[2];
	//cout << "p1 = " << p1 <<endl;
	//cout << "p2 = " << p2 <<endl;
	//
	//double dy = cloud->points[p1].y - cloud->points[p2].y;
	//double dz = cloud->points[p1].z - cloud->points[p2].z;
	int p1 = 0;
	int p2 = 0;
	double dy = 0.0;
	double dz = 0.0;

	//cout << "p1y = " << cloud->points[p1].y << endl;
	//cout << "p2y = " << cloud->points[p2].y << endl;
	//cout << "p1z = " << cloud->points[p1].z << endl;
	//cout << "p2z = " << cloud->points[p2].z << endl;
	
	//在拐点间等分求点
	/*
	for (int i = 0; i < ARC_NUM - 1; i++)
	{
		cout << "*****find the fit point in No." << i + 1 << "arc.*****" << endl;
		double patch = 0.0;
		vector<int> tiqu;
		p1 = pid[i];
		p2 = pid[i + 1];

		//cout << "p1 = " << p1 << endl;
		//cout << "p2 = " << p2 << endl;
		//cout << "function test start" << endl;

		dy = abs(cloud->points[p1].y - cloud->points[p2].y);
		dz = abs(cloud->points[p1].z - cloud->points[p2].z);


		fitPointCalc(dy, dz, patch, cloud, p1, p2, tiqu);

		
			//for (int i = 0; i < PATCH_NUM; i++)
			//{
			//cout << "tiqu[" << i << "] = " << tiqu[i] << endl;
			//}
			

		//cout << "function test finish" << endl;
		/*
			if (dy > dz)
			{
			cout << ">" << endl;
			patch = dy / (PATCH_NUM + 1);
			cout << "patch = " << patch << endl;
			for (n; n < PATCH_NUM + 1; n++)
			{
			cout << n << endl;

			std::vector<double> temp;
			cout << "size = " << temp.size() << endl;
			std::vector<int> tempId;
			for (int i = 0; i < cloud->points.size(); i++)
			{


			if (//abs(cloud->points[p1].x - cloud->points[i].x) < 0.01
			abs((cloud->points[p1].y - patch * n) - cloud->points[i].y) < 0.1)
			//&& abs(cloud->points[p1].z - cloud->points[i].z) < 0.01)
			{
			//cout << "11" << endl;
			double t = cloud->points[i].y - (cloud->points[p1].y - patch * n);
			//cout << "t = " << t << endl;
			temp.push_back(t);
			tempId.push_back(i);
			//tiqu[n] = i;
			//cout << t << endl;
			//cout << i << "x = " << cloud->points[i].x << endl;
			//cout << i << "y = " << cloud->points[i].y << endl;
			//cout << i << "z = " << cloud->points[i].z << endl;
			}
			//cout << "22" << endl;


			}

			int num = 0;
			double temp_min = temp[0];
			int temp_id = 0;
			vector<double>::iterator it;
			for (int t = 0; t < temp.size(); t++)
			{
			//cout << "temp = " << temp[i] << endl;
			//cout << "tempId = " << tempId[i] << endl;

			if (abs(temp[t]) <= abs(temp_min))
			{
			temp_min = temp[t];
			}



			}

			for (int t = 0; t < temp.size(); t++)
			{
			//cout << "temp = " << temp[i] << endl;
			//cout << "tempId = " << tempId[i] << endl;
			if (abs(temp[t]) == abs(temp_min))
			{
			temp_id = t;
			}



			}
			num = temp_id;


			//cout << num << endl;
			tiqu[n] = tempId[num];
			//cout << "33" << endl;
			}
			}
			else
			{
			cout << "<" << endl;
			patch = dz / (PATCH_NUM + 1);
			cout << "patch = " << patch << endl;
			for (n; n < PATCH_NUM + 1; n++)
			{
			cout << n << endl;

			std::vector<double> temp;
			cout << "size = " << temp.size() << endl;
			std::vector<int> tempId;
			for (int i = 0; i < cloud->points.size(); i++)
			{


			if (//abs(cloud->points[p1].x - cloud->points[i].x) < 0.01
			//abs((cloud->points[p1].y - patch * n) - cloud->points[i].y) < 0.1)
			abs((cloud->points[p1].z - patch * n) - cloud->points[i].z) < 0.01)
			{
			//cout << "11" << endl;
			double t = cloud->points[i].z - (cloud->points[p1].z - patch * n);
			//cout << "t = " << t << endl;
			temp.push_back(t);
			tempId.push_back(i);
			//tiqu[n] = i;
			//cout << t << endl;
			//cout << i << "x = " << cloud->points[i].x << endl;
			//cout << i << "y = " << cloud->points[i].y << endl;
			//cout << i << "z = " << cloud->points[i].z << endl;
			}
			//cout << "22" << endl;


			}

			int num = 0;
			double temp_min = temp[0];
			int temp_id = 0;
			vector<double>::iterator it;
			for (int t = 0; t < temp.size(); t++)
			{
			//cout << "temp = " << temp[i] << endl;
			//cout << "tempId = " << tempId[i] << endl;

			if (abs(temp[t]) <= abs(temp_min))
			{
			temp_min = temp[t];
			}



			}

			for (int t = 0; t < temp.size(); t++)
			{
			//cout << "temp = " << temp[i] << endl;
			//cout << "tempId = " << tempId[i] << endl;
			if (abs(temp[t]) == abs(temp_min))
			{
			temp_id = t;
			}



			}
			num = temp_id;


			//cout << num << endl;
			tiqu[n] = tempId[num];
			//cout << "33" << endl;
			}
			}
			*/

		//double p1_x = cloud->points[tiqu[1]].x;
		//double p1_y = cloud->points[tiqu[1]].y;
		//double p1_z = cloud->points[tiqu[1]].z;
		//double p2_x = cloud->points[tiqu[2]].x;
		//double p2_y = cloud->points[tiqu[2]].y;
		//double p2_z = cloud->points[tiqu[2]].z;
		//double p3_x = cloud->points[tiqu[3]].x;
		//double p3_y = cloud->points[tiqu[3]].y;
		//double p3_z = cloud->points[tiqu[3]].z;
		//double p4_x = cloud->points[tiqu[4]].x;
		//double p4_y = cloud->points[tiqu[4]].y;
		//double p4_z = cloud->points[tiqu[4]].z;





		
			
	//}


	




	//ofstream fout("out.txt");
	/*
	for (int n = 0; n < PATCH_NUM; n++)
	{
		vector<double> temp_z;
		for (int i = 0; i < cloud->points.size(); i++)
		{

			if (abs(cloud->points[i].y - tiqu_y[n]) < 0.01)
				temp_z.push_back(abs(cloud->points[i].z));
			

		}
		vector<double>::iterator maxPosition = max_element(temp_z.begin(), temp_z.end());
		
		cout << "p[" << n << "].z = " << *maxPosition << endl;
	}
	//fout.close();
	*/
	//vector<double>::iterator minPosition = min_element(temp_z.begin(), temp_z.end());
	cout << "**********************************" << endl;
	//cout << "p1_z = " << *maxPosition << " at the postion of " << maxPosition - temp_z.begin() << endl;
	//cout << "p1_z = " << *minPosition << " at the postion of " << minPosition - temp_z.begin() << endl;
	/*
	for (int m = 1; m < 3; m++)
	{
		if (dz[m] > dy[m])
		{
			patch = dz[m] / 4;
			for (; n < 4;)
			{
				for (int i = 0; i < cloud->points.size(); i++)
				{
					if (abs(cloud->points[i].x - cloud->points[pid[m]].x) < 0.001
						&& abs(cloud->points[i].y - cloud->points[pid[m]].y) < 0.001
						&& abs((cloud->points[i].z + patch * n) - (cloud->points[pid[m]].z + patch * n)) < 0.001)
					{
						tiqu[m] = i;
					}
					n++;
				}
			}
		}
		else
		{
			patch = dy[m] / 4;
			for (; n < 4;)
			{
				for (int i = 0; i < cloud->points.size(); i++)
				{
					if (abs(cloud->points[i].x - cloud->points[pid[m]].x) < 0.001
						&& abs((cloud->points[i].y + patch * n) - (cloud->points[pid[m]].y + patch * n)) < 0.001
						&& abs(cloud->points[i].z - cloud->points[pid[m]].z) < 0.001)
					{
						tiqu[m+5] = i;
					}
					n++;
				}
			}
		}
	}
	*/
	/*
	for (int i = 0; i < PATCH_NUM + 1; i++)
	{
		cout << "tiqu[" << i << "] = " << tiqu[i] << endl;
	}
*/
	

	cout << "6" << endl;

	//double point[100] = { 0.0 };
	/*
	#pragma omp parallel for
	for (int i = 0; i < cloud_boundaries->size(); i++){
	cout << i << endl;
	viewer_ori->addText3D(std::to_string(i), cloud_boundaries->points[i], 0.001, 1.0, 1.0, 1.0);

	}
	*/

	cout << "original cloud visulization finish" << endl;

	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normal, 20, 0.03, "normals");
	while (!viewer_ori->wasStopped())
	{

		viewer_ori->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}