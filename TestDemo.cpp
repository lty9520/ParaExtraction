/*************************************************************
*                    Load obj file                           * 
* First transform obj file to pcd file so that the file could*
* be visualized in the pcl.Then use VIsualization class to l-*
* oad pcd file to visualize the point cloud file.            *
**************************************************************/
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
				


//红绿蓝 xyz




using namespace std;
using namespace pcl;


int user_data;

void
viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	//设置背景颜色
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	//球体坐标
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	//添加球体
	//viewer.addSphere(o, 0.25, "cube", 0);
	//std::cout << "i only run once" << std::endl;

}

//求数组最大值
double 
max_array(double *array, int n)//array 为int类型的数组的指针 ，n为数组元素个数
{
	double max = array[0];
	for (int i = 0; i< n; i++)
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
void calcLOD1(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int size, double* x,double* y, double* z){
	
	//求宽度t
	#pragma omp parallel for
	for (int i = 0; i < size; i++){

		x[i] = cloud_in->points[i].x;
		//fout << x[i];
		//fout << "\n";
		//fout << flush;
	}

	//fout.close();

	double max_x = 0.0;
	double min_x = 0.0;

	max_x = max_array(x, size);
	min_x = min_array(x, size);

	double t = max_x - min_x;

	//cout << "max_x = " << max_x << endl;
	//cout << "min_x = " << min_x << endl;
	cout << "t = " << t << "\n" << endl;

	//求高度h
#pragma omp parallel for
	for (int m = 0; m < size; m++){

		y[m] = cloud_in->points[m].y;
		//fout << x[i];
		//fout << "\n";
		//fout << flush;
	}

	//fout.close();

	double max_y = 0.0;
	double min_y = 0.0;

	max_y = max_array(y, size);
	min_y = min_array(y, size);

	double h = max_y - min_y;

	//cout << "max_x = " << max_x << endl;
	//cout << "min_x = " << min_x << endl;
	cout << "h = " << h << "\n" << endl;

	//求长度b
#pragma omp parallel for
	for (int n = 0; n < size; n++){

		z[n] = cloud_in->points[n].z;
		//fout << x[i];
		//fout << "\n";
		//fout << flush;
	}

	//fout.close();

	double max_z = 0.0;
	double min_z = 0.0;

	max_z = max_array(z, size);
	min_z = min_array(z, size);

	double b = max_z - min_z;

	//cout << "max_x = " << max_x << endl;
	//cout << "min_x = " << min_x << endl;
	cout << "b = " << b << "\n" << endl;
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



void cutPointCloud(int n, double max_x, double min_x, double* y, double* z, double* x, 
					pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int size,
					pcl::PointCloud<pcl::Boundary>& boundaries, 
					double res_cloud,
					pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_boundaries)
{

	double pitch = (max_x - min_x) / n;
	double * x_pitch = new  double[n];
	double * y_cuted = new double[size];
	double * z_cuted = new double[size];

	x_pitch[0] = min_x;
	#pragma omp parallel for
	for (int b = 1; b < n + 2; b++){
		x_pitch[b] = x_pitch[b - 1] + pitch;
		cout << "x_pitch[" << b << "] = " << x_pitch[b - 1] << endl;
	}

	double * x_cuted = new double[n];

	

	
	#pragma omp parallel for
	for (int i = 0; i < n + 1; i++){
		x_cuted[i] = x_pitch[i] + 0.5 * pitch;
	}
	#pragma omp parallel for
	for (int k = 1; k < 2; k++){
		cout << "k =" << k << endl;
		int u = 0;
	#pragma omp parallel for
		for (int v = 0; v < size; v++){

			if (x[v] >= x_pitch[k] && x[v] <= x_pitch[k + 1]){
				y_cuted[u] = y[v];
				z_cuted[u] = z[v];
				u++;
			}
			continue;

		}



		cout << "number:" << u << endl;


		cloud_in->width = u;
		cloud_in->height = 1;
		cloud_in->points.resize(cloud_in->width * cloud_in->height);
		cloud_in->is_dense = true;
		#pragma omp parallel for
		for (int i = 0; i < u; i++){

			cloud_in->points[i].x = x_cuted[k];
			cloud_in->points[i].y = y_cuted[i];
			cloud_in->points[i].z = z_cuted[i];



		}
		cloud_in->width = u;
		cloud_in->points.resize(cloud_in->width * cloud_in->height);

		cout << "point cloud_cuted has:" << cloud_in->points.size() << "data points" << endl;

		res_cloud = computeCloudResolution(cloud_in);
		std::cout << "cloud resolution = " << res_cloud << endl;

		boundaryEstimation(boundaries, cloud_in, res_cloud, cloud_boundaries);
	}

}


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
	//cloud->push_back(PointXYZ(0,0,0));
	
	//创建mesh对象
	pcl::PolygonMesh mesh;
	//读取polygon文件，obj格式读取为mesh
	pcl::io::loadPolygonFile("zhengti-dy.obj", mesh);
	
	//初始化结果存储点云final
	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
	//将mesh格式转换为PointCloud格式 方便读取
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	//转存为可读取的PCD文件格式
	pcl::io::savePCDFileASCII("zhengti-dy.pcd", *cloud);
	
	//可输出点的数量
	cout << cloud->size() << endl;
	cout << "OK!";

	
	//加载点云文件
	pcl::io::loadPCDFile("zhengti-dy.pcd", *cloud);
	//std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;
    std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl; 
	

	//求点云size大小
	int size = cloud->points.size();
	double* x = new double[size];
	double* y = new double[size];
	double* z = new double[size];

	//calcLOD1(cloud, size, x, y, z);

	//对x轴方向切片	
	#pragma omp parallel for
	for (int i = 0; i < size; i++){

		x[i] = cloud->points[i].x;
		y[i] = cloud->points[i].y;
		z[i] = cloud->points[i].z;
		//fout << x[i];
		//fout << "\n";
		//fout << flush;
	}
	//求x的最大值
	double max_x = max_array(x, size);
	//求x的最小值
	double min_x = min_array(x, size);
	//求y的最大值
	double max_y = max_array(y, size);
	//求y的最小值
	double min_y = min_array(y, size);
	//求z的最大值
	double max_z = max_array(z, size);
	//求z的最小值
	double min_z = min_array(z, size);
	//初始化边界对象
	pcl::PointCloud<pcl::Boundary> boundaries;
	//初始化切片点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted(new pcl::PointCloud <pcl::PointXYZ>);
	//初始化存储边界对象的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundaries(new pcl::PointCloud <pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
	//片数
	int n = 2;
	//点云密度
	double res_cloud = 0.0;
	//x轴点云切片
	//cutPointCloud(n, max_x, min_x, y, z, x, cloud_cuted, size, boundaries, res_cloud, cloud_boundaries);
	//y轴点云切片
	//cutPointCloud(n, max_y, min_y, z, x, y, cloud_cuted, size, boundaries, res_cloud, cloud_boundaries);
	//z轴点云切片
	//cutPointCloud(n, max_z, min_z, x, y, z, cloud_cuted, size, boundaries, res_cloud, cloud_boundaries);


	//计算法线
	//normalEstimation(cloud_normal, cloud_boundaries);
	/*
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>nor;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	nor.setInputCloud(cloud_boundaries);
	nor.setNumberOfThreads(50);
	nor.setRadiusSearch(0.01);
	nor.setSearchMethod(tree);
	nor.compute(*cloud_normal);
	*/
	res_cloud = computeCloudResolution(cloud);
	boundaryEstimation(boundaries, cloud, res_cloud, cloud_boundaries);

	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_trans->width = cloud->points.size();
	cloud_trans->height = 1;
	cloud_trans->points.resize(cloud_trans->width * cloud_trans->height);
	cloud_trans->is_dense = true;
	#pragma omp parallel for
	for (int i = 0; i < cloud->points.size(); i++){

		cloud_trans->points[i].x = 0;
		cloud_trans->points[i].y = cloud->points[i].y;
		cloud_trans->points[i].z = cloud->points[i].z;



	}
	cloud_trans->width = cloud->points.size();
	cloud_trans->points.resize(cloud_trans->width * cloud_trans->height);
	*/	
	//pcl::io::savePCDFileASCII("zhengti_bd.pcd", *cloud_boundaries); //将点云保存到PCD文件中
	//pcl::io::savePCDFileASCII("normalzhuzi.pcd", *cloud_normal); //将点云保存到PCD文件中
	//创建viewer对象
	//原始点云显示窗口
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ori(new pcl::visualization::PCLVisualizer("原始点云"));
	//viewer_ori->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ori_color_handler(cloud, 255, 0, 0);
	viewer_ori->addPointCloud(cloud, ori_color_handler, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> boundaryoints_color_handler(cloud_boundaries, 0, 255, 0);
	viewer_ori->addPointCloud(cloud_boundaries, boundaryoints_color_handler, "boundary");
	viewer_ori->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "boundary");
	viewer_ori->addCoordinateSystem(1);
	
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
		
		viewer_ori->spinOnce(1);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	int id = 0;

	/*
	ofstream fout("point.txt");
	fout << "id \t x \t y \t z \t";
	fout << "\n";
	for (int i = 0; i < 20; i++)
	{
		cin >> id;
		fout << id;
		fout << "\t";
		fout << cloud_boundaries->points[id].x;
		fout << "\t";
		fout << cloud_boundaries->points[id].y;
		fout << "\t";
		fout << cloud_boundaries->points[id].z;
		fout << "\n";
	}
	fout.close();

*/
	//切片点云显示窗口
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_cuted(new pcl::visualization::PCLVisualizer("切片"));
	viewer_cuted->addPointCloud<pcl::PointXYZ>(cloud_cuted, "cut");

	cout << "cuted cloud visulization finished" << endl;

	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normal, 20, 0.03, "normals");
	while (!viewer_cuted->wasStopped())
	{
		viewer_cuted->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	//边界投影点云显示窗口
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_proj(new pcl::visualization::PCLVisualizer("边界投影"));
	viewer_proj->addPointCloud<pcl::PointXYZ>(cloud_boundaries, "projection");

	cout << "projection visulization finished" << endl;

	
	viewer_proj->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_boundaries, cloud_normal, 20, 0.03, "normals");
	while (!viewer_proj->wasStopped())
	{
		viewer_proj->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	DWORD end_time = GetTickCount(); //结束计时
	cout << "The run time is:" << (end_time - start_time) << "ms!" << endl; //输出时间
	
	cin.get();
	return 0;
}