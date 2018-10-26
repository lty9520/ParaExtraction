/*************************************************************
*                    Load obj file                           * 
* First transform obj file to pcd file so that the file could*
* be visualized in the pcl.Then use VIsualization class to l-*
* oad pcd file to visualize the point cloud file.            *
**************************************************************/
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>		//PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ����ͷ�ļ���
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/sample_consensus/method_types.h>   //����������Ʒ���ͷ�ļ�
#include <pcl/sample_consensus/model_types.h>   //ģ�Ͷ���ͷ�ļ�
#include <pcl/segmentation/sac_segmentation.h>   //���ڲ���һ���Էָ�����ͷ�ļ�
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>			//�߽���ȡͷ�ļ�
				


//������ xyz




using namespace std;
using namespace pcl;


int user_data;

void
viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	//���ñ�����ɫ
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	//��������
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	//�������
	//viewer.addSphere(o, 0.25, "cube", 0);
	//std::cout << "i only run once" << std::endl;

}

//���������ֵ
double 
max_array(double *array, int n)//array Ϊint���͵������ָ�� ��nΪ����Ԫ�ظ���
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

//��������Сֵ
double
min_array(double *array, int n)//array Ϊint���͵������ָ�� ��nΪ����Ԫ�ظ���
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


//��������ܶȣ����Ƽ�ƽ����ࣩ
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
void calcLOD1(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int size, double* x,double* y, double* z){
	
	//����t
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

	//��߶�h
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

	//�󳤶�b
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
	2.9  ��/2		�ӷ�ǵ�
	7 3.8 ��/2		С���ǵ�
	3 2.5 ��/2		����С���ǵ� ȱһ��

	zuizhong 
	3.5		3		1.png + 2.png
	3.3		3		2.png ��
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
	//��ȡ��ǰ·��
	TCHAR path[100];
	GetCurrentDirectory(MAX_PATH, path);
	cout << path << "\n" << endl;

	string a;
	a.assign(path);

	a.append("\\0606.obj");

	cout << a << "\n" << endl;
*/

	DWORD start_time = GetTickCount(); //��ʼ��ʱ
	//��ʼ��PointCloud����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pushback
	//cloud->push_back(PointXYZ(0,0,0));
	
	//����mesh����
	pcl::PolygonMesh mesh;
	//��ȡpolygon�ļ���obj��ʽ��ȡΪmesh
	pcl::io::loadPolygonFile("zhengti-dy.obj", mesh);
	
	//��ʼ������洢����final
	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
	//��mesh��ʽת��ΪPointCloud��ʽ �����ȡ
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	//ת��Ϊ�ɶ�ȡ��PCD�ļ���ʽ
	pcl::io::savePCDFileASCII("zhengti-dy.pcd", *cloud);
	
	//������������
	cout << cloud->size() << endl;
	cout << "OK!";

	
	//���ص����ļ�
	pcl::io::loadPCDFile("zhengti-dy.pcd", *cloud);
	//std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;
    std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl; 
	

	//�����size��С
	int size = cloud->points.size();
	double* x = new double[size];
	double* y = new double[size];
	double* z = new double[size];

	//calcLOD1(cloud, size, x, y, z);

	//��x�᷽����Ƭ	
	#pragma omp parallel for
	for (int i = 0; i < size; i++){

		x[i] = cloud->points[i].x;
		y[i] = cloud->points[i].y;
		z[i] = cloud->points[i].z;
		//fout << x[i];
		//fout << "\n";
		//fout << flush;
	}
	//��x�����ֵ
	double max_x = max_array(x, size);
	//��x����Сֵ
	double min_x = min_array(x, size);
	//��y�����ֵ
	double max_y = max_array(y, size);
	//��y����Сֵ
	double min_y = min_array(y, size);
	//��z�����ֵ
	double max_z = max_array(z, size);
	//��z����Сֵ
	double min_z = min_array(z, size);
	//��ʼ���߽����
	pcl::PointCloud<pcl::Boundary> boundaries;
	//��ʼ����Ƭ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cuted(new pcl::PointCloud <pcl::PointXYZ>);
	//��ʼ���洢�߽����ĵ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundaries(new pcl::PointCloud <pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
	//Ƭ��
	int n = 2;
	//�����ܶ�
	double res_cloud = 0.0;
	//x�������Ƭ
	//cutPointCloud(n, max_x, min_x, y, z, x, cloud_cuted, size, boundaries, res_cloud, cloud_boundaries);
	//y�������Ƭ
	//cutPointCloud(n, max_y, min_y, z, x, y, cloud_cuted, size, boundaries, res_cloud, cloud_boundaries);
	//z�������Ƭ
	//cutPointCloud(n, max_z, min_z, x, y, z, cloud_cuted, size, boundaries, res_cloud, cloud_boundaries);


	//���㷨��
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
	//pcl::io::savePCDFileASCII("zhengti_bd.pcd", *cloud_boundaries); //�����Ʊ��浽PCD�ļ���
	//pcl::io::savePCDFileASCII("normalzhuzi.pcd", *cloud_normal); //�����Ʊ��浽PCD�ļ���
	//����viewer����
	//ԭʼ������ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ori(new pcl::visualization::PCLVisualizer("ԭʼ����"));
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
	//��Ƭ������ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_cuted(new pcl::visualization::PCLVisualizer("��Ƭ"));
	viewer_cuted->addPointCloud<pcl::PointXYZ>(cloud_cuted, "cut");

	cout << "cuted cloud visulization finished" << endl;

	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normal, 20, 0.03, "normals");
	while (!viewer_cuted->wasStopped())
	{
		viewer_cuted->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	//�߽�ͶӰ������ʾ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_proj(new pcl::visualization::PCLVisualizer("�߽�ͶӰ"));
	viewer_proj->addPointCloud<pcl::PointXYZ>(cloud_boundaries, "projection");

	cout << "projection visulization finished" << endl;

	
	viewer_proj->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_boundaries, cloud_normal, 20, 0.03, "normals");
	while (!viewer_proj->wasStopped())
	{
		viewer_proj->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	DWORD end_time = GetTickCount(); //������ʱ
	cout << "The run time is:" << (end_time - start_time) << "ms!" << endl; //���ʱ��
	
	cin.get();
	return 0;
}