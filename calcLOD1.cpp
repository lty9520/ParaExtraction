#include "calcLOD1.h"

calcLOD1::calcLOD1()
{

}

calcLOD1::~calcLOD1()
{

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
void calcLOD1::calc(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int size, double* x, double* y, double* z){

	//求宽度t
	for (int i = 0; i < size; i++){

		x[i] = cloud_in->points[i].x;
	}

	//fout.close();

	double max_x = 0.0;
	double min_x = 0.0;

	max_x = tool.max_array(x, size);
	min_x = tool.min_array(x, size);

	double t = max_x - min_x;

	//cout << "max_x = " << max_x << endl;
	//cout << "min_x = " << min_x << endl;
	cout << "t = " << t << "\n" << endl;

	//求高度h
	for (int m = 0; m < size; m++){

		y[m] = cloud_in->points[m].y;
		//fout << x[i];
		//fout << "\n";
		//fout << flush;
	}

	//fout.close();

	double max_y = 0.0;
	double min_y = 0.0;

	max_y = tool.max_array(y, size);
	min_y = tool.min_array(y, size);

	double h = max_y - min_y;

	//cout << "max_x = " << max_x << endl;
	//cout << "min_x = " << min_x << endl;
	cout << "h = " << h << "\n" << endl;

	//求长度b
	for (int n = 0; n < size; n++){

		z[n] = cloud_in->points[n].z;
		//fout << x[i];
		//fout << "\n";
		//fout << flush;
	}

	//fout.close();

	double max_z = 0.0;
	double min_z = 0.0;

	max_z = tool.max_array(z, size);
	min_z = tool.min_array(z, size);

	double b = max_z - min_z;

	//cout << "max_x = " << max_x << endl;
	//cout << "min_x = " << min_x << endl;
	cout << "b = " << b << "\n" << endl;
}