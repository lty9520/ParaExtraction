#include "cloud_cut.h"

cloud_cut::cloud_cut()
{

}

cloud_cut::~cloud_cut()
{

}

//************************************
// Method:    cutPointCloud
// FullName:  cloud_cut::cutPointCloud
// Access:    public 
// Returns:   void
// Qualifier:
// Parameter: int n
// Parameter: double max_x
// Parameter: double min_x
// Parameter: double * y
// Parameter: double * z
// Parameter: double * x
// Parameter: pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in
// Parameter: int size
// Parameter: pcl::PointCloud<pcl::Boundary> & boundaries
// Parameter: double res_cloud
// Parameter: pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_boundaries
//************************************
void cloud_cut::cutPointCloud(int n, double max_x, double min_x, double* y, double* z, double* x,
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
	for (int b = 1; b < n + 2; b++){
		x_pitch[b] = x_pitch[b - 1] + pitch;
		cout << "x_pitch[" << b << "] = " << x_pitch[b - 1] << endl;
	}

	double * x_cuted = new double[n];




	for (int i = 0; i < n + 1; i++){
		x_cuted[i] = x_pitch[i] + 0.5 * pitch;
	}
	for (int k = 1; k < 2; k++){
		cout << "k =" << k << endl;
		int u = 0;
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
		for (int i = 0; i < u; i++){

			cloud_in->points[i].x = x_cuted[k];
			cloud_in->points[i].y = y_cuted[i];
			cloud_in->points[i].z = z_cuted[i];



		}
		cloud_in->width = u;
		cloud_in->points.resize(cloud_in->width * cloud_in->height);

		cout << "point cloud_cuted has:" << cloud_in->points.size() << "data points" << endl;

		res_cloud = tool.computeCloudResolution(cloud_in);
		std::cout << "cloud resolution = " << res_cloud << endl;

		tool.boundaryEstimation(boundaries, cloud_in, res_cloud, cloud_boundaries);
	}

}