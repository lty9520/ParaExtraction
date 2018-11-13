#ifndef CLOUD_CUT_H
#define CLOUD_CUT_H

#include "Tools.h"

using namespace std;
using namespace pcl;

class cloud_cut{

private:

	Tools tool;


public:

	cloud_cut();
	virtual ~cloud_cut();

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
	void cutPointCloud(int n, double max_x, double min_x, double* y, double* z, double* x,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, int size,
		pcl::PointCloud<pcl::Boundary>& boundaries,
		double res_cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_boundaries);

};

#endif	//CLOUD_CUT_H
