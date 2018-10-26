#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/console/parse.h>


using namespace std;
using namespace pcl;


typedef pcl::PointXYZ PointType;

//����������
/*
*	����� ����һƬ�����һƬ��t1b3����	angular_resolution = 0.05
*											support_size = 0.02
*	����� ��h2t2����	angular_resolution = 0.05
*											support_size = 0.015
*	���ӵ�һƬ(1) ��	angular_resolution = 0.02
*					support_size = 0.0049
*	���ӵ�һƬ(2) ��	angular_resolution = 0.025
*					support_size = 0.0049
*	���ӵ�һƬ(2) ��	angular_resolution = 0.018
*					support_size = 0.0045
*   ���ӷ��      0.023��0.013
*/
/*
float angular_resolution = 0.2f;
float support_size = 0.008f;
*/
float angular_resolution = 0.005f;
float support_size = 0.003f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;
bool rotation_invariant = true;

//�������
void
printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-r <float>   angular resolution in degrees (default " << angular_resolution << ")\n"
		<< "-c <int>     coordinate frame (default " << (int)coordinate_frame << ")\n"
		<< "-m           Treat all unseen points to max range\n"
		<< "-s <float>   support size for the interest points (diameter of the used sphere - "
		"default " << support_size << ")\n"
		<< "-o <0/1>     switch rotational invariant version of the feature on/off"
		<< " (default " << (int)rotation_invariant << ")\n"
		<< "-h           this help\n"
		<< "\n\n";
}

void
setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)//setViewerPose
{
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
		look_at_vector[0], look_at_vector[1], look_at_vector[2],
		up_vector[0], up_vector[1], up_vector[2]);
}

int
main(int argc, char** argv)
{
	// ���ò������
	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}
	if (pcl::console::find_argument(argc, argv, "-m") >= 0)
	{
		setUnseenToMaxRange = true;
		cout << "Setting unseen values in range image to maximum range readings.\n";
	}
	if (pcl::console::parse(argc, argv, "-o", rotation_invariant) >= 0)
		cout << "Switching rotation invariant feature version " << (rotation_invariant ? "on" : "off") << ".\n";
	int tmp_coordinate_frame;
	if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
	{
		coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
		cout << "Using coordinate frame " << (int)coordinate_frame << ".\n";
	}
	if (pcl::console::parse(argc, argv, "-s", support_size) >= 0)
		cout << "Setting support size to " << support_size << ".\n";
	if (pcl::console::parse(argc, argv, "-r", angular_resolution) >= 0)
		cout << "Setting angular resolution to " << angular_resolution << "deg.\n";
	angular_resolution = pcl::deg2rad(angular_resolution);

	//��һ�������е�.pcd�ļ�  �������û��ָ���ͻ��Զ�����
	pcl::PointCloud<PointType>::Ptr    point_cloud_ptr(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;

	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
	std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");

	if (!pcd_filename_indices.empty())   //����Ƿ���far_ranges.pcd
	{
		std::string filename = argv[pcd_filename_indices[0]];
		if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
		{
			cerr << "Was not able to open file \"" << filename << "\".\n";
			printUsage(argv[0]);
			return 0;
		}
		scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
			point_cloud.sensor_origin_[1],
			point_cloud.sensor_origin_[2])) *
			Eigen::Affine3f(point_cloud.sensor_orientation_);
		std::string far_ranges_filename = pcl::getFilenameWithoutExtension(filename) + "_far_ranges.pcd";
		//if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
			//std::cout << "Far ranges file \"" << far_ranges_filename << "\" does not exists.\n";
	}
	else
	{
		setUnseenToMaxRange = true;
		cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
		for (float x = -0.5f; x <= 0.5f; x += 0.01f)   //���û�д򿪵��ļ�������һ�����εĵ���
		{
			for (float y = -0.5f; y <= 0.5f; y += 0.01f)
			{
				PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
				point_cloud.points.push_back(point);
			}
		}
		point_cloud.width = (int)point_cloud.points.size();  point_cloud.height = 1;
	}

	//�ӵ����н����������ͼ
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	range_image.createFromPointCloud(point_cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
		scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	//range_image.integrateFarRanges(far_ranges);
	if (setUnseenToMaxRange)
		range_image.setUnseenToMaxRange();

	//��3D viewer���������
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(0.75, 0.75, 0.75);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
	viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "range image");
	//viewer.addCoordinateSystem (1.0f, "global");
	//PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
	//viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
	viewer.initCameraParameters();
	setViewerPose(viewer, range_image.getTransformationToWorldSystem());
	//��ʾ
	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(range_image);

	//��ȡNARF����
	pcl::RangeImageBorderExtractor range_image_border_extractor;    //�������ͼ��Ե��ȡ��
	pcl::NarfKeypoint narf_keypoint_detector;                       //narf_keypoint_detectorΪ���ƶ���

	narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extractor);
	narf_keypoint_detector.setRangeImage(&range_image);
	narf_keypoint_detector.getParameters().support_size = support_size;    //���������ȡ�Ĵ�С

	pcl::PointCloud<int> keypoint_indices;
	narf_keypoint_detector.compute(keypoint_indices);
	std::cout << "Found " << keypoint_indices.points.size() << " key points.\n";

	// ----------------------------------------------
	// -----Show keypoints in range image widget-----
	// ----------------------------------------------
	//for (size_t i=0; i<keypoint_indices.points.size (); ++i)
	//range_image_widget.markPoint (keypoint_indices.points[i]%range_image.width,
	//keypoint_indices.points[i]/range_image.width);





	//��3Dviewer��ʾ��ȡ��������Ϣ
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
	keypoints.points.resize(keypoint_indices.points.size());
	for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
		keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();

	pcl::PointCloud<pcl::PointXYZL>::Ptr cloudL(new pcl::PointCloud<pcl::PointXYZL>);

	cloudL->width = static_cast<int>(keypoints_ptr->points.size());
	cloudL->height = 1;
	cloudL->points.resize(cloudL->width * cloudL->height);
	cloudL->is_dense = true;


	for (int i = 0; i < keypoints_ptr->points.size(); i++){
		cloudL->points[i].x = keypoints_ptr->points[i].x;
		cloudL->points[i].y = keypoints_ptr->points[i].y;
		cloudL->points[i].z = keypoints_ptr->points[i].z;
		cloudL->points[i].label = i;
	}
	cout << "transform finished" << endl;

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> keypoints_color_handler(cloudL, 255, 0, 0);
	viewer.addPointCloud<pcl::PointXYZL>(cloudL, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints");
	viewer.setBackgroundColor(255, 255, 255);
	//viewer.addText("ajsldkhaiuwqiohkkjsbrke", 1.0, 2.0);
	/*
	for (int i = 0; i < cloudL->points.size(); i++ ){
	cout << cloudL->points[i].label << "\tx:" << cloudL->points[i].x
	<< "\ty:" << cloudL->points[i].y
	<< "\tz:" << cloudL->points[i].z
	<< "\n" << endl;
	}

	*/



	/*
	*	��ʾ�ؼ�����
	*	���һƬx	1000 z+1, y+1
	*	�м�y
	*/
	for (int i = 0; i < keypoints_ptr->points.size(); i++)
	{
		string label;
		stringstream ss;
		ss << i;
		ss >> label;
		//cout << label << endl;
		//viewer.addText(label, (keypoints_ptr->points[i].y + 0.5) * 900, (keypoints_ptr->points[i].z + 0.5) * 600, label, 0);
		/*
		*	Բ����Ƭ���ѷ죩 ��scale 0.001
		*	���������һƬ�� ��scale 0.005
		*/
		viewer.addText3D(label, keypoints_ptr->points[i], 0.0015, 0.0, 0.0, 0.0, label, 0);

	}

	

	//�ڹؼ�����ȡNARF������
	std::vector<int> keypoint_indices2;
	keypoint_indices2.resize(keypoint_indices.points.size());
	for (unsigned int i = 0; i<keypoint_indices.size(); ++i) // This step is necessary to get the right vector type
		keypoint_indices2[i] = keypoint_indices.points[i];      ///����NARF�ؼ����������������ʸ����ΪNARF���������������ʹ��

	pcl::NarfDescriptor narf_descriptor(&range_image, &keypoint_indices2);//����narf_descriptor���󡣲����˴˶����������ݣ������������������
	narf_descriptor.getParameters().support_size = support_size;//support_sizeȷ������������ʱ���ǵ������С
	narf_descriptor.getParameters().rotation_invariant = rotation_invariant;    //������ת�����NARF������
	pcl::PointCloud<pcl::Narf36> narf_descriptors;               //����Narf36�ĵ�����������ƶ��󲢽���ʵ�ʼ���
	narf_descriptor.compute(narf_descriptors);                 //����������
	cout << "Extracted " << narf_descriptors.size() << " descriptors for "   //��ӡ������������Ŀ����ȡ�����ӵ���Ŀ
		<< keypoint_indices.points.size() << " keypoints.\n";

	/*

	cout << "����Ҫ�󼸶ξ���\n";
	int num = 0;
	cin >> num;

	cout << "����Ҫ���������\n";
	double dis[100] = {0.0};
	int p1[100] = {0};
	int p2[100] = {0};

	for (int i = 0; i < num; i++){
		int a, b = 0;
		cin >> a >> b;
		p1[i] = a;
		p2[i] = b;
		double distance = 0.0;
		double x1 = keypoints_ptr->points[a].x;
		double y1 = keypoints_ptr->points[a].y;
		double z1 = keypoints_ptr->points[a].z;
		double x2 = keypoints_ptr->points[b].x;
		double z2 = keypoints_ptr->points[b].z;
		double y2 = keypoints_ptr->points[b].y;
		distance = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
		dis[i] = distance;
	}
	ofstream fout1("1.txt");
	fout1 << "i\tPointID1\tPointID2\tDisatance";
	fout1 << "\n";
	for (int i = 0; i < num; i++){
		fout1 << i;
		fout1 << "\t";
		fout1 << p1[i];
		fout1 << "\t";
		fout1 << p2[i];
		fout1 << "\t";
		fout1 << dis[i];
		fout1 << "\n";
	}
	fout1.close();

	*/
	

	//��ѭ������
	while (!viewer.wasStopped())
	{
		range_image_widget.spinOnce();  // process GUI events
		viewer.spinOnce();
		//pcl_sleep(0.01);
	}



	cout << "����Ҫȡ������  ";
	int numID = 0;
	cin >> numID;

	int pID[100] = { 0 };
	cout << "����Ҫȡ�ĵ�ĵ��  \n";
	for (int i = 0; i < numID; i++){
		cin >> pID[i];
	}


	ofstream fout("chaijie3_zuihou.txt");
	fout << "i\tx\ty\tz";
	fout << "\n";
	for (int i = 0; i < numID; i++){
		fout << pID[i];
		fout << "\t";
		fout << keypoints_ptr->points[pID[i]].x;
		fout << "\t";
		fout << keypoints_ptr->points[pID[i]].y;
		fout << "\t";
		fout << keypoints_ptr->points[pID[i]].z;
		fout << "\n";
	}
	fout.close();

}