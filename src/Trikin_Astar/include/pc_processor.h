#pragma once
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <vector>
#include <algorithm>
#include <cmath>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/surface/impl/organized_fast_mesh.hpp>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace Eigen;

class PcProcessor
{
private:
	double max_z;
	double min_z;
	double max_x;
	double min_x;
	double max_y;
	double min_y;
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointMap;
public:
	PcProcessor():pointMap(new pcl::PointCloud<pcl::PointXYZ>)
	{}
	~PcProcessor()
	{}
	void boxFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr source, Matrix<double, 4, 4> T);
	pcl::PointCloud<pcl::PointXYZ>::Ptr outlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int knum, double threshold);
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxelgridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, Vector4f leafsize);
	pcl::PointCloud<pcl::PointXYZ>::Ptr mlsSmooth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, double radius, int order = 2, bool computeNormal = false);
	pcl::PolygonMesh fast_mesh_gen(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int width = 640, int height = 480, int size = 2,\
    	float fx = 60, float fy = 60, float cx = 320, float cy = 240);
};