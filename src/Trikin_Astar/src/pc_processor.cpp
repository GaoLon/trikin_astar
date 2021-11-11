#include "pc_processor.h"

void PcProcessor::boxFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr source, Matrix<double, 4, 4> T)
{
	pcl::CropBox<pcl::PointXYZ> clipper;
	Matrix<double, 4, 1> mindown(min_x, min_y, min_z, 1);
	Matrix<double, 4, 1> maxup(max_x, max_y, max_z, 1);
	Matrix<double, 4, 1> minp = T * mindown;
	Vector4f min_point(minp.col(0)[0], minp.col(0)[1], minp.col(0)[2], minp.col(0)[3]);
	Matrix<double, 4, 1> maxp = T * maxup;
	Vector4f max_point(maxp.col(0)[0], maxp.col(0)[1], maxp.col(0)[2], maxp.col(0)[3]);
	clipper.setMin(min_point);
	clipper.setMax(max_point);
	clipper.setInputCloud(source);
	clipper.setNegative(false);
	clipper.filter(*pointMap);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PcProcessor::outlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int knum, double threshold)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setMeanK(knum);
	sor.setStddevMulThresh(threshold);
	sor.filter(*cloud_out);
	return cloud_out;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PcProcessor::voxelgridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, Vector4f leafsize)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setLeafSize(leafsize);
	sor.filter(*cloud_out);
	return cloud_out;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PcProcessor::mlsSmooth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, double radius, int order, bool computeNormal)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>  sor;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr temptree(new pcl::search::KdTree<pcl::PointXYZ>);
	temptree->setInputCloud(cloud_in);
	sor.setInputCloud(cloud_in);
	sor.setComputeNormals(computeNormal);
	sor.setPolynomialFit(false);
	sor.setPolynomialOrder(order);
	sor.setSearchMethod(temptree);
	sor.setSearchRadius(radius);
	sor.process(*cloud_out);
	return cloud_out;
}

pcl::PolygonMesh PcProcessor::fast_mesh_gen(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int width, \
	int height, int size, float fx, float fy, float cx, float cy)
{
	pcl::StopWatch time;
	cout << "begin mesh..." << endl;

	pcl::RangeImage::CoordinateFrame cf = pcl::RangeImage::CAMERA_FRAME;
	Eigen::Affine3f sP;
	sP.setIdentity();

	pcl::RangeImagePlanar::Ptr rI(new pcl::RangeImagePlanar);
	rI->createFromPointCloudWithFixedSize(*cloud_in, width, height, cx, cy, fx, fy, sP, cf);
	
	pcl::OrganizedFastMesh<pcl::PointXYZ>::Ptr ti(new pcl::OrganizedFastMesh<pcl::PointXYZ>);

	pcl::OrganizedFastMesh<pcl::PointWithRange>::Ptr tri(new pcl::OrganizedFastMesh<pcl::PointWithRange>);
	pcl::search::KdTree<pcl::PointWithRange>::Ptr tree(new pcl::search::KdTree<pcl::PointWithRange>);
	tree->setInputCloud(rI);
	pcl::PolygonMesh t;
	tri->setTrianglePixelSize(size);
	tri->setInputCloud(rI);
	tri->setSearchMethod(tree);
	tri->setTriangulationType(pcl::OrganizedFastMesh<pcl::PointWithRange>::TRIANGLE_ADAPTIVE_CUT);
	tri->reconstruct(t);

	cout << "sucess, time consuming = " << time.getTimeSeconds() << " s" << endl;
	return t;
}