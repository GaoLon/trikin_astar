#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <map>
#include <iostream>
#include <fstream>
#include <list>
#include <ctime>
#include <string.h>
#include <queue>

using namespace std;
using namespace CGAL;
namespace PMP = CGAL::Polygon_mesh_processing;

typedef CGAL::Simple_cartesian<double> Kernal;
typedef Kernal::Point_3 Point;
typedef Kernal::Vector_3 Vector;
typedef Kernal::Ray_3 Ray;
typedef Kernal::Plane_3 Plane;
typedef Kernal::Segment_3 Segment;
typedef Kernal::FT FT;
typedef CGAL::Polyhedron_3<Kernal> PolyMesh;
typedef PolyMesh::Halfedge_handle Halfedge_handle;
typedef PolyMesh::Vertex_iterator Vertex_iterator;

typedef boost::graph_traits<PolyMesh>::vertex_descriptor     vertex_descriptor;
typedef boost::graph_traits<PolyMesh>::halfedge_descriptor   halfedge_descriptor;
typedef boost::graph_traits<PolyMesh>::face_descriptor       face_descriptor;

typedef CGAL::AABB_face_graph_triangle_primitive<PolyMesh> Primitive;
typedef CGAL::AABB_traits<Kernal, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef Tree::Point_and_primitive_id Ppid;

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

struct State
{
	Point pose;
	Vector orient;
	Vector znormal;
	State() {}
	State(Point p, Vector o, Vector z) :pose(p), orient(o), znormal(z) {}
	~State() {}
};

struct GridNode;
typedef GridNode* GridNodePtr;
struct GridNode {
	halfedge_descriptor node;
	Point center;
	GridNodePtr parent = nullptr;
	double fScore = inf;
	double gScore = inf;
	double heu = inf;
	char node_state = NOT_EXPAND;
	GridNode(const halfedge_descriptor h) : node(h) {}
	void reset() { parent = nullptr; fScore = gScore = heu = inf; node_state = NOT_EXPAND; }
	~GridNode() {}
};

class PathNode {
public:
	/* -------------------- */
	face_descriptor findex;
	State state;
	double g_score, f_score;
	double input;
	double duration;
	PathNode* parent;
	char node_state;

	/* -------------------- */
	PathNode() {
		parent = nullptr;
		node_state = NOT_EXPAND;
	}
	~PathNode() {};
};
typedef PathNode* PathNodePtr;



class TrikinAstar
{
protected:
	ros::NodeHandle nh;
	bool inProcess = false;
	PolyMesh* MeshMap;
	Tree* DabTree;
	vector<PathNodePtr> path_node_pool_;
	multimap<double, PathNodePtr> openSet;
	map<face_descriptor, PathNodePtr> expanded_nodes_;
	map<face_descriptor, Vector> fnormals;
	map<halfedge_descriptor, GridNodePtr> AstarBuffer;
	vector<halfedge_descriptor> AstarPath;
	vector<PathNodePtr> path_nodes_;
	int use_node_num_, iter_num_, max_iter = 10000;

	double min_tau_ = 0.08;	
	double max_kappa = 2;
	double w_time_ = 10.0, lambda_heu_ = 5.0, k_rho = 0, k_dkap = 1;
	int allocate_num_ = 100000;

	double tolerance2 = 0.01, resolution_ = 0.05, com_area2 = 7.6071e-05;
	double min_ctilt = -1;

	void retrievePath(PathNodePtr end_node);
	double estimateHeuristic(const State x, const Point& end)
	{
		return sqrt((x.pose - end).squared_length());
	}
	void stateTransit(const State& state0, State& state1, double um, double tau)
	{
		if (fabs(um) < 1e-2)
		{
			state1.znormal = state0.znormal;
			state1.pose = state0.pose + state0.orient*tau;
			state1.orient =state0.orient;
		}
		else
		{
			Vector ori_y = cross_product(state0.znormal, state0.orient);
			double theta = um * tau;
			double st = sin(theta);
			double ct = cos(theta);
			state1.znormal = state0.znormal;
			state1.pose = state0.pose + state0.orient*st / um + ori_y * (1 - ct) / um;
			state1.orient = ct * state0.orient + st * ori_y;
		}
	}
	Ppid closestFace(State& s);
	Ppid closestFace(face_descriptor fd, Point p);
	State castDown(const State& s, Ppid pp)
	{
		State s1;
		Vector zt = fnormals[pp.second];
		Vector xt = s.orient - scalar_product(s.orient, zt)*zt;
		xt /= sqrt(xt.squared_length());
		s1.pose = pp.first;
		s1.orient = xt;
		s1.znormal = zt;
		
		return s1;
	}
	void resetAbuf(void)
	{
		for (auto i : AstarBuffer)
		{
			i.second->reset();
		}
	}
	void AstarSearch(PathNodePtr start, face_descriptor end);
	bool isValidHe(halfedge_descriptor hd)	// can processed in mesh processing, can speed up
	{
		return fnormals[hd->facet()].z() > min_ctilt;
	}
	double faceArea2(face_descriptor f)
	{
		Vector a = f->halfedge()->vertex()->point() - f->halfedge()->next()->vertex()->point();
		Vector b = f->halfedge()->opposite()->vertex()->point() - f->halfedge()->next()->vertex()->point();
		return cross_product(a, b).squared_length();
	}

public:
	TrikinAstar()
	{
		nh = ros::NodeHandle("~");
		nh.getParam("min_tau_", min_tau_);
		nh.getParam("max_kappa", max_kappa);
		nh.getParam("max_iter", max_iter);
		nh.getParam("w_time_", w_time_);
		nh.getParam("lambda_heu_", lambda_heu_);
		nh.getParam("k_rho", k_rho);
		nh.getParam("k_dkap", k_dkap);
		nh.getParam("allocate_num_", allocate_num_);
		nh.getParam("tolerance2", tolerance2);
		nh.getParam("resolution_", resolution_);
		nh.getParam("com_area2",com_area2);
		nh.getParam("min_ctilt",min_ctilt);
		nh.getParam("mapFile", mapFile);
		ROS_INFO("read parameters done.");

		pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/overall_map", 2, &TrikinAstar::pclMapCallback, this);
		goal_sub = nh.subscribe("/move_base_simple/goal", 10, &TrikinAstar::setGoalCallback, this);
		path_pub = nh.advertise<geometry_msgs::PoseArray>("/planning/path", 2);
	}
	~TrikinAstar()
	{
		for (int i = 0; i < allocate_num_; i++)
		{
			delete path_node_pool_[i];
		}
		for (auto i : AstarBuffer)
		{
			delete i.second;
		}
	}

	//void setParam(ros::NodeHandle& nh);	//get parameters from ROS launch file 
	void init(PolyMesh* MeshMap_, Tree* DabTree_)
	{
		MeshMap = MeshMap_;
		DabTree = DabTree_;
		PMP::compute_face_normals(*MeshMap, boost::make_assoc_property_map(fnormals));

		for (auto p = MeshMap->halfedges_begin(); p != MeshMap->halfedges_end(); p++) {
			auto Gp = new GridNode(p);
			AstarBuffer.insert(make_pair(p, Gp));
		}
		boost::make_assoc_property_map(AstarBuffer);

		path_node_pool_.resize(allocate_num_);
		for (int i = 0; i < allocate_num_; i++)
		{
			path_node_pool_[i] = new PathNode;
		}

		use_node_num_ = 0;
		iter_num_ = 0;
	}
	void reset()
	{
		MeshMap = nullptr;
		DabTree = nullptr;

		expanded_nodes_.clear();
		path_nodes_.clear();
		openSet.clear();

		for (int i = 0; i < use_node_num_; i++)
		{
			PathNodePtr node = path_node_pool_[i];
			node->parent = nullptr;
			node->node_state = NOT_EXPAND;
		}

		resetAbuf();

		use_node_num_ = 0;
		iter_num_ = 0;
	}
	vector<PathNodePtr> getPath() { return path_nodes_; }
	bool search(State start, Point goal);

	string mapFile;
	ros::Publisher path_pub;
	ros::Subscriber goal_sub, pcl_sub;
	void setGoalCallback(const geometry_msgs::PoseStamped &goal_msg);
	void pclMapCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
	void visualizePath(void);
};