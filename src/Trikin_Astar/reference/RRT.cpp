#include "RRT.h"

pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
bool mapReceived = false;
// kdtree.setInputCloud(PCTrajNode::PCMap);

void RRT::visualizePath(vector<RRT_Node> path)
{
	geometry_msgs::PoseArray msg;
	msg.header.frame_id = "/map";
	msg.header.stamp = ros::Time::now();

	geometry_msgs::Pose pose;
	tf::Matrix3x3 R;
	Matrix4d T;
	Vector3d point, point_parent;
	geometry_msgs::Point point_msg1, point_msg2;

	for (auto node : path)
	{
		T = node.node.get_T();
		point = T.block<3, 1>(0, 3);
		pose.position.x = point(0);
		pose.position.y = point(1);
		pose.position.z = point(2);
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				R[i][j] = T(i, j);
			}
		}
		tfScalar yaw, pitch, row;
		R.getEulerYPR(yaw, pitch, row);
		pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(row, pitch, yaw);
		msg.poses.push_back(pose);
	}
	path_pub.publish(msg);
}

void RRT::visualizeTrees()
{

	// visualize tree nodes
	geometry_msgs::PoseArray msg;
	msg.header.frame_id = "/map";
	msg.header.stamp = ros::Time::now();

	//visualize tree edges
	visualization_msgs::Marker edges;
	edges.header.frame_id = "/map";
	edges.header.stamp = ros::Time::now();
	edges.ns = "RRT";
	edges.id = 0;
	edges.type = visualization_msgs::Marker::LINE_LIST;
	edges.scale.x = 0.2;
	edges.color.a = 1;

	geometry_msgs::Pose pose;
	tf::Matrix3x3 R;
	Matrix4d T;
	Vector3d point, point_parent;
	geometry_msgs::Point point_msg1, point_msg2;

	for (auto node : start_tree)
	{
		T = node.node.get_T();
		point = T.block<3, 1>(0, 3);
		pose.position.x = point(0);
		pose.position.y = point(1);
		pose.position.z = point(2);
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				R[i][j] = T(i, j);
			}
		}
		tfScalar yaw, pitch, row;
		R.getEulerYPR(yaw, pitch, row);
		pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(row, pitch, yaw);
		msg.poses.push_back(pose);

		if (node.parent.second != -1)
		{
			point_msg1.x = point(0);
			point_msg1.y = point(1);
			point_msg1.z = point(2);
			point_parent = start_tree[node.parent.second].node.get_T().block<3, 1>(0, 3);
			point_msg2.x = point_parent(0);
			point_msg2.y = point_parent(1);
			point_msg2.y = point_parent(2);
			edges.points.push_back(point_msg1);
			edges.points.push_back(point_msg2);
		}
	}
	for (auto node : goal_tree)
	{
		T = node.node.get_T();
		point = T.block<3, 1>(0, 3);
		pose.position.x = point(0);
		pose.position.y = point(1);
		pose.position.z = point(2);
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				R[i][j] = T(i, j);
			}
		}
		tfScalar yaw, pitch, row;
		R.getEulerYPR(yaw, pitch, row);
		pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(row, pitch, yaw);
		msg.poses.push_back(pose);

		if (node.parent.second != -1)
		{
			point_msg1.x = point(0);
			point_msg1.y = point(1);
			point_msg1.z = point(2);
			point_parent = goal_tree[node.parent.second].node.get_T().block<3, 1>(0, 3);
			point_msg2.x = point_parent(0);
			point_msg2.y = point_parent(1);
			point_msg2.y = point_parent(2);
			edges.points.push_back(point_msg1);
			edges.points.push_back(point_msg2);
		}
	}

	tree_node_pub.publish(msg);
	tree_edge_pub.publish(edges);
}

void RRT::pclMapCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
	// pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
	if (!mapReceived)
	{
		ROS_INFO("Callback called!");
		pcl::fromROSMsg(*laserCloudMsg, *PCTrajNode::PCMap);
		kdtree.setInputCloud(PCTrajNode::PCMap);

		int n = PCTrajNode::PCMap->points.size();
		for (int i = 0; i < n; i++)
		{
			PCTrajNode::PCMap->points[i].intensity = -1;
		}
		mapReceived = true;

		// pcl::io::savePCDFileASCII<pcl::PointXYZI>("/home/edward/indoor.pcd", *PCTrajNode::PCMap);

		ROS_INFO("Point cloud planner: Map received.");
		// visualizePointCloud();
		// planOnce();
	}
}

void RRT::setGoalCallback(const geometry_msgs::PoseStamped &goal_msg)
{
	if (mapReceived)
	{
		tf::Quaternion q;
		tf::quaternionMsgToTF(goal_msg.pose.orientation, q);
		tf::Matrix3x3 R(q);

		Matrix4d goal_mat = Matrix4d::Identity();
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				goal_mat(i, j) = R[i][j];
			}
		}
		goal_mat(0, 3) = goal_msg.pose.position.x;
		goal_mat(1, 3) = goal_msg.pose.position.y;
		goal_mat(2, 3) = goal_msg.pose.position.z;
		//TODO
		PCTrajNode pcn(goal_mat), pseudoOdom(Matrix4d::Identity());
		find_path(pseudoOdom, pcn);
		visualizeTrees();
	}
	else
	{
		ROS_INFO("No map received yet! Can't plan now.");
	}
}

void RRT::publishSamplePoint(Vector3d point)
{
	geometry_msgs::PointStamped msg;
	msg.header.frame_id = "/map";
	msg.header.stamp = ros::Time::now();
	msg.point.x = point(0);
	msg.point.y = point(1);
	msg.point.z = point(2);
	sample_pub.publish(msg);
}

vector<RRT_Node> RRT::get_path()
{
	vector<RRT_Node> result;
	pair<bool, int> g = {true, goal_idx};
	cout << "trajectory cost=" << get_Node(g).cost << endl;
	while (g.second != -1)
	{
		RRT_Node &p = get_Node(g);
		// cout << p.node.get_pos()[0] << " " << p.node.get_pos()[1] << " " << p.node.get_pos()[2] << endl;
		result.push_back(p);
		g = p.parent;
	}
	inProcess = false;

	visualizePath(result);
	ROS_INFO("Path nodes number: %d", result.size());
	return result;
}

vector<RRT_Node> RRT::find_path(const PCTrajNode &start, const PCTrajNode &goal)
{
	inProcess = true;

	// initialization
	vector<RRT_Node> result;
	if(clear_start_tree)
	{
		start_tree.clear();
	}
	// 
	goal_tree.clear();
	RRT_Node s(start, {true, -1});
	RRT_Node g(goal, {false, -1});
	s.cost = 0;
	g.cost = 0;
	start_tree.push_back(s);
	goal_tree.push_back(g);

	// begin iteration
	cout << "RRT begin!" << endl;
	pcl::StopWatch time;
	for (int i = 0; i < max_iter; i++)
	{
		// sample, extend this tree
		// cout << "iteration: " << i << endl;
		if (i % 100 == 0)
		{
			ROS_INFO("Iteration: %d", i);
			visualizeTrees();
		}
		Vector3d p = sample();
		//visualize
		publishSamplePoint(p);
		pair<bool, int> p_new = extend(p);

		if (p_new.second != -1) // if it's valid
		{
			// change, and try connect with that tree
			change_direction();		//
			if (try_connect(p_new)) // path found
			{
				cout << "get path in iteration: " << i << endl;
				cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
				goal_idx = start_tree.size() - 1;
				return get_path();
			}
			// can't connect, from that tree extend to p_new, try connect continuely
			else
			{
				Vector3d p_new_pos = get_Node(p_new).node.get_pos();
				pair<bool, int> p_t = extend(p_new_pos);
				int cnt = 0;
				while (p_t.second != -1)
				{
					cnt++;
					if (cnt >= heuristic_straight_thresh)
					{
						break;
					}
					change_direction();
					if (try_connect(p_t))
					{
						cout << "get path in  iteration: " << i << endl;
						cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
						goal_idx = start_tree.size() - 1;
						return get_path();
					}
					change_direction();
					p_t = extend(p_new_pos);
				}
			}
		}
	}

	// can't find path
	cout << "can't find path!!!" << endl;
	ROS_INFO("start tree size: %d, goal tree size: %d", start_tree.size(), goal_tree.size());
	inProcess = false;
	return result;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "RRT_node");
	ROS_INFO("Planner started!");
	RRT planner;
	ros::spin();
}