#include "Trikin_Astar.h"

bool mapReceived = false;

void TrikinAstar::visualizePath(void)
{
	geometry_msgs::PoseArray msg;
	msg.header.frame_id = "/map";
	msg.header.stamp = ros::Time::now();

	geometry_msgs::Pose pose;
	tf::Matrix3x3 R;

	for (auto node : path_nodes_)
	{
		pose.position.x = node->state.pose.x();
		pose.position.y = node->state.pose.y();
		pose.position.z = node->state.pose.z();
		Vector xb(node->state.orient);
		Vector zb(node->state.znormal);
		Vector yb = cross_product(zb,xb);

		R[0][0] = xb[0];
		R[1][0] = xb[1];
		R[2][0] = xb[2];
		
		R[0][1] = yb[0];
		R[1][1] = yb[1];
		R[2][1] = yb[2];

		R[0][2] = zb[0];
		R[1][2] = zb[1];
		R[2][2] = zb[2];
		
		tfScalar yaw, pitch, row;
		R.getEulerYPR(yaw, pitch, row);
		pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(row, pitch, yaw);
		msg.poses.push_back(pose);
	}
	path_pub.publish(msg);
}

void TrikinAstar::setGoalCallback(const geometry_msgs::PoseStamped &goal_msg)
{
	// call 
	if (mapReceived)
	{
		tf::Quaternion q;
		tf::quaternionMsgToTF(goal_msg.pose.orientation, q);
		tf::Matrix3x3 R(q);

		Point start_b(0, -1, -0.22);
		Vector zb(0, 0, 1);
		Vector xb(-1, 0, 0);
		Point end(4.625,-1.95,-0.34);
		State start(start_b, xb, zb);
		// Point goal(4.625,-1.95,-0.34);
		Point goal(goal_msg.pose.position.x, goal_msg.pose.position.y, -0.34);

		expanded_nodes_.clear();
		path_nodes_.clear();
		openSet.clear();

		for (int i = 0; i < use_node_num_; i++)
		{
			PathNodePtr node = path_node_pool_[i];
			node->parent = nullptr;
			node->node_state = NOT_EXPAND;
		}
		use_node_num_ = 0;
		iter_num_ = 0;
		search(start, goal);
		visualizePath();
	}
	else
	{
		ROS_INFO("No map received yet! Can't plan now.");
	}
}

void TrikinAstar::pclMapCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
	// call mesh generator
	if (!mapReceived)
	{
		// TODO: do somethong...
		// pcl::fromROSMsg(*laserCloudMsg, *PCTrajNode::PCMap);
		// kdtree.setInputCloud(PCTrajNode::PCMap);

		// int n = PCTrajNode::PCMap->points.size();
		// for (int i = 0; i < n; i++)
		// {
		// 	PCTrajNode::PCMap->points[i].intensity = -1;
		// }

		mapReceived = true;

		ROS_INFO("Point cloud planner: Map received.");
	}
}

void TrikinAstar::retrievePath(PathNodePtr end_node)
{
	PathNodePtr cur_node = end_node;
	path_nodes_.push_back(cur_node);

	while (cur_node->parent != nullptr)
	{
		cur_node = cur_node->parent;
		path_nodes_.push_back(cur_node);
	}

	reverse(path_nodes_.begin(), path_nodes_.end());
}

double crossValue(const Vector& a, const Vector& b)
{
	return a.x()*b.y()-a.y()*b.x();
}

Ppid TrikinAstar::closestFace(face_descriptor fd, Point p)
{
	queue<face_descriptor> qf;
	vector<face_descriptor> vf;
	qf.push(fd);
	vf.push_back(fd);
	int cnt = 0;
	while(!qf.empty())
	{
		face_descriptor current = qf.front();
		qf.pop();
		halfedge_descriptor hd = current->halfedge();
		Point a = hd->vertex()->point();
		Point b = hd->next()->vertex()->point();
		Point c = hd->next()->next()->vertex()->point();
		Vector cp = p - c;
		Vector fn = fnormals[current];
		Vector pdc = scalar_product(cp, fn)*fn - cp;
		Point pd = c - pdc;
		Vector pdb = b - pd;
		Vector pda = a - pd;
		double cross_a = crossValue(pda, pdb);
		double cross_b = crossValue(pdb, pdc);
		double cross_c = crossValue(pdc, pda);

		if (cross_a*cross_b >= 0 && cross_a*cross_c >= 0)
		{
			return {pd, current};
		}
		else
		{
			for (int i=0; i<3; i++, hd = hd->next())
			{
				if (hd->is_border_edge())
				{
					continue;
				}
				face_descriptor temp = hd->opposite()->facet();
				if (find(vf.begin(),vf.end(),temp)==vf.end())
				{
					qf.push(temp);
					vf.push_back(temp);
				}
			}
		}
		cnt++;
		if (cnt>50)
		{
			// cout<<"faces is out of limitation!"<<endl;
			break;
		}
	}

	// std::cout<<"nothing found!"<<std::endl;
	return {p,fd};
}

Ppid TrikinAstar::closestFace(State& s)
{
	// clock_t startTime, endTime;
	// startTime = clock();
	// Ppid a =  (*DabTree).closest_point_and_primitive(s.pose);
	// endTime = clock(); 
	// std::cout << "closestFace time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
	// Ppid b{a.first,a.second};
	// return a;
	for (auto i=MeshMap->facets_begin();i!=MeshMap->facets_end();i++)
	{
		halfedge_descriptor hd = i->halfedge();
		Point a = hd->vertex()->point();
		Point b = hd->next()->vertex()->point();
		Point c = hd->next()->next()->vertex()->point();
		Vector cp = s.pose - c;
		Vector fn = fnormals[i];
		Vector pdc = scalar_product(cp, fn)*fn - cp;
		Point pd = c - pdc;
		Vector pdb = b - pd;
		Vector pda = a - pd;
		double cross_a = crossValue(pda, pdb);
		double cross_b = crossValue(pdb, pdc);
		double cross_c = crossValue(pdc, pda);

		if (cross_a*cross_b >= 0 && cross_a*cross_c >= 0)
		{
			return {pd, i};
		}
	}

	clock_t startTime, endTime;
	startTime = clock();
	Ppid a =  (*DabTree).closest_point_and_primitive(s.pose);
	endTime = clock(); 
	std::cout << "closestFace time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
	Ppid b{a.first,a.second};
	return a;
}

void TrikinAstar::AstarSearch(PathNodePtr start, face_descriptor end)
{
	AstarPath.clear();
	resetAbuf();

	// get start halfedge
	Point endc = centroid(end->halfedge()->vertex()->point(), end->halfedge()->next()->vertex()->point(), end->halfedge()->next()->next()->vertex()->point());
	Point o = start->state.pose;
	Vector od[3];
	double dot[3];
	double dotV = -2;
	size_t a = -1;
	halfedge_descriptor hd = start->findex->halfedge();
	for (size_t i = 0; i < 3; i++, hd = hd->next())
	{
		od[i] = hd->vertex()->point() - o;
		od[i] /= sqrt(od[i].squared_length());
		dot[i] = scalar_product(od[i], start->state.orient);
		if (dotV < dot[i])
		{
			dotV = dot[i];
			a = i;
		}
	}
	Vector dotH = od[a] + start->state.orient;
	dotH /= sqrt(dotH.squared_length());
	for (size_t i = 0; i < 3; i++)
	{
		if (i != a && scalar_product(od[i], dotH) < dot[i])
		{
			for (int j = 0; j < 2 - (a + i) % 3; j++)
			{
				hd = hd->next();
			}
			break;
		}
	}

	// initialization
	multimap<double, GridNodePtr> OS;
	GridNodePtr s = AstarBuffer[hd];
	s->center = midpoint(hd->vertex()->point(), hd->opposite()->vertex()->point());
	s->fScore = s->gScore = 0;
	OS.insert(make_pair(s->fScore, s));

	// begin A star search
	int cnt = 0;
	while (!OS.empty()) {
		cnt++;
		auto iter = begin(OS);
		GridNodePtr current = iter->second;
		OS.erase(iter);
		current->node_state = IN_CLOSE_SET;

		if (current->node->opposite()->face() == end) {
			//cout << cnt << endl;
			GridNodePtr p = current;
			while (p != nullptr) {
				AstarPath.push_back(p->node);
				p = p->parent;
			}
			reverse(AstarPath.begin(), AstarPath.end());
			break;
		}

		halfedge_descriptor p = current->node->opposite();
		for (int i = 0; i < 2; i++, p = p->next()) {
			auto tempI = p->next();
			GridNodePtr btempI = AstarBuffer[tempI];
			/// openSet
			if (btempI->node_state == IN_OPEN_SET) {
				double sgScore = current->gScore + 1;
				//double sgScore = btempI->gScore = current->gScore + (btempI->center - current->center).squared_length();
				if (sgScore < btempI->gScore) {
					btempI->gScore = sgScore;
					btempI->fScore = btempI->gScore + btempI->heu;
					btempI->parent = current;
				}
				continue;
			}
			/// closeSet
			if (btempI->node_state == IN_CLOSE_SET) {
				double sgScore = current->gScore + 1;
				//double sgScore = btempI->gScore = current->gScore + (btempI->center - current->center).squared_length();
				if (sgScore < btempI->gScore) {
					btempI->gScore = sgScore;
					btempI->fScore = btempI->gScore + btempI->heu;
					btempI->parent = current;
					btempI->node_state = IN_OPEN_SET;
					OS.insert(make_pair(btempI->fScore, btempI));
				}
				continue;
			}
			/// unVisited
			if (btempI->node_state == NOT_EXPAND && !tempI->is_border_edge()) {
				btempI->gScore = current->gScore + 1;
				//btempI->center = midpoint(tempI->vertex()->point(), tempI->opposite()->vertex()->point());
				//btempI->gScore = current->gScore + (btempI->center-current->center).squared_length();
				btempI->heu = (endc - btempI->center).squared_length();
				btempI->fScore = btempI->gScore + btempI->heu;
				btempI->parent = current;
				btempI->node_state = IN_OPEN_SET;
				OS.insert(make_pair(btempI->fScore, btempI));
			}
		}

		if (cnt > MeshMap->capacity_of_halfedges()) {
			cout << "A* SEARCH HAS LOOP, ERROR OCCURED!" << endl;
			break;
		}
	}
}

bool TrikinAstar::search(State start, Point goal)
{
	std::cout << "begin Trikin A star" << std::endl;
	clock_t startTime, endTime;
	startTime = clock();

	PathNodePtr cur_node = path_node_pool_[0];
	cur_node->parent = nullptr;
	Ppid pp_start = closestFace(start);
	// Ppid pp_start = closestFace(, start.pose);
	cur_node->findex = pp_start.second;
	cur_node->state = castDown(start, pp_start);
	cur_node->input = 0.0;
	cur_node->g_score = 0.0;
	cur_node->f_score = lambda_heu_ * estimateHeuristic(start, goal);
	cur_node->node_state = IN_OPEN_SET;
	openSet.insert(make_pair(cur_node->f_score, cur_node));
	use_node_num_++;
	expanded_nodes_.insert(make_pair(cur_node->findex, cur_node));

	PathNodePtr neighbor = nullptr;
	PathNodePtr terminate_node = nullptr;
	
	///*----------- TEST EFFICIENCY OF A_STAR -----------*/
	//Ppid pp_goal = (*DabTree).closest_point_and_primitive(goal);
	//clock_t sc = clock();
	//AstarSearch(cur_node, pp_goal.second);
	//clock_t ec = clock();
	//cout << "Astar search time: " << (double)(ec - sc) / CLOCKS_PER_SEC << "s" << endl;
	//for (halfedge_descriptor i : AstarPath)
	//{
	//	cout << (i->vertex()->point().x() + i->opposite()->vertex()->point().x()) / 2 << " "
	//		<< (i->vertex()->point().y() + i->opposite()->vertex()->point().y()) / 2 << " "
	//		<< (i->vertex()->point().z() + i->opposite()->vertex()->point().z()) / 2 << endl;
	//}
	//return false;
	///*----------- TEST EFFICIENCY OF A_STAR -----------*/

	while (!openSet.empty())
	{
		auto iter = begin(openSet);
		cur_node = iter->second;

		if ((cur_node->state.pose - goal).squared_length() < tolerance2)
		{
			terminate_node = cur_node;
			retrievePath(terminate_node);
			endTime = clock(); 
			std::cout << "Find path! The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
			std::cout << "use node num: " << use_node_num_ << endl;
			std::cout << "iter num: " << iter_num_ << endl;
			return true;
		}

		openSet.erase(iter);
		cur_node->node_state = IN_CLOSE_SET;
		iter_num_++;

		State cur_state = cur_node->state;
		State pro_state;
		vector<PathNodePtr> tmp_expand_nodes;
		double duration = min_tau_*sqrt(faceArea2(cur_node->findex)/com_area2);
		for (double kap = -max_kappa; kap <= max_kappa + 1e-3; kap += max_kappa * resolution_)
		{
			stateTransit(cur_state, pro_state, kap, duration);

			// check if in close set
			// clock_t cfst, cfet;			cfst = clock();
			Ppid pro_id = closestFace(cur_node->findex, pro_state.pose);
			// cfet = clock();	std::cout << "new closestFace time is: " << (double)(cfet - cfst) / CLOCKS_PER_SEC << "s" << endl;
			map<face_descriptor, PathNodePtr>::iterator pro_pair = expanded_nodes_.find(pro_id.second);
			if (pro_pair != expanded_nodes_.end() && pro_pair->second->node_state == IN_CLOSE_SET)
			{
				continue;
			}

			// check maximal kappa
			/*if (fabs(pro_state.kappa) > max_kappa)
			{
				continue;
			}*/

			// check not in the same face
			if (cur_node->findex == pro_id.second)
			{
				continue;
			}

			// check path and safety, compute rho
			bool safe = true;
			double rho = 2;
			// AstarSearch(cur_node, pro_id.second);
			// if (AstarPath.empty())
			// {
			// 	continue;
			// }
			// for (auto i = 0; i < AstarPath.size(); i++)
			// {
			// 	halfedge_descriptor hd = AstarPath[i];
			// 	if (!isValidHe(hd))
			// 	{
			// 		safe = false;
			// 		break;
			// 	}
			// 	double tmp_rho = scalar_product(fnormals[hd->opposite()->facet()], fnormals[hd->facet()]);
			// 	if (tmp_rho < rho)
			// 	{
			// 		rho = tmp_rho;
			// 	}
			// }
			// if (!safe)
			// {
			// 	continue;
			// }
			// rho = k_rho * (1 - rho);
			rho = 0;

			double time_to_goal, tmp_g_score, tmp_f_score;
			tmp_g_score = (1 + 0.5*k_dkap*fabs(kap - cur_node->input) / max_kappa)*(1 + rho) * duration + cur_node->g_score;
			//tmp_g_score = (dkap + w_time_) * duration + cur_node->g_score;
			tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, goal);

			// Compare nodes expanded from the same parent
			bool prune = false;
			for (int j = 0; j < tmp_expand_nodes.size(); ++j)
			{
				PathNodePtr expand_node = tmp_expand_nodes[j];
				if (pro_id.second==expand_node->findex)
				{
					prune = true;
					if (tmp_f_score < expand_node->f_score)
					{
						expand_node->f_score = tmp_f_score;
						expand_node->g_score = tmp_g_score;
						expand_node->state = castDown(pro_state, pro_id);
						expand_node->input = kap;
						expand_node->duration = duration;
					}
					break;
				}
			}

			// This node end up on a face different from others
			if (!prune)
			{
				if (pro_pair == expanded_nodes_.end())
				{
					PathNodePtr pro_node = path_node_pool_[use_node_num_];
					pro_node->findex = pro_id.second;
					pro_node->state = castDown(pro_state, pro_id);
					pro_node->f_score = tmp_f_score;
					pro_node->g_score = tmp_g_score;
					pro_node->input = kap;
					pro_node->duration = duration;
					pro_node->parent = cur_node;
					pro_node->node_state = IN_OPEN_SET;

					openSet.insert(make_pair(pro_node->f_score, pro_node));
					expanded_nodes_.insert(make_pair(pro_id.second, pro_node));
					tmp_expand_nodes.push_back(pro_node);

					use_node_num_ ++;
					if (use_node_num_ == allocate_num_)
					{
						std::cout << "run out of memory." << endl;
						return false;
					}
				}
				else
				{
					PathNodePtr pro_node = (*pro_pair).second;
					if (tmp_f_score < pro_node->f_score)
					{
						// pro_node->findex = pro_id.second;
						pro_node->state = castDown(pro_state, pro_id);
						pro_node->f_score = tmp_f_score;
						pro_node->g_score = tmp_g_score;
						pro_node->input = kap;
						pro_node->duration = duration;
						pro_node->parent = cur_node;
					}
				}
			}
		}
		if (iter_num_ > max_iter) {
			std::cout << "TRIKIN-ASTAR SEARCH OUT OF MAX_ITER, ERROR OCCURED!" << endl;
			break;
		}
	}

	std::cout << "open set empty, no path!" << endl;
	std::cout << "use node num: " << use_node_num_ << endl;
	std::cout << "iter num: " << iter_num_ << endl;
	return false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Trikin_Astar");
	ROS_INFO("Planner started!");
	TrikinAstar planner;

	// read triangle mesh map from off file and make AABBTree
	PolyMesh P;
	//ifstream in1("cat_face.off");
	ifstream in1(planner.mapFile);
	in1 >> P;
	if (!P.is_pure_triangle())
	{
		std::cout << "ERROR! NEED TRIANGLE MESH!" << endl;
		return -1;
	}
	if (P.is_empty())
	{
		std::cout<< "ERROR! NO MESH LOADED!"<<endl;
		return -1;
	}
	Tree tree(faces(P).first, faces(P).second, P);
	
	// define start state and end point
	//Point start_b(0.0285, -0.3294, 1.04);
	//Vector zb(-0.3782, -0.1171, 0.9183);
	//Vector xb(0.3484, -0.9371, 0.02300);
	////Point end(0.4063, 0.6373, 0.1441);
	//Point end(0.080489333333333,   0.562805000000000,   0.253087);
	//State start(start_b, xb, zb);
	Point start_b(0, -1, -0.22);
	Vector zb(0, 0, 1);
	Vector xb(-1, 0, 0);
	Point end(4.625,-1.95,-0.34);
	State start(start_b, xb, zb);

	// call trinkin Astar planner
	planner.init(&P, &tree);
	
	if (planner.search(start, end))
	{
		vector<PathNodePtr> path = planner.getPath();
		for (auto i : path)
		{
			std::cout << i->state.pose << endl;
		}
		planner.visualizePath();
	}

	ros::spin();

	return 0;
}

