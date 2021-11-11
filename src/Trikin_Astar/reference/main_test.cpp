#include "geometrycentral/surface/flip_geodesics.h"
#include "geometrycentral/surface/geometry.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh_factories.h"
#include <algorithm>
#include <ctime>
#include <map>
#include <iostream>
#include <string>

using namespace geometrycentral;
using namespace geometrycentral::surface;
using namespace std;

int main() {

  /* Generate Mesh from Costom Data */
  // std::unique_ptr<ManifoldSurfaceMesh> Smesh;
  // std::unique_ptr<VertexPositionGeometry> Sgeometry;
  // vector<vector<size_t>> polygons{{0, 1, 2}, {2, 1, 3}};
  // vector<Vector3> vPos{{0, 0, 0}, {0, 1, 0}, {1, 0, 0}, {1, 1, 1}};
  // std::tie(Smesh, Sgeometry) = makeManifoldSurfaceMeshAndGeometry(polygons, vPos);
  /* Generate Mesh from Costom Data */

  std::unique_ptr<ManifoldSurfaceMesh> mesh;
  std::unique_ptr<VertexPositionGeometry> geometry;
  std::tie(mesh, geometry) =
      readManifoldSurfaceMesh("C:\\Users\\11612\\Desktop\\geometry-central-master\\test\\src\\cat_head.obj");
  Vertex vStart = mesh->vertex(90);
  Face fEnd = mesh->face(9);
  Vertex vEnd = *(fEnd.adjacentVertices().begin());
  std::vector<Vector3> path;
  Vector3 xb{0.117592221112593, -0.874280346968934, 0.470962784557004};

  // begin
  cout << "begin .." << endl;
  clock_t startTime, endTime;
  startTime = clock(); //计时开始

  // get start pose
  Vector3 xs{0, 0, 0};
  Vector3 Ends{0, 0, 0};
  for (Vertex v : fEnd.adjacentVertices()) {
    Ends += geometry->vertexPositions[v];
  }
  Ends /= 3;
  path.push_back(Ends);
  Halfedge dotH;
  double dotV = -2;
  vector<Halfedge> Surf_Path;
  /// get nearist vector dotH
  for (Halfedge he : vStart.outgoingHalfedges()) {
    Vector3 vhe = geometry->halfedgeVector(he);
    double dottemp = vhe[0] * xb[0] + vhe[1] * xb[1];
    dottemp /= sqrt(vhe[0] * vhe[0] + vhe[1] * vhe[1]);
    if (dottemp > dotV) {
      dotV = dottemp;
      dotH = he;
    }
  }
  /// get the other and cast xb down
  Vector3 a = geometry->halfedgeVector(dotH);
  Vector3 vhe = a + xb;
  dotV = vhe[0] * xb[0] + vhe[1] * xb[1];
  dotV /= sqrt(vhe[0] * vhe[0] + vhe[1] * vhe[1]);
  vhe = geometry->halfedgeVector(dotH.next().next().twin());
  vhe /= sqrt(vhe[0] * vhe[0] + vhe[1] * vhe[1]);
  if (dotV > 2 * (vhe[0] * xb[0] + vhe[1] * xb[1])) {
    Halfedge B = dotH.twin().next();
    Vector3 b = geometry->halfedgeVector(B);
    Vector3 ab = cross(a, b);
    xs = xb - dot(ab, xb) * ab / norm2(ab);
    Surf_Path.push_back(B.next());
  } else {
    Halfedge B = dotH.next();
    Vector3 ab = cross(a, vhe);
    xs = xb - dot(ab, xb) * ab / norm2(ab);
    Surf_Path.push_back(B);
  }

  // A* path search
  /// initialization
  double krho = 0.5;
  double kroll = 0.5;  // TODO: take roll into consideration (simple, just consider normal or and possible orientation)
  double kpitch = 0.5; // TODO: take pitch into consideration (simple, just consider normal or and possible orientation)
  double kapmax = 2;   // TODO: take kappa into consideration
  int kapnum = 2;

  constexpr double inf = 1 >> 20;
  struct GridNode;
  typedef GridNode* GridNodePtr;
  struct GridNode {
    Halfedge node;
    GridNodePtr parent = nullptr;
    double fScore = inf;
    double gScore = inf;
    double rho = 0;
    double heu = inf;
    Vector3 center;
    GridNode(const Halfedge h) : node(h) {}
    ~GridNode() {}
  };

  class NodeComparator {
  public:
    bool operator()(GridNodePtr node1, GridNodePtr node2) { return node1->fScore > node2->fScore; }
  };
  std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet;
  std::multimap<double, GridNodePtr> OS;
  vector<int> nodeState; // 0: unvisited; 1: openset; 2: closeset
  vector<GridNodePtr> buffer;
  for (size_t i = 0; i < mesh->nHalfedges(); i++) {
    nodeState.push_back(0);
    buffer.push_back(nullptr);
  }
  geometry->requireHalfedgeIndices();
  for (Halfedge he : mesh->halfedges()) {
    buffer[geometry->halfedgeIndices[he]] = new GridNode(he);
  }
  GridNodePtr startPtr = buffer[geometry->halfedgeIndices[Surf_Path[0]]];
  startPtr->fScore = startPtr->gScore = 0;
  startPtr->rho = 1;
  Vertex tipV = startPtr->node.tipVertex();
  Vertex tailV = startPtr->node.tailVertex();
  startPtr->center = 0.5 * (geometry->vertexPositions[tipV] + geometry->vertexPositions[tailV]);
  OS.insert(make_pair(startPtr->fScore, startPtr));
  //openSet.push(startPtr);

  /// begin graph search
  int cnt = 0;
  geometry->requireFaceNormals();
  while (!OS.empty()) {
    cnt += 1;
    //GridNodePtr current = openSet.top();
    auto iter = std::begin(OS);
    GridNodePtr current = iter->second;
    OS.erase(iter);
    //openSet.pop();
    nodeState[geometry->halfedgeIndices[current->node]] = 2;

    if (current->node.twin().face() == fEnd) {
      cout << cnt << endl;
      GridNodePtr p = current;
      while (p != nullptr) {
        path.push_back(p->center);
        p = p->parent;
      }
      path.push_back(geometry->vertexPositions[vStart]);
      reverse(path.begin(), path.end());
      break;
    }

    Halfedge p = current->node.twin();
    for (int i = 0; i < 4; i++, p = p.next()) {
      if (i == 2) {
        p = current->node;
      }
      size_t tempI = geometry->halfedgeIndices[p.next()];
      if (nodeState[tempI] == 0) {
        Vertex tipV = buffer[tempI]->node.tipVertex();
        Vertex tailV = buffer[tempI]->node.tailVertex();
        buffer[tempI]->center = 0.5 * (geometry->vertexPositions[tipV] + geometry->vertexPositions[tailV]);
      }
      //    if (current->parent != nullptr) {
      //      Vector2 one{current->center[0] - current->parent->center[0], current->center[1] -
      //      current->parent->center[1]}; Vector2 two{buffer[tempI]->center[0] - current->center[0],
      //      buffer[tempI]->center[1] - current->center[1]}; double none = norm(one); double ntwo = norm(two);
      //      // cout << dot(one, two) / normot << endl;
      //      if (dot(one, two) / (none*ntwo) < 0.17) continue;
      //      if (current->parent->parent != nullptr) {
      //        Vector2 three{current->parent->center[0] - current->parent->parent->center[0],
      //                      current->parent->center[1] - current->parent->parent->center[1]};
      //        double nthree = norm(three);
      //        double dtheta = dot(two, three) / (ntwo * nthree);
      //  if (dtheta/(none+ntwo+nthree) < -2) continue;
      //      }
      // else
      //{
      //        Vector2 three {current->center[0] - geometry->vertexPositions[vStart][0],
      //                      current->center[1] - geometry->vertexPositions[vStart][1]};
      //        double nthree = norm(three);
      //        double dtheta = dot(two, three) / (ntwo * nthree);
      //        if (dtheta / (none + ntwo + nthree) < -2) continue;
      //}
      //    } else {
      //      Vector2 one{current->center[0] - geometry->vertexPositions[vStart][0],
      //                  current->center[1] - geometry->vertexPositions[vStart][1]};
      //      // Vector2 one{xb[0], xb[1]};
      //      Vector2 two{buffer[tempI]->center[0] - current->center[0], buffer[tempI]->center[1] - current->center[1]};
      //      double normot = norm(one) * norm(two);
      //      // cout << "normot= " << normot << endl;
      //      cout << dot(one, two) / normot << endl;
      //      if (dot(one, two) / normot < 0.17) continue;
      //    }
      /// openSet
      if (nodeState[tempI] == 1) {
        double sgScore = current->gScore + (2 - buffer[tempI]->rho) * norm(current->center - buffer[tempI]->center);
        if (sgScore < buffer[tempI]->gScore) {
          buffer[tempI]->gScore = sgScore;
          buffer[tempI]->fScore = buffer[tempI]->gScore + buffer[tempI]->heu;
          buffer[tempI]->parent = current;
        }
        continue;
      }
      /// closeSet
      if (nodeState[tempI] == 2) {
        double sgScore = current->gScore + (2 - buffer[tempI]->rho) * norm(current->center - buffer[tempI]->center);
        if (sgScore < buffer[tempI]->gScore) {
          buffer[tempI]->gScore = sgScore;
          buffer[tempI]->fScore = buffer[tempI]->gScore + buffer[tempI]->heu;
          buffer[tempI]->parent = current;
          nodeState[tempI] = 1;
          //openSet.push(buffer[tempI]);
          OS.insert(make_pair(buffer[tempI]->fScore, buffer[tempI]));
        }
        continue;
      }
      /// unVisited
      if (nodeState[tempI] == 0) {
        /// compute rho
        // TODO: compute pitch, roll and kappa
        buffer[tempI]->rho = dot(geometry->faceNormal(buffer[tempI]->node.face()),
                                 geometry->faceNormal(buffer[tempI]->node.twin().face()));
        buffer[tempI]->gScore =
            current->gScore + (2 - buffer[tempI]->rho * krho) * norm(buffer[tempI]->center - current->center);
        // TODO: package the heuristic fuction and find a better one (may can add orientation error but need to do
        // experiments)
        buffer[tempI]->heu = norm(buffer[tempI]->center - Ends);
        buffer[tempI]->fScore = buffer[tempI]->gScore + buffer[tempI]->heu;
        buffer[tempI]->parent = current;
        nodeState[tempI] = 1;
        //openSet.push(buffer[tempI]);
        OS.insert(make_pair(buffer[tempI]->fScore, buffer[tempI]));
      }
    }
    // TODO: add time limitation
    /*ros::Time time_2 = ros::Time::now();
    if ((time_2 - time_1).toSec() > 0.2) {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
    }*/

    if (cnt > geometry->halfedgeIndices.size()) {
      cout << "A* SEARCH HAS LOOP, ERROR OCCURED!" << endl;
      break;
    }
  }
  geometry->unrequireFaceNormals();
  geometry->unrequireHalfedgeIndices();
  for (auto i : buffer) delete i;


  /******** GET PATH BY DIJKSTRA, BEZIER, FLIP ********/
  // std::unique_ptr<FlipEdgeNetwork> edgeNetwork;

  //// get path using dijkstra directly
  // edgeNetwork = FlipEdgeNetwork::constructFromDijkstraPath(*mesh, *geometry, vStart, vEnd);

  ///* Get Path using Bezier Curve*/
  ///*auto buf = edgeNetwork->getPathPolyline();
  // vector<Vertex> DijkPath;
  // for (auto i : buf[0])
  //{
  //  DijkPath.push_back(i.vertex);
  //}
  // edgeNetwork = FlipEdgeNetwork::constructFromPiecewiseDijkstraPath(*mesh, *geometry, DijkPath, false, true);
  // edgeNetwork->bezierSubdivide(3);*/
  ///* Get Path using Bezier Curve*/

  //// get path using flip edges
  // edgeNetwork->iterativeShorten();

  //// change path to points
  // edgeNetwork->posGeom = geometry.get();
  // vector<vector<Vector3>> ppath;
  // ppath = edgeNetwork->getPathPolyline3D();
  // ppath[0].push_back(Ends);
  /******** GET PATH BY DIJKSTRA, BEZIER, FLIP ********/


  // print runtime and result
  endTime = clock(); //计时结束
  cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
  for (auto j : path) {
    cout << j.x << " " << j.y << " " << j.z << endl;
  }
  return 0;
}


// TEST G-central
//#include <iostream>
//#include <string>
//
//#include "gtest/gtest.h"
//
// using std::cout;
// using std::endl;
//
// TEST(BasicTest, HelloWorldTest) {
//  int two = 2;
//  int four = two + two;
//  EXPECT_EQ(four, 4);
//}
//
// int main(int argc, char** argv) {
//  ::testing::InitGoogleTest(&argc, argv);
//  std::cout << "OK?" << endl;
//  return RUN_ALL_TESTS();
//}
