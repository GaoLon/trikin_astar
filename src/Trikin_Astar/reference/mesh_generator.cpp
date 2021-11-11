#include <CGAL/Simple_cartesian.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/jet_smooth_point_set.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/property_map.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <fstream>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
typedef Kernel::Sphere_3 Sphere_3;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Point_set_3<Point_3, Vector_3> Point_set;

typedef std::array<std::size_t, 3> Facet;

namespace std {
	std::ostream&
		operator<<(std::ostream& os, const Facet& f)
	{
		os << "3 " << f[0] << " " << f[2] << " " << f[1];
		return os;
	}
}
namespace SMS = CGAL::Surface_mesh_simplification;

struct Perimeter {
	double bound;
	Perimeter(double bound)
		: bound(bound)
	{}
	template <typename AdvancingFront, typename Cell_handle>
	double operator() (const AdvancingFront& adv, Cell_handle& c,
		const int& index) const
	{
		// bound == 0 is better than bound < infinity
		// as it avoids the distance computations
		if (bound == 0) {
			return adv.smallest_radius_delaunay_sphere(c, index);
		}
		// If perimeter > bound, return infinity so that facet is not used
		double d = 0;
		d = sqrt(squared_distance(c->vertex((index + 1) % 4)->point(),
			c->vertex((index + 2) % 4)->point()));
		if (d > bound) return adv.infinity();
		d += sqrt(squared_distance(c->vertex((index + 2) % 4)->point(),
			c->vertex((index + 3) % 4)->point()));
		if (d > bound) return adv.infinity();
		d += sqrt(squared_distance(c->vertex((index + 1) % 4)->point(),
			c->vertex((index + 3) % 4)->point()));
		if (d > bound) return adv.infinity();
		// Otherwise, return usual priority value: smallest radius of
		// delaunay sphere
		return adv.smallest_radius_delaunay_sphere(c, index);
	}
};

int main(int argc, char*argv[])
{
	// Read point clouds file
	Point_set ps;
	std::vector<Point_3> points;
	std::vector<Facet> facets;
	std::ifstream stream("a.ply", std::ios_base::binary);
	if (!stream)
	{
		std::cerr << "Error: cannot read file " << std::endl;
		return EXIT_FAILURE;
	}
	stream >> ps;
	for (auto i : ps.points())
	{
		if (fabs(i.z())<0.4 && fabs(i.x()) < 5 && i.y() < 2 && i.y() > -5)
			points.push_back(Point_3(i.x(), i.y(), i.z()));
	}
	std::cout << "Read " << points.size() << " point(s)" << std::endl;
	if (points.empty())
		return EXIT_FAILURE;

	// Remove outliers
	const int nb = 24;
	const double rp = 10.0;
	points.erase(CGAL::remove_outliers<CGAL::Sequential_tag>
		(points,nb,CGAL::parameters::threshold_percent(rp).threshold_distance(0.)),points.end());
	std::cout << points.size() << std::endl;

	// Compute average spacing, and simplify
	double spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, 6);
	points.erase(CGAL::grid_simplify_point_set(points, 2*spacing), points.end());
	std::cout << points.size() << std::endl;

	// Smooth
	CGAL::jet_smooth_point_set<CGAL::Sequential_tag>(points, nb);	

	Perimeter perimeter(3);
	double rrb = 5.0;

	// Reconstruction
	unsigned int reconstruction_choice = 0;
	CGAL::advancing_front_surface_reconstruction(points.begin(),
		points.end(),
		std::back_inserter(facets),perimeter,rrb);

	// Output
	std::vector<Point_3> vertices;
	vertices.reserve(points.size());
	std::copy(points.begin(), points.end(), std::back_inserter(vertices));
	Polyhedron output_mesh;
	CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(vertices, facets, output_mesh);
	std::ofstream f("out.off");
	f << output_mesh;
	f.close();


	//// Simplification
	//Polyhedron output_mesh;
	//const char* filename = (argc > 1) ? argv[1] : "advance.off";
	//std::ifstream is(filename);
	//if (!is || !(is >> output_mesh))
	//{
	//	std::cerr << "Failed to read input mesh: " << filename << std::endl;
	//	return EXIT_FAILURE;
	//}
	//if (!CGAL::is_triangle_mesh(output_mesh))
	//{
	//	std::cerr << "Input geometry is not triangulated." << std::endl;
	//	return EXIT_FAILURE;
	//}
	//const std::size_t edge_count_treshold = output_mesh.size_of_halfedges / 4;
	//SMS::Count_stop_predicate<Polyhedron> stop(edge_count_treshold);
	//std::cout << "Collapsing edges, aiming for " << edge_count_treshold << " final edges..." << std::endl;
	//int r = SMS::edge_collapse(output_mesh, stop,
	//	CGAL::parameters::vertex_index_map(get(CGAL::vertex_external_index, output_mesh))
	//	.halfedge_index_map(get(CGAL::halfedge_external_index, output_mesh)));
	//std::cout << "\nFinished!\n" << r << " edges removed.\n"
	//	<< (output_mesh.size_of_halfedges() / 2) << " final edges.\n";
	//std::ofstream os("simple.off");
	//os.precision(17);
	//os << output_mesh;
	//os.close();

	return 0;
}