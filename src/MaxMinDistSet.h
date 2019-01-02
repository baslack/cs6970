#ifndef MAX_MIN_DIST_SET

#define MAX_MIN_DIST_SET
#define CGAL_EIGEN3_ENABLED
#define DllExport   __declspec( dllexport )

#include <boost/container/vector.hpp>
//#include <vector>
#include <CGAL\Epick_d.h>
#include <CGAL\Delaunay_triangulation.h>
#include <iostream>

using namespace boost::container;
//using namespace std;

typedef CGAL::Triangulation<CGAL::Epick_d<CGAL::Dynamic_dimension_tag>> T;

class DllExport MaxMinDist
{
public:
	MaxMinDist(int);
	const vector<T::Point>& points() const;
	int createPointsFromData(const vector<double>&);
	int dimension(void) const;
	int size(void) const;
private:
	T dt_;
	vector<T::Point> points_;
	double localMinDist(const T::Point&);
	double globalMinDist(void);
	double averageMinDist(void);
	void iterateGlobalFPO(void);
	void iterateLocalFPO(void);
	double toroidalDistanceOnAxis(double, double);
	double toroidalSquaredDistance(const T::Point&, const T::Point&);
};

#endif