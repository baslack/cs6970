#include "MaxMinDistSet.h"

// public 

MaxMinDist::MaxMinDist(int dimension):
	dt_(dimension)
{
}

const vector<T::Point>& MaxMinDist::points() const
{
	// TODO: insert return statement here
	return points_;
}

int MaxMinDist::createPointsFromData(const vector<double>& data)
{
	// data supplied doesn't match the dimension
	if (data.size() % dimension() != 0) return -1;
	// break up the data into points
	points_.reserve(data.size() % dimension());
	for (auto iter = data.begin(); iter != data.end(); iter += dimension()) {
		T::Point p(iter, iter + dimension());
		points_.push_back(p);
	}
	// add the points to the triangulation
	T::Vertex_handle hint;
	int i = 0;
	for (auto pt_iter = points_.begin(); pt_iter != points_.end(); ++pt_iter) {
		std::cout << "Processing: " << *pt_iter << ", " << ++i << " of " << int(points_.size()) << std::endl;
		if (T::Vertex_handle() != hint) {
			hint = dt_.insert(*pt_iter, hint);
		}
		else {
			hint = dt_.insert(*pt_iter);
		}
	}
	//dt_.insert(points_.begin(), points_.end());
	return 0;
}

int MaxMinDist::dimension(void) const
{
	return dt_.maximal_dimension();
}

int MaxMinDist::size(void) const
{
	return int(points_.size());
}

// private

double MaxMinDist::localMinDist(const T::Point& other)
{
	return 0.0;
}

double MaxMinDist::globalMinDist(void)
{
	return 0.0;
}

double MaxMinDist::averageMinDist(void)
{
	return 0.0;
}

void MaxMinDist::iterateGlobalFPO(void)
{
}

void MaxMinDist::iterateLocalFPO(void)
{
}
/*
Assumes a periodic coordinate from 0 to 1
Returns the shortest distance between the two
*/
double MaxMinDist::toroidalDistanceOnAxis(double x1, double x2)
{
	double dist = x1 - x2;
	if (dist > 0.5) {
		dist = dist - 1.0;
	}
	return dist;
}

double MaxMinDist::toroidalSquaredDistance(const T::Point& x1, const T::Point& x2)
{
	double sqdist = 0.0;
	double dist = 0.0;
	int idx = 0;
	for (auto x1_iter = x1.cartesian_begin(); x1_iter != x1.cartesian_end(); x1_iter++, idx++) {
		dist = toroidalDistanceOnAxis(x1[idx], x2[idx]);
		sqdist += (dist * dist);
	}
	return sqdist;
}
