#include "Grid.h"

Grid::Grid(int dimension, int num_points):
	dimension_(dimension)
{
	min_num_pts_ = int(nearbyint(pow(double(num_points), 0.5)));
	generateGrid();
}

vector<double> Grid::rawCoords() const
{
	return convertToRaw();
}

vector<vector<double>> Grid::points() const
{
	return points_;
}

int Grid::dimension(void) const
{
	return dimension_;
}

int Grid::num_pts(void) const
{
	return num_pts_;
}

void Grid::generateGrid()
{
	// calc the actual number of points and the needed divisions
	int divisions = int(floor(pow(double(min_num_pts_), (1.0/double(dimension_)))) + 1);
	num_pts_ = int(pow(double(divisions), double(dimension_)));
	double segment = 1.0 / double(divisions + 2);
	// for each indexed point
	// generate each dimension as a function of index
	// for each dimension
	// divide pt number by base ( divisions )
	// remainder = coordinate
	// dividend = number for next dimension
	for (int point_index = 0; point_index < num_pts_; point_index++) {
		int dividend = point_index;
		vector<double> this_point;
		for (int current_dimension = 0; current_dimension < dimension_; current_dimension++) {
			int remainder = dividend % divisions;
			double coord = double(remainder + 1) * segment;
			this_point.push_back(coord);
			dividend = dividend / divisions;
		}
		points_.push_back(this_point);
	}
}

vector<double> Grid::convertToRaw(void) const
{
	vector<double> raw_coords;

	raw_coords.reserve(num_pts() * dimension());

	for (auto pt_iter = points().begin(); pt_iter != points().end(); pt_iter++) {
		for (auto coord_iter = (*pt_iter).begin(); coord_iter != (*pt_iter).end(); coord_iter++) {
			raw_coords.push_back(*coord_iter);
		}
	}
		
	return raw_coords;
}

void NormalizedGrid::generateGrid(unsigned int idx, double remainder)
{
	if (idx == dimension() - 1) {
		curr_coord_.push_back(remainder);
		points().push_back(curr_coord_);
	}
	else if (idx == 0) {
		curr_coord_.clear();
	}
	double step = 0.25;
	double equal_threshold = 1.0e-6;
	for (
		double current_step = 0.0; 
		abs(remainder - current_step) > equal_threshold; 
		) 
	{
		curr_coord_.push_back(current_step);
		generateGrid(idx + 1, remainder - current_step);
		current_step += step;
		current_step = round(current_step * 10.0) * 0.1;
	}
}
