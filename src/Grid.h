#ifndef GRID
#define GRID

#include <vector>
#include <cmath>

using namespace std;

class Grid {
public:
	Grid(int, int);
	vector<double> rawCoords() const;
	vector<vector<double>> points() const;
	int dimension(void) const;
	int num_pts(void) const;
private:
	virtual void generateGrid(void);
	vector<double> convertToRaw(void) const;
	vector<vector<double>> points_;
	double min_range_ = 0.0;
	double max_range_ = 1.0;
	int dimension_;
	int min_num_pts_;
	int num_pts_;
};

class NormalizedGrid : public Grid {
private:
	//virtual void generateGrid(void);
	void generateGrid(unsigned int, double);
	vector<double> curr_coord_;
};
#endif
