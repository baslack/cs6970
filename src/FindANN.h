#ifndef FIND_ANN
#define FIND_ANN

#include <ANN\ANN.h>
#include <vector>

using namespace std;

class FindANN {
public:
	FindANN(const double*, int, int);
	FindANN(const vector<vector<double>>&);
	~FindANN();
	int search(const vector<double>&);
private:
	int k_ = 1;
	size_t dimension_ = 2;
	double error_bound_ = 1.0e-6;
	size_t nPts_;
	ANNpointArray dataPts_;
	ANNidxArray nnIdx_;
	ANNdistArray dists_;
	ANNkd_tree* kdTree_;
	ANNpoint queryPt_;
};

#endif