#include "FindANN.h"

FindANN::FindANN(const double* rawPts, int dimension, int numPts) :
	dimension_(dimension),
	nPts_(numPts)
{
	dataPts_ = annAllocPts(int(nPts_), int(dimension_));
	nnIdx_ = new ANNidx[k_];
	dists_ = new ANNdist[k_];
	// populate the dataPts array
	for (int i = 0; i < nPts_; i++) {
		for (int j = 0; j < dimension_; j++) {
			*(dataPts_[i] + j) = rawPts[i*dimension_+j];
		}
	}
	kdTree_ = new ANNkd_tree(dataPts_, int(nPts_), int(dimension_));
}

FindANN::FindANN(const vector<vector<double>>& pts)
{
	nPts_ = pts.size();
	dimension_ = pts[0].size();
	dataPts_ = annAllocPts(int(nPts_), int(dimension_));
	nnIdx_ = new ANNidx[k_];
	dists_ = new ANNdist[k_];
	int i = 0;
	for (auto pt_iter = pts.cbegin(); pt_iter != pts.cend(); pt_iter++, i++) {
		int j = 0;
		for (auto coord_iter = (*pt_iter).cbegin(); coord_iter != (*pt_iter).cend(); coord_iter++, j++) {
			*(dataPts_[i] + j) = *coord_iter;
		}
	}
	kdTree_ = new ANNkd_tree(dataPts_, int(nPts_), int(dimension_));
}

FindANN::~FindANN()
{
	delete[] nnIdx_;
	delete[] dists_;
	delete kdTree_;
	annClose();
}

int FindANN::search(const vector<double>& pt)
{
	//queryPt_ = annAllocPt(int(dimension_));
	ANNpoint queryPt = annAllocPt(int(dimension_));
	ANNidxArray nnIdx = new ANNidx[k_];
	ANNdistArray dists = new ANNdist[k_];
	int i = 0;
	for (auto coord_iter = pt.cbegin(); coord_iter != pt.cend(); coord_iter++, i++) {
		*(queryPt + i) = *coord_iter;
	}
	//kdTree_->annkSearch(queryPt, k_, nnIdx_, dists_, error_bound_);
	kdTree_->annkSearch(queryPt, k_, nnIdx, dists, error_bound_);
	//return int(nnIdx_[0]);
	int ret_val = nnIdx[0];
	annDeallocPt(queryPt);
	delete[] nnIdx;
	delete[] dists;
	return ret_val;
	//return int(nnIdx[0]);
}
