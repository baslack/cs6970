#ifndef CLUSTER
#define CLUSTER

#include <vector>

using namespace std;

class Cluster {
public:
	const vector<double> position(void) const;
	void position(vector<double>);
	const vector<int> indexes(void) const;
	void indexes(vector<int>);
	void addIndex(int);
	void removeIndex(int);
private:
	vector<double> position_;
	vector<int> contained_vertex_indexes_;
};

#endif