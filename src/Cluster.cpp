#include "Cluster.h"

const vector<double> Cluster::position(void) const
{
	return position_;
}

void Cluster::position(vector<double> newpos)
{
	position_ = newpos;
}

const vector<int> Cluster::indexes(void) const
{
	return contained_vertex_indexes_ ;
}

void Cluster::indexes(vector<int> newidx)
{
	contained_vertex_indexes_ = newidx;
}

void Cluster::addIndex(int index)
{
	contained_vertex_indexes_.push_back(index);
}

void Cluster::removeIndex(int target)
{
	for (auto iter = contained_vertex_indexes_.begin(); iter != contained_vertex_indexes_.end(); iter++) {
		if (*iter == target) {
			contained_vertex_indexes_.erase(iter);
		}
	}
	contained_vertex_indexes_.shrink_to_fit();
}
