#include "corSkinCmd.h"

double corSkinCmd::w_ = 0.1;
double corSkinCmd::e_ = 0.1;
double corSkinCmd::o_ = 0.1;
double corSkinCmd::E_ = 1.0e-3;

const char* corSkinCmd::kName = "corSkinCmd";
const char* corSkinCmd::kPrecompFlag = "precomp";
const char* corSkinCmd::kNaivePrecompFlag = "naive";
const char* corSkinCmd::kDeformerNameFlag = "deformer";
const char* corSkinCmd::kPrecompFlagShort = "p";
const char* corSkinCmd::kNaivePrecompFlagShort = "n";
const char* corSkinCmd::kDeformerNameFlagShort = "d";
const char* corSkinCmd::kMultithreadFlag = "multithread";
const char* corSkinCmd::kMultithreadFlagShort = "m";

corSkinCmd::corSkinCmd()
	:name_("corSkinCluster#")
{

}

corSkinCmd::~corSkinCmd() {
	if (ann_ != nullptr) {
		delete(ann_);
	}
}

void* corSkinCmd::creator() {
	return new corSkinCmd;
}

bool corSkinCmd::isUndoable()
{
	return true;
}

MSyntax corSkinCmd::newSyntax() {
	MStatus status;
	MSyntax syntax;
	status = syntax.addFlag(kPrecompFlagShort, kPrecompFlag, MSyntax::kNoArg);
	if (status.error()) {
		MGlobal::displayError(status.errorString());
	}
	status = syntax.addFlag(kNaivePrecompFlagShort, kNaivePrecompFlag, MSyntax::kNoArg);
	if (status.error()) {
		MGlobal::displayError(status.errorString());
	}
	status = syntax.addFlag(kDeformerNameFlagShort, kDeformerNameFlag, MSyntax::kString);
	if (status.error()) {
		MGlobal::displayError(status.errorString());
	}
	status = syntax.addFlag(kMultithreadFlagShort, kMultithreadFlag, MSyntax::kUnsigned);
	if (status.error()) {
		MGlobal::displayError(status.errorString());
	}
	//syntax.setObjectType(MSyntax::kStringObjects, 1, 255);
	status = syntax.setObjectType(MSyntax::kSelectionList, 0, 255);
	if (status.error()) {
		MGlobal::displayError(status.errorString());
	}
	syntax.useSelectionAsDefault(true);
	return syntax;
}

typedef struct subdivedVertex {
	MPoint position;
	vector<double> weights;
	int original_index;
}t_subdividedVertex;

MStatus corSkinCmd::generateSubdivMesh() {
	MStatus status;
	// generate subdiv mesh?
	MObject subdivMesh;
	MFnMeshData fnMeshData;
	subdivMesh = fnMeshData.create(&status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MFnMesh fnMesh(subdivMesh, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	fnMesh.copy(oInGeo_);
	// create mapping from original verts to
	// subdiv verts
	// attached weights to subdiver verts
	vector<t_subdividedVertex> sdVerts;
	MItGeometry v_iter(oInGeo_, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	//v_iter.reset();
	sdVerts.reserve(v_iter.exactCount());
	for (int i = 0; i < sdVerts.size(); i++, v_iter.next()) {
		sdVerts[i].position = v_iter.position();
		sdVerts[i].weights = weights_[i];
		sdVerts[i].original_index = i;
	}

	// subdive edges by l2 weight distance metric
	return status;
}

MStatus corSkinCmd::getInputGeo() {
	MStatus status;
	MFnDependencyNode fnDG(deformer_, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug inputPlug = fnDG.findPlug(corSkinCluster::input, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug inputGeoPlug = inputPlug[0].child(corSkinCluster::inputGeom, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = inputGeoPlug.getValue(oInGeo_);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MFnSkinCluster fnSkin(deformer_, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MObjectArray inGeo_ar;
	status = fnSkin.getInputGeometry(inGeo_ar);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MFnDagNode fnDAG(inGeo_ar[0], &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = fnDAG.getPath(pInGeoPath_);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return MS::kSuccess;
}

MStatus corSkinCmd::getVertexWeights() {
	MStatus status;
	MFnDependencyNode fnDG(deformer_, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug weightListPlug = fnDG.findPlug(corSkinCluster::weightList, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	weights_.reserve(weightListPlug.numElements());
	unsigned int num_bones;
	status = getNumBones(num_bones);
	for (unsigned int i = 0; i < weightListPlug.numElements(); i++) {
		MPlug weightsPlug = weightListPlug[i].child(corSkinCluster::weights, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		vector<double> extracted_weights;
		extracted_weights.reserve(num_bones);
		for (unsigned int j = 0; j < num_bones; j++) {
			double weight_val = 0.0;
			MPlug weightPlug = weightsPlug.elementByLogicalIndex(j, &status);
			if (!status.error()) {
				weight_val = weightPlug.asDouble();
			}
			extracted_weights.push_back(weight_val);
		}
		weights_.push_back(extracted_weights);
	}
	return MS::kSuccess;
}

MStatus corSkinCmd::getNumBones(unsigned int& num_bones) {
	MStatus status;
	MPlug bones_matricies;
	MFnDependencyNode fnDG(deformer_, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	bones_matricies = fnDG.findPlug(corSkinCluster::matrix, &status);
	num_bones = bones_matricies.evaluateNumElements(&status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return status;
}

MStatus corSkinCmd::generateANNSearchStructure() {
	// generate a grid
	// generate an ANN
	MStatus status;
	size_t num_weights = weights_.size();
	unsigned int num_bones = 0;
	status = getNumBones(num_bones);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	// create the grid
	Grid g((int)num_bones, (int)num_weights);
	
	// reserve the needed clusters
	clusters_.reserve(g.points().size());
	
	// create a cluster for each point in the grid
	int i = 0;
	vector<vector<double>> these_points = g.points();
	for (auto pt_iter = these_points.begin(); pt_iter != these_points.end(); pt_iter++, i++) {
		Cluster new_cluster;
		new_cluster.position((*pt_iter));
		clusters_.push_back(new_cluster);
	}
	
	// setup the ANN search for the grid points
	ann_ = new FindANN(g.points());

	// map each weight to a cluster
	i = 0;
	for (auto wt_iter = weights_.cbegin(); wt_iter != weights_.cend(); wt_iter++, i++) {
		int cluster_index = ann_->search((*wt_iter));
		clusters_[cluster_index].addIndex(i);
	}
	return MS::kSuccess;
}

MStatus corSkinCmd::setupNaivePrecomp() {
	MStatus status;
	// from the input geometry
	status = getInputGeo();
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// get the verticies
	MItGeometry v_iter(pInGeoPath_, MObject::kNullObj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = v_iter.allPositions(verts_);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	//reserve space for the COR
	cor_.setLength(verts_.length());

	// get the vertex weights
	status = getVertexWeights();
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// cache the areas, vert postition and avg weights
	status = cachePrecompSupport();
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return MS::kSuccess;
}

MStatus corSkinCmd::setupPrecomp() {
	MStatus status;
	// from the input geometry
	status = getInputGeo();
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// get the verticies
	MItGeometry v_iter(pInGeoPath_, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = v_iter.allPositions(verts_);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	//reserve space for the COR
	status = cor_.setLength(verts_.length());
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// get the vertex weights
	status = getVertexWeights();
	CHECK_MSTATUS_AND_RETURN_IT(status);

	//generateSubdivMesh(); TODO, not sure if I can implement this leaving for future

	status = generateANNSearchStructure();
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// cache the areas, vert postition and avg weights
	status = cachePrecompSupport();
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return MS::kSuccess;
}

MStatus corSkinCmd::parseArgs(const MArgList& args) {
	MStatus status;
	MArgDatabase db(syntax(), args, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = db.getObjects(sl_);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	bPrecomp = db.isFlagSet(corSkinCmd::kPrecompFlag, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	bNPrecomp = db.isFlagSet(corSkinCmd::kNaivePrecompFlag, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	bool bMulti = false;
	bMulti = db.isFlagSet(corSkinCmd::kMultithreadFlag, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	if (bMulti) {
		status = db.getFlagArgument(corSkinCmd::kMultithreadFlag, 0, iMT);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		if (iMT < 1) {
			iMT = 1;
		}
	}
	bDeformerSpecified = db.isFlagSet(corSkinCmd::kDeformerNameFlag, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	if (bDeformerSpecified) {
		status = db.getFlagArgument(corSkinCmd::kDeformerNameFlag, 0, name_);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}
	return MStatus::kSuccess;
}

MStatus corSkinCmd::parseDAGPaths() {
	MStatus status;
	//bones and meshes
	//mesh will be last
	status = sl_.getDagPath(sl_.length() - 1, mesh_dag_);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	//for (unsigned i = 0; i < sl_.length(); i++) {
	//	MDagPath temp_path;
	//	status = sl_.getDagPath(int(i), temp_path);
	//	CHECK_MSTATUS_AND_RETURN_IT(status);
	//	MString partial_name = temp_path.partialPathName(&status);
	//	CHECK_MSTATUS_AND_RETURN_IT(status);
	//	const char* partial_name_str = partial_name.asChar();
	//}
	status = getShapeNode(mesh_dag_);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	if (!mesh_dag_.node().hasFn(MFn::kMesh)) {
		MGlobal::displayError("corSkinCluster can only be applied to kMesh objects");
		return MStatus::kFailure;
	}

	MItSelectionList iter_sl(sl_, MFn::kJoint, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	bone_dag_ar_.clear();
	while (!iter_sl.isDone()) {
		MDagPath this_joint;
		status = iter_sl.getDagPath(this_joint);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = bone_dag_ar_.append(this_joint);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = iter_sl.next();
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}

	return MStatus::kSuccess;
}

bool corSkinCmd::isShapeNode(MDagPath& path) {
	return path.node().hasFn(MFn::kMesh);
}

MStatus corSkinCmd::getShapeNode(MDagPath& path, bool intermediate) {
	MStatus status;

	MString path_name = path.partialPathName(&status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	const char* path_name_str = path_name.asChar();

	if (path.hasFn(MFn::kTransform)) {
		unsigned int shape_nodes_count = 0;
		status = path.numberOfShapesDirectlyBelow(shape_nodes_count);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		if (isShapeNode(path)) {
			path.pop();
		}

		for (unsigned int i = 0; i < shape_nodes_count; i++) {
			status = path.extendToShapeDirectlyBelow(i);
			CHECK_MSTATUS_AND_RETURN_IT(status);
			MFnDagNode fnDAG(path, &status);
			CHECK_MSTATUS_AND_RETURN_IT(status);
			if ((!fnDAG.isIntermediateObject() && !intermediate) ||
				(fnDAG.isIntermediateObject() && intermediate)) {
				return MStatus::kSuccess;
			}
			path.pop();
		}
	}

	return MStatus::kFailure;
}

MStatus corSkinCmd::populateDeformerFromName() {
	// extract the deformer node from its name
	MStatus status;
	MSelectionList temp_sl;
	status = temp_sl.add(name_);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = temp_sl.getDependNode(0, deformer_);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return MStatus::kSuccess;
}

MStatus corSkinCmd::connectJointToCluster(unsigned int idx) {
	MString long_name = "lockInfluenceWeights";
	MString short_name = "liw";
	MString command = "";
	MStatus status;
	const MDagPath& bone_dag = bone_dag_ar_[idx];
	//get the bone's dg node
	MObject oBone;
	oBone = bone_dag.node(&status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MFnDependencyNode fnDG(oBone, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	bool bHas_liw = fnDG.hasAttribute(long_name, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	if (!bHas_liw) {
		// add via API, this can likely be dumped
		MFnNumericAttribute fnNA;
		fnNA.create(long_name, short_name, MFnNumericData::kBoolean, 0.0, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MObject oAttr = fnNA.object(&status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = dgMod_.addAttribute(oBone, oAttr);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		/*
		// add via MEL and dgMod, so can undo
		command += "select -r " + bone_dag.partialPathName(&status) + ";";
		CHECK_MSTATUS_AND_RETURN_IT(status);
		command += "addAttr - sn \"" + short_name + "\" - ln \"" + long_name + "\" - at \"bool\";";
		status = dgMod_.commandToExecute(command);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		*/
	}
	//connect attrs
	// connect lockWeights (lockWeights created at bind time, apparently)
	command = "connectAttr ";
	command += bone_dag.partialPathName(&status) + "." + short_name + " ";
	command += name_ + ".lockWeights[" + idx + "];";
	status = dgMod_.commandToExecute(command);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	//connect world matrices
	command = "connectAttr ";
	command += bone_dag.partialPathName(&status) + ".worldMatrix[0] ";
	CHECK_MSTATUS_AND_RETURN_IT(status);
	command += name_ + ".matrix[" + idx + "];";
	status = dgMod_.commandToExecute(command);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	//connect influence color
	command = "connectAttr ";
	command += bone_dag.partialPathName(&status) + ".objectColorRGB ";
	CHECK_MSTATUS_AND_RETURN_IT(status);
	command += name_ + ".influenceColor[" + idx + "];";
	status = dgMod_.commandToExecute(command);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	// set preBindMatrix
	// get world inverse matrix off the bone
	MPlug pWim = fnDG.findPlug("worldInverseMatrix", &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	//unsigned int num_elements = pWim.numElements();
	//const char* object_name = fnDG.name().asChar();
	//bool bIsArray = pWim.isArray();
	MObject oWim;
	status = pWim.elementByLogicalIndex(0).getValue(oWim);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	//MFnMatrixData fnMtxData(oWim, &status);
	//CHECK_MSTATUS_AND_RETURN_IT(status);
	//const MMatrix wim = fnMtxData.matrix(&status);
	//CHECK_MSTATUS_AND_RETURN_IT(status);
	// set the appropriate preBindAttr on the cluster
	status = fnDG.setObject(deformer_);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug pBPM_ar = fnDG.findPlug(corSkinCluster::bindPreMatrix, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug pBPM_i = pBPM_ar.elementByLogicalIndex(idx, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = dgMod_.newPlugValue(pBPM_i, oWim);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = dgMod_.doIt();
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return MS::kSuccess;
 }

MStatus corSkinCmd::createDeformer() {
	MStatus status;
	MString command = "deformer -type " + corSkinCluster::kname + " -n \"" + name_ + "\"";
	const char* command_str = command.asChar();
	// create deformer add to the mesh
	command += " " + mesh_dag_.partialPathName();
	command_str = command.asChar();
	status = dgMod_.commandToExecute(command);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = dgMod_.doIt();
	CHECK_MSTATUS_AND_RETURN_IT(status);
	// get the deformer now that it exists
	MItDependencyGraph itDG(
		mesh_dag_.node(), 
		MFn::kGeometryFilt, 
		MItDependencyGraph::Direction::kUpstream,
		MItDependencyGraph::Traversal::kDepthFirst,
		MItDependencyGraph::Level::kNodeLevel,
		&status
	);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	bool bFoundDeformer = false;
	while (!bFoundDeformer && !itDG.isDone()) {
		MObject this_obj = itDG.currentItem(&status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MFnDependencyNode fnNode(this_obj, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		if (fnNode.typeId() == corSkinCluster::id) {
			bFoundDeformer = true;
			deformer_ = this_obj;
			name_ = fnNode.name(&status);
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}
	}
	// add each bone to the mesh in turn
	for (unsigned int i = 0; i < bone_dag_ar_.length(); i++) {
		status = connectJointToCluster(i);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}
	// set max influences
	unsigned int max_inf;
	max_inf = bone_dag_ar_.length();
	if (max_inf > 5) max_inf = 5;
	command = "skinCluster -e -maximumInfluences ";
	command += to_string(max_inf).data();
	command += " " + name_ + ";";
	const char* temp_cmd = command.asChar();
	status = dgMod_.commandToExecute(command);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = dgMod_.doIt();
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return MS::kSuccess;
}

MStatus corSkinCmd::doIt(const MArgList& args)
{
	
	// rework for creating deformer
	MStatus status;
	
	// parse the command line args
	status = parseArgs(args);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// if deformer not specified, create deformer from selection list
	if (!bDeformerSpecified) {
		status = parseDAGPaths();
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = createDeformer();
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}
	// else populate deformer from specified
	else {
		status = populateDeformerFromName();
		if (status.error()) {
			MGlobal::displayError(status.errorString());
		}
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}
	// whether deformer specified or not, need to do a precomp
	if (bNPrecomp) {
		status = setupNaivePrecomp();
	}
	else if(bPrecomp) {
		status = setupPrecomp();
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}
	dgMod_.doIt();
	redoIt();
	return MStatus::kSuccess;
}

double corSkinCmd::l2_distance(const vector<double>& wt_a, const vector<double>& wt_b) {
	double l2 = 0.0;
	for (int i = 0; i < wt_a.size(); i++) {
		double sq = wt_b[i] - wt_a[i];
		sq *= sq;
		l2 += (sq);
	}
	return l2;
}

double corSkinCmd::similarity(const vector<double>& Wp, const vector<double>& Wv)
{
	double similarity = 0;
	for (int j = 0; j < Wp.size(); j++) {
		for (int k = 0; k < Wv.size(); k++) {
			if (!(j == k)) {
				double temp = 0.0;
				temp = (Wp[j] * Wp[k] * Wv[j] * Wv[k]);
				double exp = 0.0;
				exp = Wp[j] * Wv[k] - Wp[k] * Wv[j];
				exp *= exp;
				exp /= (o_ * o_);
				exp *= -1;
				exp = std::exp(exp);
				similarity += temp * exp;
			}
		}
	}
	return similarity;
}

MPoint corSkinCmd::averagePosition(const MPointArray& ar)
{
	MPoint avgPos(0,0,0);
	for (unsigned int i = 0; i < ar.length(); i++) {
		avgPos += ar[i];
	}
	avgPos.x /= ar.length();
	avgPos.y /= ar.length();
	avgPos.z /= ar.length();

	return avgPos;
}

vector<double> corSkinCmd::averageWeight(const MIntArray& ar)
{
	MStatus status;
	vector<double> avgWeights;
	unsigned int num_bones = 0;
	status = getNumBones(num_bones);
	avgWeights.reserve(num_bones);
	for (unsigned int i = 0; i < num_bones; i++) {
		avgWeights.push_back(0.0);
	}
	for (unsigned int i = 0; i < ar.length(); i++) {
		for (unsigned int j = 0; j < num_bones; j++) {\
			int idx = ar[i];
			vector<double> these_weights = weights_[idx];
			avgWeights[j] += these_weights[j];
		}
	}
	for (unsigned int j = 0; j < num_bones; j++) {
		avgWeights[j] /= ar.length();
	}
	return avgWeights;
}

double corSkinCmd::area(const MPointArray& ar)
{
	MVector a(ar[0]);
	MVector b(ar[1]);
	MVector c(ar[2]);
	MVector ab = b - a;
	MVector ac = c - a;
	double area = 0.0;
	area = 0.5 * (ab ^ ac).length();
	return area;
}

MStatus corSkinCmd::growTheSeeds(int i, set<int>& grown_to) {
	MStatus status;
	MItMeshVertex mv_iter(oInGeo_, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	queue<int> process_queue;
	for (auto it = grown_to.cbegin(); it != grown_to.cend(); it++) {
		process_queue.push(*it);
	}
	while (process_queue.size() > 0) {
		int next, last = 0;
		next = process_queue.front();
		process_queue.pop();
		// with the index popped of the queue, set the iterator to that index
		status = mv_iter.setIndex(next, last);
		MIntArray adjacent_verts;
		// get the connected verts
		status = mv_iter.getConnectedVertices(adjacent_verts);
		// for each connected vert
		for (unsigned int j = 0; j < adjacent_verts.length(); j++) {
			// is it already in the set of grown seeds?
			auto found_iter = grown_to.find(adjacent_verts[j]);
			if (found_iter == grown_to.end()) {
				// this vert isn't in the grown to
				// calc it's similarity to the main point
				double s = similarity(weights_[i], weights_[adjacent_verts[j]]);
				// if less than threshold, add it to the grown to
				if (s >= E_) {
					// add it to the process queue
					process_queue.push(adjacent_verts[j]);
					// add it to the seeds
					grown_to.insert(adjacent_verts[j]);
				}
			}
			else {
				continue;
			} //end if
		} // end adj verts iter
	} // end while
	
	return MS::kSuccess;
}

MStatus corSkinCmd::calcCOR(int i, set<int>& grown_seeds, MPoint& cor) {
	// for each unique polygon connected to seeds
	// for each triange of said polygon
	// calculate cor contribution

	// get faces
	MStatus status;
	if (earlyExit(i)) {
		cor = MPoint(0, 0, 0);
		return MS::kSuccess;
	}

	MItMeshVertex v_iter(pInGeoPath_, MObject::kNullObj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	set<int>face_idx_set;
	int next, last = 0;
	for (auto it = grown_seeds.cbegin(); it != grown_seeds.cend(); it++) {
		next = *it;
		v_iter.setIndex(next, last);
		MIntArray face_indexes;
		status = v_iter.getConnectedFaces(face_indexes);
		for (unsigned int j = 0; j < face_indexes.length(); j++) {
			face_idx_set.insert(face_indexes[j]);
		}
	}

	// for each face, calculate each triangle's contribution
	MItMeshPolygon p_iter(pInGeoPath_, MObject::kNullObj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	MPoint top(0, 0, 0);
	double bot = 0;

	for (auto it = face_idx_set.cbegin(); it != face_idx_set.cend(); it++) {
		next = *it;
		p_iter.setIndex(next, last);
		int num_tri;
		status = p_iter.numTriangles(num_tri);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		for (int k = 0; k < num_tri; k++) {
			MPointArray t_pts;
			MIntArray idx_pts;
			status = p_iter.getTriangle(k, t_pts, idx_pts);
			// need triange area
			//double tri_area = area(t_pts);  //commented out to try caching
			double tri_area = cached_tri_areas_[next][k];
			// need average triange weights
			//vector<double> avg_weights = averageWeight(idx_pts);  //comment out to try caching;
			vector<double> avg_weights = cached_avg_weights_[next][k];
			// need average position
			//MPoint avgPos = averagePosition(t_pts);  // commented out to try caching
			MPoint avgPos = cached_avg_tri_positions_[next][k];
			// need sim weight to avg
			double w_to_avg_sim = similarity(weights_[i], avg_weights);
			if (w_to_avg_sim != 0) {
				top += (w_to_avg_sim * avgPos * tri_area);
				bot += (w_to_avg_sim * tri_area);
			}  // end if
		} // end for, for number of triangles in face
	} // end for, for face iteration
	if (bot != 0) {
		cor = top / bot;
	}
	else {
		cor = MPoint(0, 0, 0);
	}
	return MStatus::kSuccess;
}

MStatus corSkinCmd::setCOR()
{
	MStatus status;
	MFnDependencyNode fnDG(deformer_, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug cor_plug = fnDG.findPlug(corSkinCluster::cor_ar, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// put the data into an MObject so you can set the plug
	MFnPointArrayData fnPAD;
	MObject cor_data;
	cor_data = fnPAD.create(&status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = fnPAD.set(cor_);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// set the plug
	status = cor_plug.setValue(cor_data);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return MS::kSuccess;
}

MStatus corSkinCmd::setValid(bool isValid)
{
	MStatus status;
	MFnDependencyNode fnDG(deformer_, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MPlug validPlug = fnDG.findPlug(corSkinCluster::cor_valid, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = validPlug.setValue(isValid);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return MS::kSuccess;
}

MStatus corSkinCmd::startTimer()
{
	time(&start);
	return MS::kSuccess;
}

MStatus corSkinCmd::stopTimer()
{
	time(&end);
	return MS::kSuccess;;
}

MStatus corSkinCmd::readTimer(unsigned int &microseconds)
{
	double diff = difftime(end, start);
	diff *= 1e6;
	microseconds = unsigned int(nearbyint(diff));
	return MS::kSuccess;
}

MStatus corSkinCmd::readTimer(double &seconds)
{
	seconds = difftime(end, start);
	return MS::kSuccess;
}

bool corSkinCmd::earlyExit(unsigned int idx)
{
	double nearness_threshold = 1.0e-6;
	double target_val = 1.0;
	for (auto wt_iter = weights_[idx].cbegin(); wt_iter != weights_[idx].cend(); wt_iter++) {
		double diff = *wt_iter - target_val;
		diff *= diff;
		diff = pow(diff, 0.5);
		if (diff <= nearness_threshold) {
			return true;
		}
	}
	return false;
}


MStatus corSkinCmd::performNaivePrecomp() {
	MStatus status;
	status = setValid(false);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	int max_num_proc = 0;
	int num_proc = 0;
	max_num_proc = std::thread::hardware_concurrency();
	if (max_num_proc == 0) {
		num_proc = iMT;
	}
	else if (int(iMT) <= max_num_proc) {
		num_proc = iMT;
	}
	else {
		num_proc = max_num_proc;
	}

	int end = verts_.length();
	int j = 0;
	
	// naive, we're getting all the triangles
	set<int> all_vert_idx;
	for (int i = 0; i < end; i++) {
		all_vert_idx.insert(i);
	}

#pragma omp parallel for num_threads(num_proc) \
 private(status)
	for (int i = 0; i < end; i++) {
		// for each vertex, seeds = all verts that are not the ith

		
		// calc COR
		MPoint a_cor(0, 0, 0);
		status = calcCOR(i, all_vert_idx, a_cor);
		//CHECK_MSTATUS_AND_RETURN_IT(status);
		status = cor_.set(a_cor, i);
		//CHECK_MSTATUS_AND_RETURN_IT(status);

	} // end omp loop

	  // set the CoR plug on the deformer
	status = setCOR();
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = setValid(true);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;
}
MStatus corSkinCmd::cachePrecompSupport()
{
	MStatus status;

	MItMeshPolygon iter_faces(pInGeoPath_, MObject::kNullObj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	cached_tri_areas_.reserve(iter_faces.count());
	cached_avg_tri_positions_.reserve(iter_faces.count());
	cached_avg_weights_.reserve(iter_faces.count());
	for (unsigned int i = 0; !iter_faces.isDone(); i++, iter_faces.next()) {
		// pushing something into the vector
		vector<double> face_tri_area;
		cached_tri_areas_.push_back(face_tri_area);
		vector<MPoint> face_avg_pts;
		cached_avg_tri_positions_.push_back(face_avg_pts);
		vector<vector<double>> face_avg_weights;
		cached_avg_weights_.push_back(face_avg_weights);
		
		// determine triangle number
		int numTriangles;
		status = iter_faces.numTriangles(numTriangles);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		// now that we have something in the vectors we can use the access
		cached_tri_areas_.at(i).reserve(numTriangles);
		cached_avg_tri_positions_.at(i).reserve(numTriangles);
		cached_avg_weights_.at(i).reserve(numTriangles);
		for (int j = 0; j < numTriangles; j++) {
			MPointArray pts;
			MIntArray vert_idx;
			status = iter_faces.getTriangle(j, pts, vert_idx);
			CHECK_MSTATUS_AND_RETURN_IT(status);
			cached_tri_areas_.at(i).push_back(area(pts));
			cached_avg_tri_positions_.at(i).push_back(averagePosition(pts));
			cached_avg_weights_.at(i).push_back(averageWeight(vert_idx));
		}
	}
	return MS::kSuccess;
}

MStatus corSkinCmd::performPrecomp() {
	MStatus status;
	status = setValid(false);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	int max_num_proc = 0;
	int num_proc = 0;
	max_num_proc = std::thread::hardware_concurrency();
	if (max_num_proc == 0) {
		num_proc = iMT;
	}
	else if (int(iMT) <= max_num_proc){
		num_proc = iMT;
	}
	else {
		num_proc = max_num_proc;
	}

	int end = verts_.length();

	int j = 0;
	int test_end = 100;

#pragma omp parallel for num_threads(num_proc) \
 private(status)
	//	default(shared)
	for (int i = 0; i < end; i++) {
		// for each vertex

		// find nearest cluster
		int cluster_id;
#pragma omp critical (ann)
		cluster_id = ann_->search(weights_[i]);

		// get the seeds
		vector<int> seeds = clusters_[cluster_id].indexes();
		// eliminate any seeds that are past the threshold
		vector<int> good_seeds;
		for (auto seed_iter = seeds.begin(); seed_iter != seeds.end(); seed_iter++) {
			if (l2_distance(weights_[i], weights_[*seed_iter]) <= w_) {
				good_seeds.push_back(*seed_iter);
			}
		}
		seeds = good_seeds;

		// grow the seeds
		set<int> grown_to(seeds.begin(), seeds.end());
		set<int>temp(seeds.begin(), seeds.end());
		//status = test(i, temp);
		status = growTheSeeds(i, grown_to);
		//CHECK_MSTATUS_AND_RETURN_IT(status);

		// calc COR
		MPoint a_cor(0, 0, 0);
		status = calcCOR(i, grown_to, a_cor);
		//CHECK_MSTATUS_AND_RETURN_IT(status);
		status = cor_.set(a_cor, i);
		//CHECK_MSTATUS_AND_RETURN_IT(status);

	} // end omp loop

	// set the CoR plug on the deformer
	status = setCOR();
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = setValid(true);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return MS::kSuccess;
}

MStatus corSkinCmd::redoIt() {
	MStatus status;
	bool bNoTimer = false;
	if (bNPrecomp) {
		startTimer();
		status = performNaivePrecomp();
		CHECK_MSTATUS_AND_RETURN_IT(status);
		stopTimer();
	}
	else if(bPrecomp){
		startTimer();
		status = performPrecomp();
		CHECK_MSTATUS_AND_RETURN_IT(status);
		stopTimer();
	}
	else {
		bNoTimer = true;
	}
	//unsigned int micro_sec;
	double seconds;
	MStringArray result_list;
	result_list.append(name_);
	if (!bNoTimer) {
		if (bNPrecomp) {
			result_list.append("NPrecomp");
		}
		else {
			result_list.append("Precomp");
		}
		result_list.append(to_string(iMT).data());
		//readTimer(micro_sec);
		readTimer(seconds);
		//string ms_str = to_string(micro_sec);
		string ms_str = to_string(seconds);
		result_list.append(ms_str.data());
	}
	
	setResult(result_list);
	return MS::kSuccess;
}

MStatus corSkinCmd::undoIt()
{
	MStatus status;
	status = dgMod_.undoIt();
	CHECK_MSTATUS_AND_RETURN_IT(status);
	return MS::kSuccess;
}
