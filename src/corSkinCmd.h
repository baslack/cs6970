#ifndef COR_SKIN_CMD
#define COR_SKIN_CMD

#include "corSkinCluster.h"
#include "Cluster.h"
#include "Grid.h"
#include "FindANN.h"
#include <maya/MPxCommand.h>
#include <maya/MArgList.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <maya/MSelectionList.h>
#include <maya/MItSelectionList.h>
#include <maya/MStringArray.h>
#include <maya/MString.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MGlobal.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <omp.h>
#include <thread>
#include <set>
#include <queue>
#include <ctime>
#include <cmath>
#include <string>
#include <maya/MItMeshVertex.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MDGModifier.h>
#include <maya/MItDependencyGraph.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MFnSet.h>

class corSkinCmd : public MPxCommand {
public:
	corSkinCmd();
	~corSkinCmd();
	static const char* kName;
	static const char* kPrecompFlag;
	static const char* kNaivePrecompFlag;
	static const char* kDeformerNameFlag;
	static const char* kPrecompFlagShort;
	static const char* kNaivePrecompFlagShort;
	static const char* kDeformerNameFlagShort;
	static const char* kMultithreadFlag;
	static const char* kMultithreadFlagShort;

	static MSyntax newSyntax();
	MStatus doIt(const MArgList&);
	MStatus redoIt();
	MStatus undoIt();
	static void* creator();
private:
	MDGModifier dgMod_;
	MString name_;
	MSelectionList sl_;
	MDagPath mesh_dag_;
	MDagPathArray bone_dag_ar_;
	bool bPrecomp = false;
	bool bNPrecomp = false;
	unsigned int iMT = 1;
	bool bDeformerSpecified = false;
	bool isUndoable();
	MPointArray cor_;
	MObject deformer_;
	MStringArray objNames_;
	MSelectionList selectionList_;
	FindANN* ann_ = nullptr;
	vector<Cluster> clusters_;
	MPointArray verts_;
	MObject oInGeo_;
	MDagPath pInGeoPath_;
	vector<vector<double>> weights_;
	MStatus setupPrecomp();
	MStatus setupNaivePrecomp();
	MStatus createDeformer();
	MStatus connectJointToCluster(unsigned int);
	MStatus populateDeformerFromName();
	MStatus parseArgs(const MArgList&);
	MStatus parseDAGPaths();
	MStatus getShapeNode(MDagPath&, bool = false);
	bool isShapeNode(MDagPath&);
	MStatus generateSubdivMesh(void);
	MStatus getInputGeo(void);
	MStatus getVertexWeights(void);
	MStatus generateANNSearchStructure(void);
	MStatus getNumBones(unsigned int&);
	MStatus growTheSeeds(int, set<int>&);
	MStatus performPrecomp();
	MStatus performNaivePrecomp();
	MStatus cachePrecompSupport();
	double l2_distance(const vector<double>&, const vector<double>&);
	double similarity(const vector<double>&, const vector<double>&);
	MPoint averagePosition(const MPointArray&);
	vector<double> averageWeight(const MIntArray&);
	double area(const MPointArray&);
	MStatus calcCOR(int, set<int>&, MPoint&);
	MStatus setCOR();
	MStatus setValid(bool);
	vector<vector<double>> cached_tri_areas_;
	vector<vector<MPoint>> cached_avg_tri_positions_;
	vector<vector<vector<double>>> cached_avg_weights_;
	// timer functions & data for output
	time_t start;
	time_t end;
	MStatus startTimer();
	MStatus stopTimer();
	MStatus readTimer(unsigned int&);
	MStatus readTimer(double &);
	bool earlyExit(unsigned int);
	static double w_;
	static double e_;
	static double o_;
	static double E_;
};

#endif // !COR_SKIN_PRECOMP
