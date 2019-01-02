Cluster, simple class for tracking related vertices in weight space

CMakeLists.txt CMake project file

corSkinCluster, deformer plugin and GPU deformer plugin

corSkinCmd, maya command for creating corSkinCluster and performing precomputation

corSkinDef.cl, OpenCL kernel for GPU deformer

FindANN, wrapper for ANN library calls

Grid, regular subdivision of N-dimensional weight space

MaxMinDist, non-functioning sketch of Schlomer max min distance sets

pluginMain.cpp, adds corSkinCluster and corSkinCmd to Maya

---

Detailed Readme of corSkinCmd, per Dr. Bae. I've just
broken down each member using the header and some 
descriptive type.

class corSkinCmd : public MPxCommand {
public:
	corSkinCmd();
	~corSkinCmd();
	/*
	Constants
	*/
	static const char* kName;
	static const char* kPrecompFlag;
	static const char* kNaivePrecompFlag;
	static const char* kDeformerNameFlag;
	static const char* kPrecompFlagShort;
	static const char* kNaivePrecompFlagShort;
	static const char* kDeformerNameFlagShort;
	static const char* kMultithreadFlag;
	static const char* kMultithreadFlagShort;

	/*
	Overrides for MPxCommand
	These methods are what Maya actually calls
	when performing the new command (corSkinCmd).
	*/
	static MSyntax newSyntax();
	MStatus doIt(const MArgList&);
	MStatus redoIt();
	MStatus undoIt();
	static void* creator();
	bool isUndoable();
	
private:
	// used in the undo system, records commands
	MDGModifier dgMod_;
	// holds the name of the current corSkinCluster 
	MString name_;
	// holds a list of DAG (directed acyclic graph) nodes
	MSelectionList sl_;
	// holds a DAG path (kind of like a directory path
	// to the deformed geometry
	MDagPath mesh_dag_;
	// holds DAG paths to the various bones/joints
	MDagPathArray bone_dag_ar_;
	// flag values set by arguments to direct program flow
	bool bPrecomp = false;
	bool bNPrecomp = false;
	// flag value set by argument to control number 
	// of processors
	unsigned int iMT = 1;
	// flag value for program flow
	bool bDeformerSpecified = false;
	// data storage for the calculated CoRs
	MPointArray cor_;
	// Maya data reference to the current corSkinCluster
	MObject deformer_;
	// list of the names of objects involved in deformation
	MStringArray objNames_;
	// utility class for handling selections
	MSelectionList selectionList_;
	// refernce to the ANN utility class used for searching
	// in the growing the seeds part of precomputation
	FindANN* ann_ = nullptr;
	// collectionf of clusters for acceleration of precomp
	vector<Cluster> clusters_;
	// data for all the vertex positions
	MPointArray verts_;
	// data handle for the input geometry (deformed mesh)
	MObject oInGeo_;
	// DAG path for the input geometry
	MDagPath pInGeoPath_;
	// extracted per vertex weights
	vector<vector<double>> weights_;
	
	// from here, we get into utility methods
	// which support data extraction and 
	// actual performance of precomputation
	// wrapper method for tasks required before
	// precomputation can begin: primarily data 
	// extraction and caching.
	MStatus setupPrecomp();
	MStatus setupNaivePrecomp();
	// utility method for creating a new corSKinCluster
	MStatus createDeformer();
	// utility method for performing the required
	// DAG connections for a new deformer
	MStatus connectJointToCluster(unsigned int);
	// utility method for extracting a data handle for 
	// a corSkinCluster from its name
	MStatus populateDeformerFromName();
	// utility method for parsing the command line
	// argumetns of corSkinCmd
	MStatus parseArgs(const MArgList&);
	// utility for extracting the DAG path of the
	// deforming and deformed objects
	MStatus parseDAGPaths();
	// utility node for getting the vertex data (shape)
	// node, from the input node name. In Maya, objects 
	// are separated into transforms and shapes. Transforms 
	// hold object position, rotation, etc. Shapes hold 
	// local vertex positions. Deformers work on shapes.
	MStatus getShapeNode(MDagPath&, bool = false);
	// utility node for confirming given DAG path is a
	// shape node
	bool isShapeNode(MDagPath&);
	// to be implemented, subdivision for higher accuracy
	MStatus generateSubdivMesh(void);
	// utility method for populating input geometry 
	// member fields
	MStatus getInputGeo(void);
	// utility method for extracting per vertex weights
	MStatus getVertexWeights(void);
	// utility method for setting up ANN search structure
	MStatus generateANNSearchStructure(void);
	// utility method for determing how many joints
	// are used in the deformer
	MStatus getNumBones(unsigned int&);
	// precomputation procedure for extending the seeds 
	// until the similarity threshold is reached
	MStatus growTheSeeds(int, set<int>&);
	// main accelerated precomputation method
	MStatus performPrecomp();
	// main naive precomputation method
	MStatus performNaivePrecomp();
	// utility method for caching average positions
	// area, and average weights, used in precomputation
	MStatus cachePrecompSupport();
	// euclidean n-dimensional distance metric, used
	// in precomputation
	double l2_distance(const vector<double>&, const vector<double>&);
	// Le/Hodgins similarity metric, used in precomputation
	double similarity(const vector<double>&, const vector<double>&);
	// utility method for calculating averagePosition
	// of a set of points
	MPoint averagePosition(const MPointArray&);
	// utility method for calculating average weights for
	// a list of vertices indexed by id int
	vector<double> averageWeight(const MIntArray&);
	// utility method for calculating area using 
	// cross product of two triange edges, defined by
	// their vertices' 
	double area(const MPointArray&);
	// main method for performing CoR calculation
	// takes vertex id, seeds, and a point refernce
	// for output.
	MStatus calcCOR(int, set<int>&, MPoint&);
	// utility method for setting the CoR array on 
	// the deformer
	MStatus setCOR();
	// utility method for setting the CoRs valid flag
	// n the deformer
	MStatus setValid(bool);
	// data members for the cached derived data used 
	// in precomputation
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
	
	/*
	internal tunables for the percomputation
	*/
	static double w_;
	static double e_;
	static double o_;
	static double E_;
};