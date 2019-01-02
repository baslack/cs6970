#ifndef COR_SKIN_CLUSTER

#define COR_SKIN_CLUSTER 1

// various includes
//#include <maya/MFnPlugin.h>
#include <maya/MTypeId.h>
#include <maya/MMatrixArray.h>
//#include <maya/MStringArray.h>
#include <maya/MPxSkinCluster.h>
#include <maya/MItGeometry.h>
#include <maya/MItMeshPolygon.h>
//#include <maya/MDoubleArray.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MFnMatrixData.h>
#include <maya/MQuaternion.h>
//#include <maya/MMatrix.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MFnPointArrayData.h>
#include <maya/MPxGPUDeformer.h>
//#include <maya/MEvaluationNode.h>
#include <maya/MGPUDeformerRegistry.h>
#include <maya/MOpenCLInfo.h>
//#include <maya/MViewport2Renderer.h>
#include <clew/clew_cl.h>

//#include <CL/cl.h>

#include <maya/MPxDeformerNode.h>
#include <maya/MString.h>
#include <maya/MProfiler.h>
#include <vector>

// constants
#define NAME "corSkinCluster"
#define DEFAULT_OMEGA 0.1

class corSkinCluster : public MPxSkinCluster {
    public:
        static  void*   creator();

        static  MStatus initialize();

        virtual MStatus deform(MDataBlock&    block,
                               MItGeometry&   iter,
                               const MMatrix& mat,
                               unsigned int   multiIndex);

        virtual MStatus precomp(MDataBlock);
        static const MTypeId id;
		static const MString kname;
        static const int _profileCategory;
        static MObject cor_valid;
        static MObject cor_ar;

    private:
        static MStatus handle_to_doublearray(MArrayDataHandle&, MDoubleArray&);

        static MStatus similarity(MDoubleArray&, MDoubleArray&, int, double&);

        static MStatus qlerp(MQuaternion&, MQuaternion&, MQuaternion&);

        static const double omega;
};


class corSkinGPUDeformer : public MPxGPUDeformer {
public:
	// Virtual methods from MPxGPUDeformer
	corSkinGPUDeformer();
	virtual ~corSkinGPUDeformer();
	virtual MPxGPUDeformer::DeformerStatus evaluate(MDataBlock& block, const MEvaluationNode&, const MPlug& plug, unsigned int numElements, const MAutoCLMem, const MAutoCLEvent, MAutoCLMem, MAutoCLEvent&);
	virtual void terminate();
	static MGPUDeformerRegistrationInfo* getGPUDeformerInfo();
	static bool validateNodeInGraph(MDataBlock& block, const MEvaluationNode&, const MPlug& plug, MStringArray* messages);
	static bool validateNodeValues(MDataBlock& block, const MEvaluationNode&, const MPlug& plug, MStringArray* messages);
	static MString plugin_path;
	static MString openCL_source_file;
	static MString kName;
private:
	// helper methods
	void extractWeightArray(MDataBlock& block, const MEvaluationNode& evaluationNode, const MPlug& plug);
	MStatus extractCoR(MDataBlock& block, const MEvaluationNode& evaluationNode, const MPlug& plug);
	void extractTMnQ(MDataBlock& block, const MEvaluationNode& evaluationNode, const MPlug& plug);
	// Storage for data on the GPU
	MAutoCLMem fCLWeights;
	MAutoCLMem fCoR;
	MAutoCLMem fTM;
	MAutoCLMem fQ;
	unsigned int fNumTransforms;
	unsigned int fNumElements;
	// Kernel
	MAutoCLKernel fKernel;
};

class corSkinNodeGPUDeformerInfo : public MGPUDeformerRegistrationInfo {
public:
	corSkinNodeGPUDeformerInfo() {}
	virtual ~corSkinNodeGPUDeformerInfo() {}
	virtual MPxGPUDeformer* createGPUDeformer() {
		return new corSkinGPUDeformer();
	}

	virtual bool validateNodeInGraph(MDataBlock& block, const MEvaluationNode& evaluationNode, const MPlug& plug, MStringArray* messages) {
		return corSkinGPUDeformer::validateNodeInGraph(block, evaluationNode, plug, messages);
	}
	virtual bool validateNodeValues(MDataBlock& block, const MEvaluationNode& evaluationNode, const MPlug& plug, MStringArray* messages) {
		return corSkinGPUDeformer::validateNodeValues(block, evaluationNode, plug, messages);
	}
};

#endif