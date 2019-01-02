#include "corSkinCluster.h"
#include "corSkinCmd.h"

#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>

MStatus initializePlugin(MObject obj) {
	MStatus status;
	MFnPlugin plugin(obj, "Benjamin Slack", "1.0", "Any");
	status = plugin.registerNode(corSkinCluster::kname, corSkinCluster::id, corSkinCluster::creator, corSkinCluster::initialize,
		MPxNode::kSkinCluster);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = plugin.registerCommand(corSkinCmd::kName, corSkinCmd::creator, corSkinCmd::newSyntax);
	CHECK_MSTATUS_AND_RETURN_IT(status);
#if MAYA_API_VERSION >= 201600
	status = MGPUDeformerRegistry::registerGPUDeformerCreator(corSkinCluster::kname, corSkinGPUDeformer::kName,
		corSkinGPUDeformer::getGPUDeformerInfo());
	CHECK_MSTATUS_AND_RETURN_IT(status);
	// Set the load path so we can find the cl kernel.
	corSkinGPUDeformer::plugin_path = plugin.loadPath();
	status = MGPUDeformerRegistry::addConditionalAttribute(
		corSkinCluster::kname,
		corSkinGPUDeformer::kName,
		corSkinCluster::cor_valid);
#endif


	//if (MGlobal::mayaState() == MGlobal::kInteractive) {
	//	MGlobal::executePythonCommandOnIdle("import cvwrap.menu");
	//	MGlobal::executePythonCommandOnIdle("cvwrap.menu.create_menuitems()");
	//}

	return status;
}

MStatus uninitializePlugin(MObject obj) {
	MStatus status;
	MFnPlugin plugin(obj);

#if MAYA_API_VERSION >= 201600
	status = MGPUDeformerRegistry::deregisterGPUDeformerCreator(corSkinCluster::kname, corSkinGPUDeformer::kName);
	CHECK_MSTATUS_AND_RETURN_IT(status);
#endif
	status = plugin.deregisterCommand(corSkinCmd::kName);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = plugin.deregisterNode(corSkinCluster::id);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	//if (MGlobal::mayaState() == MGlobal::kInteractive) {
	//	MGlobal::executePythonCommandOnIdle("import cvwrap.menu");
	//	MGlobal::executePythonCommandOnIdle("cvwrap.menu.destroy_menuitems()");
	//}

	return status;
}