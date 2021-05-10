
#include "maya_includes.h"
#include <maya/MTimer.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <queue>
#include <string>
#include <fstream>

using namespace std;

std::vector<MObject> sourceList;
std::vector<MObject> targetList;

MMatrixArray sourceBindPoseRotation; 
MMatrixArray  targetBindPoseRotation;

MMatrixArray  worldRotation;
MMatrixArray  translatedRotation;

MMatrixArray  sourceParentMatrices;
MMatrixArray  targetParentMatrices;

void loadList(MObject node, bool source)
{
	MFnIkJoint joint(node);

	if (source)
		sourceList.push_back(node);
	else
		targetList.push_back(node);

	if (joint.childCount() > 0)
	{
		for (int i = 0; i < joint.childCount(); i++)
		{
			MObject child = joint.child(i);

			MFnIkJoint joint(joint.child(i));
			if (child.hasFn(MFn::kJoint))
			{
				loadList(child, source);
			}
		}
	}
}

void allocateMemory()
{
	targetList.reserve(sourceList.size());

	sourceBindPoseRotation.setLength(sourceList.size());
	targetBindPoseRotation.setLength(sourceList.size());
	sourceParentMatrices.setLength(sourceList.size());
	targetParentMatrices.setLength(sourceList.size());
	worldRotation.setLength(sourceList.size());
	translatedRotation.setLength(sourceList.size());
}

MMatrix getParentMatrix(MObject node, MMatrix parentMatrix)
{
	MFnIkJoint joint(node);
	if (joint.parent(0).hasFn(MFn::kJoint))
	{
		double x, y, z, w;
		MQuaternion orientation;
		MMatrix keyframeOrientation;
		MMatrix keyframeRotation;

		MFnIkJoint parent(joint.parent(0));
		parentMatrix = getParentMatrix(joint.parent(0), parentMatrix);

		parent.getRotationQuaternion(x, y, z, w);
		MQuaternion rotation(x, y, z, w);
		parent.getOrientation(orientation);

		keyframeOrientation = orientation.asMatrix();
		keyframeRotation = rotation.asMatrix();

		parentMatrix = (keyframeRotation * keyframeOrientation) * parentMatrix;
	}
	
	return parentMatrix;
}

void calculateSourceSkeleton(std::vector<MObject> skeleton, int keyframe)
{
	double x, y, z, w;
	MQuaternion orientation;
	MMatrix keyframeOrientation;
	MMatrix keyframeRotation;
	MMatrix identity;
	MStatus status;

	for (int i = 0; i < skeleton.size(); i++)
	{
		MFnIkJoint joint(skeleton[i]);

		// If we have passed the root bone
		if (i > 0)
		{
			// If we are at keyframe 0, save the bindpose
			if (keyframe == 0)
			{
				joint.getRotationQuaternion(x, y, z, w);
				MQuaternion bindPoserotation(x, y, z, w);
				sourceBindPoseRotation.set(bindPoserotation.asMatrix(), i-1);

				identity.setToIdentity();
				sourceParentMatrices.set(getParentMatrix(skeleton[i], identity), i-1);
			}
			
			joint.getRotationQuaternion(x, y, z, w);
			MQuaternion rotation(x, y, z, w);
			joint.getOrientation(orientation);

			keyframeOrientation = orientation.asMatrix();
			keyframeRotation = rotation.asMatrix();

			MMatrix isolatedRotation = sourceBindPoseRotation[i - 1].inverse() * keyframeRotation;

			worldRotation.append(keyframeOrientation.inverse() * sourceParentMatrices[i - 1].inverse() * isolatedRotation * sourceParentMatrices[i - 1] * keyframeOrientation);
		}
	}
}

void calculateTargetSkeleton(std::vector<MObject> skeleton, int keyframe)
{
	double x, y, z, w;
	MQuaternion orientation;
	MMatrix keyframeOrientation;
	MMatrix keyframeRotation;
	MMatrix identity;
	MStatus status;

	for (int i = 0; i < skeleton.size(); i++)
	{
		MFnIkJoint joint(skeleton[i]);

		if (i > 0)
		{
			if (keyframe == 0)
			{
				joint.getRotationQuaternion(x, y, z, w);
				MQuaternion bindPoserotation(x, y, z, w);
				targetBindPoseRotation.set(bindPoserotation.asMatrix(), i-1);

				identity.setToIdentity();
				targetParentMatrices.set(getParentMatrix(skeleton[i], identity), i-1);
			}

			joint.getRotationQuaternion(x, y, z, w);
			MQuaternion rotation(x, y, z, w);
			joint.getOrientation(orientation);

			keyframeOrientation = orientation.asMatrix();
			keyframeRotation = rotation.asMatrix();

			MMatrix b = (keyframeOrientation * targetParentMatrices[i - 1] * worldRotation[i - 1] * targetParentMatrices[i - 1].inverse() * keyframeOrientation.inverse());
			MMatrix c = targetBindPoseRotation[i - 1] * b;

			MQuaternion quat; quat = c;

			double dest[4];
			quat.get(dest);

			joint.setRotationQuaternion(dest[0], dest[1], dest[2], dest[3]);

			MGlobal::select(skeleton[i], MGlobal::ListAdjustment::kReplaceList);
			MGlobal::executeCommand("setKeyframe");
		}
	}
}

void start()
{	
	std::string fileName = "C:/Users/Pad/Documents/GitHub/ThesisProject/C++ API/C++Test.txt"; //"C:/Users/Galfi/Documents/C++.txt";	
	std::ofstream output;

	std::cout << fixed;
	cout.precision(6);

	MTimer myTimer;
	MStatus status;
	int nrOfKeyframes = 0;

	MSelectionList selected;
	MGlobal::getActiveSelectionList(selected);

	MObject sourceNode;
	MObject targetNode;

	selected.getDependNode(0, sourceNode);
	selected.getDependNode(1, targetNode);

	myTimer.beginTimer();

	// Load source skeleton
	loadList(sourceNode, true);

	// Allocate memory
	allocateMemory();

	// Load target skeleton
	loadList(targetNode, false);

	MItDependencyGraph dgIter(sourceNode, MFn::kAnimCurve, MItDependencyGraph::kUpstream, MItDependencyGraph::kBreadthFirst, MItDependencyGraph::kNodeLevel, &status);

	if (status)
	{
		MObject current = dgIter.currentItem();
		MFnAnimCurve animCurve(current, &status);

		if (status == MS::kSuccess)
		{
			nrOfKeyframes = animCurve.numKeys();
		}
	}
	else
	{
		std::cout << status.errorString() << std::endl;
	}

	for (int i = 0; i <= nrOfKeyframes; i++)
	{
		MGlobal::viewFrame(i);

		worldRotation.clear();
		translatedRotation.clear();

		double x, y, z, w;
		MQuaternion orientation;
		MMatrix keyframeOrientation;
		MMatrix keyframeRotation;

		MFnIkJoint rootObject(sourceNode);
		MVector translation = rootObject.getTranslation(MSpace::kTransform);

		rootObject.getRotationQuaternion(x, y, z, w);
		MQuaternion rotation(x, y, z, w);
		rootObject.getOrientation(orientation);

		keyframeOrientation = orientation.asMatrix();
		keyframeRotation = rotation.asMatrix();

		double rot[4];
		rotation.get(rot);

		calculateSourceSkeleton(sourceList, i);
		calculateTargetSkeleton(targetList, i);

		MFnIkJoint targetRoot(targetList[0]);
		targetRoot.setTranslation(translation, MSpace::kTransform);
		targetRoot.setRotateOrientation(orientation, MSpace::kTransform, true);
		targetRoot.setRotationQuaternion(rot[0], rot[1], rot[2], rot[3]);

		MGlobal::select(targetNode, MGlobal::ListAdjustment::kReplaceList);
		MGlobal::executeCommand("setKeyframe");
	}

	myTimer.endTimer();
	float time = myTimer.elapsedTime();

	output.open(fileName, std::ofstream::out | std::ofstream::app);
	output << time;
	output << "\n";
	output.close();
}

EXPORT MStatus initializePlugin(MObject obj) 
{		
	MStatus status;

	MFnPlugin myPlugin(obj, "Animation transfer", "1.0", "Any", &status);
	if (MFAIL(status)) 
	{
		CHECK_MSTATUS(status);
		return status;
	}  	

	/* Redirects outputs to mayas output window instead of scripting output */
	std::cout.set_rdbuf(MStreamUtils::stdOutStream().rdbuf());
	std::cerr.set_rdbuf(MStreamUtils::stdErrorStream().rdbuf());

	/* Start transfer */
	start();

	return status;
}
	
EXPORT MStatus uninitializePlugin(MObject obj) {
	MFnPlugin plugin(obj);

	MTimer gTimer;

	gTimer.endTimer();
	gTimer.clear();

	return MS::kSuccess;
}