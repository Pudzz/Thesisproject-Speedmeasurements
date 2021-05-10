import maya.api.OpenMaya as om
import maya.api.OpenMayaAnim as oma
import maya.cmds as cmds

sourceRootStr = cmds.ls(sl=True, type = 'joint')[0] 
targetRootStr = cmds.ls(sl=True, type = 'joint')[1]
total = len(cmds.ls(type = 'joint')) / 2

sourceAList = om.MDagPathArray().setLength(total)
targetAList = om.MDagPathArray().setLength(total)

sBindposeRot = om.MMatrixArray().setLength(total)
tBindposeRot = om.MMatrixArray().setLength(total)

sParentMat = om.MMatrixArray().setLength(total)
tParentMat = om.MMatrixArray().setLength(total)	

worldRot = om.MMatrixArray().setLength(total)		

animationLength = cmds.keyframe(sourceRootStr, q=True, kc=True) / 10

index = 0
def loadList(node, source):    
    global index 
    
    if source:
        sourceAList[index] = om.MDagPath(node)
        index += 1
    else:
        targetAList[index] = om.MDagPath(node)        
        index += 1
     
    object = om.MObject(node.node())
    if object.hasFn(om.MFn.kJoint):
        if node.childCount() > 0:
            for i in range(node.childCount()):
                childObject = om.MObject(node.child(i))            
                childDagNode = om.MFnDagNode(childObject)               
                childPath = om.MDagPath()
                childPath = childDagNode.getPath()
                
                loadList(childPath, source)
                
    if index == total:
        index = 0                                      
 
 
def getParentMatrix(node, parentMatrix):
    jntobject = node.parent(0)  
    if jntobject.hasFn(om.MFn.kJoint):
        parentJntObj = om.MFnTransform(jntobject)
        parentMatrix = getParentMatrix(parentJntObj, parentMatrix)
        
        # Get rotation, orientation
        rotation = parentJntObj.rotation(om.MSpace.kTransform, asQuaternion = True)  
        orientation = parentJntObj.rotateOrientation(om.MSpace.kTransform)
        
        parentMatrix = (rotation.asMatrix() * orientation.asMatrix()) * parentMatrix
    
    return parentMatrix


def loadTarget(node, keys):
    for i in range(node.__len__()):
        if i > 0:
            transNode = om.MFnTransform(node[i])
            
            if keys == 0:
                rotation = transNode.rotation(om.MSpace.kTransform, asQuaternion = True)
                tBindposeRot[i-1] = rotation.asMatrix() 
                
                par = om.MMatrix().setToIdentity()                
                parmat = getParentMatrix(transNode, par)
                tParentMat[i-1] = parmat
                        
            keyOrient = transNode.rotateOrientation(om.MSpace.kTransform)
                        
            b = (keyOrient.asMatrix() * tParentMat[i-1] * worldRot[i-1] * tParentMat[i-1].inverse() * keyOrient.inverse().asMatrix())
            c = tBindposeRot[i-1] * b   
            
            order = om.MTransformationMatrix(c)
            euler = om.MEulerRotation()
            new = euler.decompose(c, order.rotationOrder()) 
                     
            transNode.setRotation(new, om.MSpace.kTransform)
            
            dag = om.MFnDagNode(node[i].node())
            name = dag.name()
            cmds.select(name)
            target = cmds.ls(sl=True, type ='joint')[0]
                       
            cmds.setKeyframe()    

                     
def loadSource(node, keys):
    for i in range(node.__len__()):        
        if i > 0:
            transNode = om.MFnTransform(node[i])
            
            if keys == 0:
                rotation = transNode.rotation(om.MSpace.kTransform, asQuaternion = True)
                sBindposeRot[i-1] = rotation.asMatrix()
                
                par = om.MMatrix().setToIdentity() 
                parmat = getParentMatrix(transNode, par)
                sParentMat[i-1] = parmat                
                
            keyRot = transNode.rotation(om.MSpace.kTransform, asQuaternion = True) 
            keyOrient = transNode.rotateOrientation(om.MSpace.kTransform) 
            
            isolatedRotation = sBindposeRot[i-1].inverse() * keyRot.asMatrix()
                         
            worldRotation = keyOrient.inverse().asMatrix() * sParentMat[i-1].inverse() * isolatedRotation * sParentMat[i-1] * keyOrient.asMatrix()    
            worldRot[i-1] = worldRotation


def transfer():
        
    selList = om.MGlobal.getActiveSelectionList()
    
    sourceRoot = selList.getDagPath(0)
    targetRoot = selList.getDagPath(1)
    
    loadList(sourceRoot, True)
    loadList(targetRoot, False) 
   
    for i in range(int(animationLength)):
        om.MGlobal.viewFrame(i)
                        
        rootObject = om.MFnTransform(sourceRoot)               
                       
        rotation = rootObject.rotation(om.MSpace.kTransform, asQuaternion = True)       
        orientation = rootObject.rotateOrientation(om.MSpace.kTransform)        
        translation = rootObject.translation(om.MSpace.kTransform)        
                    
        loadSource(sourceAList, i)
        loadTarget(targetAList, i)
        
        cmds.select(targetRootStr)
        targetObject = om.MFnTransform(targetRoot)        
        
        targetObject.setRotation(rotation, om.MSpace.kTransform)
        targetObject.setRotateOrientation(orientation, om.MSpace.kTransform, False) 
        targetObject.setTranslation(translation, om.MSpace.kTransform)        
        
        cmds.setKeyframe()  
    
    om.MGlobal.viewFrame(0)
    cmds.select(sourceRootStr)
    cmds.select(targetRootStr, add=True) 
   

def doTest():   

    nrOfTimes = 2
    
    textfilepath = "C:/Users/Pad/Documents/GitHub/ThesisProject/OpenMaya/test_2_0.txt"
    textfile = open(textfilepath, "wb") 
    
    for i in range(nrOfTimes):
        
        cmds.timer(s=True)
        
        transfer()   
        
        t = cmds.timer(e = True)
        time = str(t) + "\n"
        timeEncode = time.encode()
        textfile.write(timeEncode)
        
        om.MGlobal.viewFrame(0)               
        
    textfile.close() 
      
doTest()
    
    
