import maya.cmds as cmds
import pymel.core as pm
import pymel.core.datatypes as dt
import scipy as sp
from scipy import linalg as math

# Root joints for source and target
sRoot = pm.ls(sl=True, type='joint')[0]
tRoot = pm.ls(sl=True, type='joint')[1]

# Animation length 
animationLength = (pm.keyframe(sRoot, q=True, kc=True)) / 10

# Source and target rotation/orientation
size = int(len(pm.ls(type = 'joint')) / 2)

# Bindpose
sBindpose = sp.zeros((size, 4, 4), dtype=sp.float32)
tBindpose = sp.zeros((size, 4, 4), dtype=sp.float32)

# Different space matrices
worldRot = sp.zeros((size, 4, 4), dtype=sp.float32)

# Parent matrices
sParentMat = sp.zeros((size, 4, 4), dtype=sp.float32)
tParentMat = sp.zeros((size, 4, 4), dtype=sp.float32)

# Source and target lists
sourceList = [None] * size
targetList = [None] * size


def loadSourceList(node, index):
    sourceList[index] = node
    index += 1
    
    if node.numChildren() > 0:
        for child in node.getChildren():
            index = loadSourceList(child, index)
    
    return index


def loadTargetList(node, index):
    targetList[index] = node
    index += 1
    
    if node.numChildren() > 0:
        for child in node.getChildren():
            index = loadTargetList(child, index)
    
    return index


def loadSource(node, keys):
    for i, joint in enumerate(node):
        if i > 0:
            if keys == 0:                
                sBindpose[i-1] = sp.matrix(joint.getRotation().asMatrix())                
                sParentMat[i-1] = getParentsMatrix(joint, sp.identity(4))                
            
            keyRot = sp.matrix(joint.getRotation().asMatrix())
            keyOrient = sp.matrix(joint.getOrientation().asMatrix())
            
            # Isolate rotation
            isolatedRotation = sp.matmul(math.inv(sBindpose[i-1]), keyRot)
            
            # World rotation
            f1 = sp.matmul(math.inv(keyOrient), math.inv(sParentMat[i-1]))
            f2 = sp.matmul(isolatedRotation, sParentMat[i-1])            
            
            worldRot[i-1] = sp.matmul(sp.matmul(f1, f2), keyOrient)
            

def loadTarget(node, keys):
    for i, joint in enumerate(node):
        if i > 0:   
            if keys == 0:                
                tBindpose[i-1] = sp.matrix(joint.getRotation().asMatrix())                
                tParentMat[i-1] = getParentsMatrix(joint, sp.identity(4))
                          
                          
            keyOrient = sp.matrix(joint.getOrientation().asMatrix())
                        
            # Calculate the rotation from the source relative to the target
            jointSpace = sp.matmul(sp.matmul(sp.matmul(keyOrient, tParentMat[i-1]), sp.matmul(worldRot[i-1], math.inv(tParentMat[i-1]))), math.inv(keyOrient))
            
            # Set the rotation and keyframe
            joint.setRotation(dt.degrees(dt.EulerRotation(dt.Matrix(sp.matmul(tBindpose[i-1], jointSpace).tolist()))))
            pm.setKeyframe(joint)
    
             
def getParentsMatrix(child, parentMatrix):
    if type(child.getParent()) == pm.nodetypes.Joint:
        parentMatrix = getParentsMatrix(child.getParent(), parentMatrix)
                
        jointParentRotation = sp.matrix(child.getParent().getRotation().asMatrix())               
        jointParentOrient = sp.matrix(child.getParent().getOrientation().asMatrix()) 
                
        parentMatrix = sp.matmul((jointParentRotation * jointParentOrient), parentMatrix)
        
    return parentMatrix 


def transferData(sIndex, tIndex):
     
    sIndex = loadSourceList(sRoot, sIndex)
    tIndex = loadTargetList(tRoot, tIndex)
       
    for keys in range(int(animationLength)):
        cmds.currentTime(keys)
        
        loadSource(sourceList, keys)
        loadTarget(targetList, keys)
        
        tRoot.setOrientation(sp.array(sRoot.getOrientation()))
        tRoot.setRotation(sp.array(sRoot.getRotation()))
        tRoot.setTranslation(sp.array(sRoot.getTranslation()))
       
        pm.setKeyframe(tRoot)
         
    pm.currentTime(0)


def testing():    
    nrOfTimes = 2
    
    textfilepath = "C:/Users/Pad/Documents/GitHub/ThesisProject/SciPy/Test.txt"
    textfile = open(textfilepath, "wb") 
    
    for i in range(nrOfTimes):        
        
        sourceIndex = 0
        targetIndex = 0
        
        cmds.timer(s=True)
        
        transferData(sourceIndex, targetIndex)  
        
        t = cmds.timer(e = True)
        time = str(t) + "\n"
        timeEncode = time.encode()
        textfile.write(timeEncode)
        pm.currentTime(0)          
           
    textfile.close() 
   
testing()
 
    
                                    

       
            
    