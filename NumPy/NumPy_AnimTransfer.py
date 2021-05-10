import maya.cmds as cmds
import pymel.core as pm
import pymel.core.datatypes as dt
import numpy as np

# Root joints for source and target
sRoot = pm.ls(sl=True, type='joint')[0]
tRoot = pm.ls(sl=True, type='joint')[1]

size = np.intc(len(pm.ls(type = 'joint')) / 2)

# Source and target lists
sourceJnts = [pm.nodetypes.Joint] * size
targetJnts = [pm.nodetypes.Joint] * size

# Animationlength
animationLength = (pm.keyframe(sRoot, q=True, kc=True)) / 10
animLength = np.intc(animationLength)

# Source and target rotation/orientation
sBindposeRot = np.zeros((size, 4, 4), dtype=np.float32)
tBindposeRot = np.zeros((size, 4, 4), dtype=np.float32)

# Different space matrices
worldRot = np.zeros((size, 4, 4), dtype=np.float32)

# Parent matrices
sParentMat = np.zeros((size, 4, 4), dtype=np.float32)
tParentMat = np.zeros((size, 4, 4), dtype=np.float32)

index = 0
def loadList(node, string):
    global index
    
    if string == "source":
        sourceJnts[index] = node 
        index += 1       
        
    if string == "target":
        targetJnts[index] = node
        index += 1      
             
    if node.numChildren() > 0:    
        for child in node.getChildren():
            loadList(child, string)  
            
    if index == size:
        index = 0   


def loadSource(node, keys):
    for i, joint in enumerate(node):
        if i > 0:
            if keys == 0:                
                sBindposeRot[i-1] = np.matrix(joint.getRotation().asMatrix())                
                sParentMat[i-1] = getParentsMatrix(joint, np.identity(4))                
            
            keyRot = np.matrix(joint.getRotation().asMatrix())
            keyOrient = np.matrix(joint.getOrientation().asMatrix())
            
            #Isolate rotation
            sBindInv = np.linalg.inv(sBindposeRot[i-1])
            isolatedRotation = np.matmul(sBindInv,keyRot)
            
            # World rotation
            keyOrientInv = np.linalg.inv(keyOrient)
            sParentInv = np.linalg.inv(sParentMat[i-1])
            
            f1 = np.matmul(keyOrientInv, sParentInv)
            f2 = np.matmul(isolatedRotation, sParentMat[i-1])            
            
            worldRot[i-1] = np.matmul(np.matmul(f1, f2),keyOrient)
            

def loadTarget(node, keys):
    for i, joint in enumerate(node):
        if i > 0:   
            if keys == 0:                
                tBindposeRot[i-1] = np.matrix(joint.getRotation().asMatrix())                
                tParentMat[i-1] = getParentsMatrix(joint, np.identity(4))
                 
            keyOrient = np.matrix(joint.getOrientation().asMatrix())
                        
            # Calculate the rotation from the source relative to the target
            tParentMatInv = np.linalg.inv(tParentMat[i-1])
            keyOrientInv = np.linalg.inv(keyOrient)
            
            f1 = np.matmul(np.matmul(np.matmul(keyOrient, tParentMat[i-1]), np.matmul(worldRot[i-1], tParentMatInv)), keyOrientInv)
            
            # Set the rotation and keyframe
            joint.setRotation(dt.degrees(dt.EulerRotation(dt.Matrix(np.matmul(tBindposeRot[i-1], f1).tolist()))))
            pm.setKeyframe(joint)
            
            
def getParentsMatrix(child, parentMatrix): 
    # If the parent is a joint, we calculates the matrix, then check if their is another parent
    if type(child.getParent()) == pm.nodetypes.Joint:
        parentMatrix = getParentsMatrix(child.getParent(), parentMatrix)
                
        jointParentRotation = np.matrix(child.getParent().getRotation().asMatrix())               
        jointParentOrient = np.matrix(child.getParent().getOrientation().asMatrix()) 
                
        parentMatrix = np.matmul((jointParentRotation * jointParentOrient), parentMatrix)
  
    return parentMatrix                            
    

def transferData():
        
    loadList(sRoot, "source")
    loadList(tRoot, "target")
       
    for keys in range(np.intc(animLength)):
        cmds.currentTime(keys)         
        
        loadSource(sourceJnts, keys)
        loadTarget(targetJnts, keys)
        
        tRoot.setOrientation(np.array(sRoot.getOrientation()))
        tRoot.setRotation(np.array(sRoot.getRotation()))
        tRoot.setTranslation(np.array(sRoot.getTranslation()))      
       
        pm.setKeyframe(tRoot)
         
    pm.currentTime(0)

              
def doTest():    
    nrOfTimes = 2
      
    textfilepath = "C:/Users/Pad/Documents/GitHub/ThesisProject/NumPy/testNumPy.txt"
    textfile = open(textfilepath, "wb") 
    
    for i in range(nrOfTimes):        

        cmds.timer(s=True)
        
        transferData()  
        
        t = cmds.timer(e = True)
        time = str(t) + "\n"
        timeEncode = time.encode()
        textfile.write(timeEncode)
        pm.currentTime(0)      
             
        
    textfile.close() 

   
doTest()