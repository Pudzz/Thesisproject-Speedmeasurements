import pymel.core as pm
import pymel.core.datatypes as dt
import maya.cmds as cmds

# Root joints for source and target
sRoot = pm.ls(sl = True, type = 'joint')[0]
tRoot = pm.ls(sl=True, type = 'joint')[1]
total = len(pm.ls(type = 'joint')) / 2

animationLength = pm.keyframe(sRoot, q=True, kc=True) / 10

# Global list for joints
sourceList = [pm.nodetypes.Joint] * int(total)
targetList = [pm.nodetypes.Joint] * int(total)

# Rotation/orientation lists
sBindposeRot = [dt.Matrix] * int(total)
tBindposeRot = [dt.Matrix] * int(total)

# Worldrotation matrices
worldRot = [dt.Matrix] * int(total)

# Parent matrices
sParentMat = [dt.Matrix] * int(total) 
tParentMat = [dt.Matrix] * int(total) 

index = 0
def loadList(node, string):
    global index
        
    if string == "source":
        sourceList[index] = node
        index += 1      
    
    if string == "target":
        targetList[index] = node
        index += 1                  
           
    if node.numChildren() > 0:
        for child in node.getChildren():
            loadList(child, string)
            
    if index == total:
        index = 0
      

def loadSource(node, keys):
    for i, joint in enumerate(node):
        if i > 0:
            if keys == 0:
                sBindposeRot[i-1] = joint.getRotation().asMatrix() 
                parent = 1
                parentMatrix = getParentsMatrix(joint, parent)
                sParentMat[i-1] = parentMatrix
            
            keyRot = joint.getRotation().asMatrix()
            keyOrient = joint.getOrientation().asMatrix()     
            
            # Isolated rotation           
            isolatedRotation = sBindposeRot[i-1].inverse() * keyRot
            
            # World rotation
            worldRot[i-1] = keyOrient.inverse() * sParentMat[i-1].inverse() * isolatedRotation * sParentMat[i-1] * keyOrient

                      
def loadTarget(node, keys):
    for i, joint in enumerate(node):
        if i > 0:  
            if keys == 0:
                tBindposeRot[i-1] = joint.getRotation().asMatrix() 
                parent = 1
                parentMatrix = getParentsMatrix(joint, parent)
                tParentMat[i-1] = parentMatrix
            
            keyOrient = joint.getOrientation().asMatrix()
            
            # Calculate the rotation from the source relative to the target
            translatedRot = (keyOrient * tParentMat[i-1] * worldRot[i-1] * tParentMat[i-1].inverse() * keyOrient.inverse())
            finalRot = tBindposeRot[i-1] * translatedRot
            
            # Set the rotation and keyframe
            joint.setRotation(dt.degrees(dt.EulerRotation(finalRot)))
            pm.setKeyframe(joint)

           
def getParentsMatrix(child, parentMat):
    jntParent = child.getParent()
    # Check if joint type,  calculates the matrix, then check if their is another parent
    if type(jntParent) == pm.nodetypes.Joint:
        parentMat = getParentsMatrix(jntParent, parentMat)
        parentMat = (jntParent.getRotation().asMatrix() * jntParent.getOrientation().asMatrix()) * parentMat
  
    return parentMat 

         
def transferData(): 
           
    loadList(sRoot, "source")
    loadList(tRoot, "target")
    
    for keys in range(int(animationLength)):
        
        pm.currentTime(keys)        
                
        loadSource(sourceList, keys)
        loadTarget(targetList, keys)
        
        # Set the attributes for the target root
        tRoot.setOrientation(sRoot.getOrientation())
        tRoot.setRotation(sRoot.getRotation() )
        tRoot.setTranslation(sRoot.getTranslation())
        
        pm.setKeyframe(tRoot)   
   
    pm.currentTime(0)
    
    
def doTest():    
    nrOfTimes = 2
    
    textfilepath = "C:/Users/Pad/Documents/GitHub/ThesisProject/PyMEL/test.txt"
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