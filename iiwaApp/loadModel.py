import os
from director import ioUtils
from director import visualization as vis
from director import objectmodel as om
from director import transformUtils



objectsRepo = os.path.expanduser('~/catkin_ws/src/perception_deps/objects')


def addModel(meshFile, frameName, modelName, rpyOffset):

    modelFileName = os.path.join(objectsRepo, meshFile)

    def loadModel():
        polyData = ioUtils.readPolyData(modelFileName)
        obj = vis.updatePolyData(polyData, modelName)
        vis.addChildFrame(obj)

    def getModelFrame():
        return om.findObjectByName(frameName)

    def onFrameModified(frame):

        obj = om.findObjectByName(modelName)
        if not obj:
            return
        meshToModel = transformUtils.frameFromPositionAndRPY([0,0,0], rpyOffset)
        modelToWorld = getModelFrame().transform
        t = transformUtils.concatenateTransforms([meshToModel, modelToWorld])
        obj.getChildFrame().copyFrame(t)

    loadModel()
    getModelFrame().connectFrameModified(onFrameModified)


#oilBottleRpy = [0,270,0]
oilBottleRpy = [0,0,0]

#addModel('meshes/oil_bottle.stl', 'oilBottle', 'oil bottle model', rpyOffset=oilBottleRpy)
#addModel('meshes/engine.stl', 'engine', 'engine model', rpyOffset=[0,0,0])

addModel('meshes/blue_funnel.stl', 'bfunnel', 'blue funnel', rpyOffset=[0,0,0])
