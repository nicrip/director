

def setCameraPoseFromFrame(frameName):
    cameraToWorld = om.findObjectByName(frameName).transform
    cameraToWorldMsg = lcmframe.rigidTransformMessageFromFrame(cameraToWorld)
    lcmUtils.publish('OPENNI_FRAME_LEFT_TO_LOCAL', cameraToWorldMsg)


def initTfVis():
    execfile('tfvis.py', globals())


def initModelVis():
    execfile('loadModel.py', globals())


def initCameraPose():
    setCameraPoseFromFrame('camera_depth_optical_frame')


initTfVis()
