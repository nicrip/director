from director import objectmodel as om
from director import lcmframe
from director import lcmUtils

cameraToWorldName = 'camera_rgb_optical_frame'
cameraToWorldFrame = om.findObjectByName(cameraToWorldName).transform
cameraToWorldMsg = lcmframe.rigidTransformMessageFromFrame(cameraToWorldFrame)
lcmUtils.publish('OPENNI_FRAME_LEFT_TO_LOCAL', cameraToWorldMsg)
