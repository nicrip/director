from director import objectmodel as om
from director import lcmframe
from director import lcmUtils
import bot_core as lcmbot






class ChannelDiscoverer(object):

    def __init__(self, channelRegex='.*'):
        self.channelRegex = channelRegex
        self.knownChannels = set()
        self._sub = None
        self.start()

    def stop(self):
        if self._sub:
            lcmUtils.removeSubscriber(self._sub)
            self._sub = None

    def start(self):
        self.stop()
        self._sub = lcmUtils.addSubscriber(self.channelRegex, callback=self.onMessage)

    def onMessage(self, msgData, channel):
        if channel not in self.knownChannels:
            self.knownChannels.add(channel)
            self.onNewChannel(channel)

    def onNewChannel(self, channel):
        pass


class TfVisualizer(ChannelDiscoverer):

    def __init__(self):
        ChannelDiscoverer.__init__(self, '/tf/.+')
        self.subscribers = {}
        self.tfToWorld = vtk.vtkTransform()
        self.folderName = 'tf'
        self.tfFrames = {}
        self.sub = lcmUtils.addSubscriber('SET_TF_TO_WORLD', lcmbot.rigid_transform_t, self.onSetTfToWorldMessage)

    def onSetTfToWorldMessage(self, msg):
        t = lcmframe.frameFromRigidTransformMessage(msg)
        self.setTfToWorld(t, publish=False)

    def getRootFolder(self):
        return om.getOrCreateContainer(self.folderName)

    def onTfMessage(self, msg, channel):
        t = lcmframe.frameFromRigidTransformMessage(msg)
        frameName = channel[4:]
        self.tfFrames[frameName] = t
        self.updateFrame(frameName)

    def updateFrames(self):
        for frameName in self.tfFrames.keys():
            self.updateFrame(frameName)

    def updateFrame(self, frameName):
        t = transformUtils.concatenateTransforms([self.tfFrames[frameName], self.tfToWorld])
        vis.updateFrame(t, frameName, parent=self.getRootFolder())

    def setTfToWorld(self, transform, publish=True):
        self.tfToWorld = transform
        self.updateFrames()
        if publish:
            self.publishTfToWorld()

    def publishTfToWorld(self):
        msg = lcmframe.rigidTransformMessageFromFrame(self.tfToWorld)
        lcmUtils.publish('SET_TF_TO_WORLD', msg)

    def onNewChannel(self, channel):
        print 'subscribing to channel:', channel
        self.subscribers[channel] = lcmUtils.addSubscriber(channel, lcmbot.rigid_transform_t, self.onTfMessage, callbackNeedsChannel=True)


tfVis = TfVisualizer()
