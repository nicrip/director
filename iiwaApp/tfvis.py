from director import objectmodel as om
from director import lcmframe
from director import lcmUtils
import bot_core as lcmbot


def onFrameMessage(msg, channel):
    folder = om.getOrCreateContainer('tf')
    t = lcmframe.frameFromRigidTransformMessage(msg)
    vis.updateFrame(t, channel[4:], parent=folder)


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

    def start(self, channelRegex):
        self.stop()
        self._sub = lcmUtils.addSubscriber(self.channelRegex, callback=self.onMessage)

    def onMessage(self, msgData, channel):
        if channel not in self.knownChannels:
            self.knownChannels.add(channel)
            self.onNewChannel(channel)

    def onNewChannel(self, channel):
        pass


class MyChannelDiscoverer(ChannelDiscoverer):

    def __init__(self):
        ChannelDiscoverer.__init__(self, '/tf/.+')
        self.subscribers = {}

    def onNewChannel(self, channel):
        print 'subscribing to channel:', channel
        self.subscribers[channel] = lcmUtils.addSubscriber(channel, lcmbot.rigid_transform_t, onFrameMessage, callbackNeedsChannel=True)


d = MyChannelDiscoverer()
