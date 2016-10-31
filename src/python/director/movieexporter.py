from director.consoleapp import ConsoleApp
from director import lcmUtils
from director.timercallback import TimerCallback
from director import cameracontrol
import lcm
import sys
import numpy as np
import time

from director import lcmlogplayer

class POVExporter(object):

    def __init__(self, view):

        self.outDir = os.path.expanduser('~/trisolMnt/Desktop/pov')
        if not os.path.isdir(self.outDir):
            os.makedirs(self.outDir)

        self.exporter = vtk.vtkPOVExporter2()
        self.exporter.SetRenderWindow(view.renderWindow())
        self.view = view
        self.nextFrame = 0

    def resetFrameCounter(self):
        self.nextFrame = 0

    def writeTest(self):
        outFile = os.path.join(self.outDir, 'test.pov')
        self.exporter.SetFileName(outFile)
        print 'writing pov file:', outFile
        self.exporter.Write()

    def writeNext(self):
        outFile = os.path.join(self.outDir, 'out_%07d.pov' % self.nextFrame)
        self.exporter.SetFileName(outFile)
        print 'writing pov file:', outFile
        self.exporter.Write()
        self.nextFrame += 1


class MovieExporter(object):

    def __init__(self):
        self.timer = TimerCallback()
        self.timer.callback = self.exportNextFrame
        self.start = self.timer.start
        self.stop = self.timer.stop

    def init(self, framesPerSecond=30, playLength=10):
        self.framesPerSecond = framesPerSecond
        self.playLength = playLength
        self.numFrames = int(playLength*framesPerSecond)
        self.dt = 1.0 / float(framesPerSecond)
        self.nextFrame = 0
        self.reset()

    def exportNextFrame(self):
        tNow = self.dt*self.nextFrame
        percentComplete = self.nextFrame/float(self.numFrames)
        self.tick(tNow, self.dt, self.nextFrame, percentComplete)

        self.nextFrame += 1
        if self.nextFrame >= self.numFrames:
            return False

    def reset(self):
        pass

    def tick(self, tNow, dt, frameIndex, percentComplete):
        pass


class MyMovieExporter(MovieExporter):

    def __init__(self, view):
        MovieExporter.__init__(self)
        self.poseInterpolator = None
        self.logPlayer = None
        self.logStartTime = 0.0
        self.povExporter = POVExporter(view)
        self.povExportEnabled = True
        self.cameraInterpolator = None
        self.view = view
        self.callbacks = []

    def setPlan(self, plan):
        self.poseInterpolator = planPlayback.getPoseInterpolatorFromPlan(plan)
        poseTimes, poses = planPlayback.getPlanPoses(plan)
        self.planEndTime = poseTimes[-1]

    def loadLogFile(self, filename):
        self.logPlayer = lcmlogplayer.LcmLogPlayer()
        self.logPlayer.readLog(filename, 306)

    def initCameraInterpolator(self):
        self.cameraInterpolator = cameracontrol.CameraInterpolator(self.view)

    def reset(self):
        self.povExporter.nextFrame = 0
        if self.logPlayer:
            self.logPlayer.skipToTime(self.logStartTime)
            self.logPlayer.resetPlayPosition(self.logStartTime)


    def tick(self, tNow, dt, frameIndex, percentComplete):

        if self.poseInterpolator:
            q = self.poseInterpolator(tNow)
            robotStateJointController.setPose('animate', q)

        if self.logPlayer:
            self.logPlayer.advanceTime(dt)

        if self.cameraInterpolator:
            self.cameraInterpolator.setViewCameraAtTime(tNow)

        for callback in self.callbacks:
            callback()

        # cleanNans(om.findObjectByName('HEIGHT_MAP_SCENE'))
        #self.view.forceRender()

        if self.povExportEnabled:
            self.povExporter.writeNext()


def setViewOptions():
    viewOptions.setProperty('Gradient background', False)
    viewOptions.setProperty('Background color', [1,1,1])
    grid.setProperty('Color', [0.9,0.9,0.9])
    grid.setProperty('Surface Mode', 'Surface')
    grid.setProperty('Alpha', 1.0)
    grid.setProperty('Visible', False)

    aspectRatio = 3/4.0
    viewWidth = 800
    viewHeight = int(viewWidth * aspectRatio)
    view.setFixedSize(viewWidth, viewHeight)


def cleanNans(obj):
    pd = obj.polyData
    filterUtils.labelNonFinitePoints(pd)
    nonFinite = vnp.getNumpyFromVtk(pd, 'is_nonfinite')
    pts = vnp.getNumpyFromVtk(pd, 'Points')
    pts[nonFinite==1] = [0.0, 0.0, 0.0]


def snapRobotToGround():

    minZ = 1e5

    for linkName in [robotSystem.ikPlanner.leftFootLink, robotSystem.ikPlanner.rightFootLink]:
        contactPts = robotSystem.ikPlanner.robotModel.getLinkContactPoints(linkName)
        linkFrame = robotSystem.robotStateModel.getLinkFrame(linkName)

        for p in contactPts:
            p = linkFrame.TransformPoint(p)

            if p[2] < minZ:
                minZ = p[2]

    moveDelta = 0.0 - minZ
    robotSystem.robotStateJointController.q[2] += moveDelta
    robotSystem.robotStateJointController.push()


filename = '/home/pat/trisolMnt/logs/raw/lcmlog-2014-11-06-continuous-kintinuous-long-stack-success'
filename = '/home/pat/trisolMnt/logs/raw/lcmlog__2015-08-18__11-17-02-417057__bbc-1'
filename = '/media/pat/SSD_240GB/logs/raw/lcmlog__2015-08-18__11-17-02-417057__bbc-1'

if 'logPlayer' not in globals():
    logPlayer = lcmlogplayer.LcmLogPlayer()
    logPlayer.readLog(filename, 306)

ex = MyMovieExporter(view)
ex.logPlayer = logPlayer
ex.logStartTime = 293


