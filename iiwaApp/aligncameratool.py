from director import objectmodel as om
from director import ioUtils
from director import visualization as vis
from director import transformUtils
from director import filterUtils
from director import segmentation
from director import pointpicker
from director import lcmframe
from director import lcmUtils
from director import applogic
from director.debugVis import DebugData
from director import viewbehaviors
from director import vtkAll as vtk
from director import vtkNumpy as vnp
import numpy as np

from director.tasks.taskuserpanel import ImageBasedAffordanceFit

import PythonQt
from PythonQt import QtGui, QtCore


class ImageFitter(ImageBasedAffordanceFit):

    def __init__(self, parent):
        ImageBasedAffordanceFit.__init__(self, imageView=parent.cameraView, numberOfPoints=3)
        self.parent = parent
        self.points = None
        self.pointCloudObjectName = 'openni point cloud'

    def getPointCloud(self):
        obj = om.findObjectByName(self.pointCloudObjectName)
        return obj.polyData if obj else vtk.vtkPolyData()

    def fit(self, polyData, points):
        self.points = points
        d = DebugData()
        for p in points:
            d.addSphere(p, radius=0.01)
        vis.showPolyData(d.getPolyData(), 'image pick points', view=self.parent.view, color=[1,0,0])
        self.parent.onImagePick()


def computeLandmarkTransform(sourcePoints, targetPoints):
    '''
    Returns a vtkTransform for the transform sourceToTarget
    that can be used to transform the source points to the target.
    '''
    sourcePoints = vnp.getVtkPointsFromNumpy(sourcePoints)
    targetPoints = vnp.getVtkPointsFromNumpy(targetPoints)

    f = vtk.vtkLandmarkTransform()
    f.SetSourceLandmarks(sourcePoints)
    f.SetTargetLandmarks(targetPoints)
    f.SetModeToRigidBody()
    f.Update()

    mat = f.GetMatrix()
    t = vtk.vtkTransform()
    t.PostMultiply()
    t.SetMatrix(mat)
    return t


class TestFitCamera(object):

    def __init__(self, robotSystem, cameraView):

        self.meshPoints = None
        self.cameraView = cameraView

        self.robotMesh = vtk.vtkPolyData()
        robotSystem.robotStateModel.model.getModelMesh(self.robotMesh)


        self.view = PythonQt.dd.ddQVTKWidgetView()
        vis.showPolyData(self.robotMesh, 'robot mesh', view=self.view)

        self.imageFitter = ImageFitter(self)

        vis.showPolyData(self.imageFitter.getPointCloud(), 'pointcloud', view=self.view, colorByName='rgb_colors', visible=False)

        self.picker = pointpicker.PointPicker(self.view)
        self.picker.pickType = 'cells'
        self.picker.numberOfPoints = 3
        self.picker.annotationName = 'mesh annotation'
        self.picker.annotationFunc = self.onPickPoints
        self.picker.start()

        self.widget = QtGui.QWidget()
        layout = QtGui.QHBoxLayout(self.widget)
        layout.addWidget(self.cameraView.view)
        layout.addWidget(self.view)
        self.widget.resize(800, 400)
        self.widget.setWindowTitle('Camera Alignment Tool')
        self.widget.show()

        self.viewBehaviors = viewbehaviors.ViewBehaviors(self.view)
        applogic.resetCamera(viewDirection=[0,1,0], view=self.view)
        applogic.setCameraTerrainModeEnabled(self.view, True)

    def onImagePick(self):
        self.align()

    def onPickPoints(self, *points):

        self.meshPoints = points

        d = DebugData()
        for p in points:
            d.addSphere(p, radius=0.01)
        vis.showPolyData(d.getPolyData(), 'mesh pick points', color=[0,1,0], view=self.view)

        self.align()

    def align(self):
        if None in [self.meshPoints, self.imageFitter.points]:
            return

        t1 = computeLandmarkTransform(self.imageFitter.points, self.meshPoints)
        polyData = filterUtils.transformPolyData(self.imageFitter.getPointCloud(), t1)

        vis.showPolyData(polyData, 'transformed pointcloud', view=self.view, colorByName='rgb_colors', visible=False)

        polyData = segmentation.cropToBounds(polyData, vtk.vtkTransform(), [[-0.5,0.50],[-0.5,0.5],[0.13,1.5]])
        #polyData = segmentation.applyVoxelGrid(polyData, leafSize=0.01)
        #polyData = segmentation.applyEuclideanClustering(polyData, clusterTolerance=0.04)
        #polyData = segmentation.thresholdPoints(polyData, 'cluster_labels', [1,1])

        vis.showPolyData(polyData, 'filtered points for icp', color=[0,1,0], view=self.view, visible=False)

        t2 = segmentation.applyICP(polyData, self.robotMesh)

        cameraToWorld = transformUtils.concatenateTransforms([t1, t2])
        polyData = filterUtils.transformPolyData(self.imageFitter.getPointCloud(), cameraToWorld)
        vis.showPolyData(polyData, 'aligned pointcloud', colorByName='rgb_colors', view=self.view, visible=True)

        cameraToWorldMsg = lcmframe.rigidTransformMessageFromFrame(cameraToWorld)
        lcmUtils.publish('OPENNI_FRAME_LEFT_TO_LOCAL', cameraToWorldMsg)


def main(robotSystem, cameraView):

    global w
    w = TestFitCamera(robotSystem, cameraView)
