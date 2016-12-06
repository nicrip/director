from director import transformUtils
from director import lcmUtils
from director import objectmodel as om
from director import visualization as vis
from director import applogic as app
from director import ikplanner
from director import vtkNumpy as vnp
import director.vtkAll as vtk
from director.debugVis import DebugData
from director.ikparameters import IkParameters
from director import segmentation

from director.tasks import robottasks as rt
from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit

import os
import sys
import numpy as np
from PythonQt import QtCore, QtGui


class MyPlanner(object):

    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        self.robotModel = robotSystem.robotStateModel
        self.ikPlanner = robotSystem.ikPlanner

    def onTestOne(self):
        print 'one'

    def onTestTwo(self):
        print 'two'


class ImageFitter(ImageBasedAffordanceFit):

    def __init__(self, planner, imageView):
        ImageBasedAffordanceFit.__init__(self, imageView=imageView, numberOfPoints=1)
        self.planner = planner
        self.pointCloudObjectName = 'openni point cloud'

    def getPointCloud(self):
        obj = om.findObjectByName(self.pointCloudObjectName)
        return obj.polyData if obj else vtk.vtkPolyData()

    def fit(self, polyData, points):


        pickPoint = points[0]
        t = vtk.vtkTransform()
        t.Translate(pickPoint)

        polyData = segmentation.cropToBox(polyData, t, [0.3,0.3,0.5])


        import colorsys
        rgb = vnp.getNumpyFromVtk(polyData, 'rgb_colors')/255.0
        hsv = np.array([colorsys.rgb_to_hsv(*t) for t in rgb])
        vnp.addNumpyToVtk(polyData, hsv[:,0].copy(), 'hue')
        vnp.addNumpyToVtk(polyData, hsv[:,1].copy(), 'saturation')
        vnp.addNumpyToVtk(polyData, hsv[:,2].copy(), 'value')


        vis.updatePolyData(polyData, 'crop region', colorByName='rgb_colors', visible=False)

        # hide input data
        #om.findObjectByName(self.pointCloudObjectName).setProperty('Visible', False)

        #cluster = segmentation.makePolyDataFields(polyData)
        #vis.showClusterObjects([cluster], parent='segmentation')

        hueRange = [0.12, 0.14]
        valueRange = [0.5, 1.0]

        points = segmentation.thresholdPoints(segmentation.thresholdPoints(polyData, 'hue', hueRange), 'value', valueRange)
        #points = segmentation.extractLargestCluster(points,  minClusterSize=10, clusterTolerance=0.02)

        vis.updatePolyData(points, 'pole points', color=[0,0,1], visible=False)


        maxZ = np.nanmax(vnp.getNumpyFromVtk(points, 'Points')[:,2])

        print 'maxZ', maxZ
        pickPoint = pickPoint[0], pickPoint[1], maxZ+0.2
        print pickPoint


        t = vtk.vtkTransform()
        t.Translate(pickPoint)

        polyData = segmentation.cropToBox(polyData, t, [0.15,0.15,0.4])

        vis.updatePolyData(polyData, 'object points', colorByName='rgb_colors', visible=False)

        cluster = segmentation.makePolyDataFields(polyData)
        vis.showClusterObjects([cluster], parent='segmentation')


class MyTaskPanel(TaskUserPanel):

    def __init__(self, robotSystem, imageView):

        TaskUserPanel.__init__(self, windowTitle='Task Panel')

        self.planner = MyPlanner(robotSystem)
        self.fitter = ImageFitter(self.planner, imageView)
        self.initImageView(self.fitter.imageView)

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

    def addButtons(self):

        self.addManualButton('test one', self.planner.onTestOne)
        self.addManualButton('test two', self.planner.onTestTwo)
        self.addManualSpacer()

    def addDefaultProperties(self):
        self.params.addProperty('Bool Property', False)

    def onPropertyChanged(self, propertySet, propertyName):
        pass

    def addTasks(self):

        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        def addFunc(name, func, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)

        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder

        def addManipTask(name, planFunc, userPrompt=False):

            prevFolder = self.folder
            addFolder(name, prevFolder)
            addFunc('plan', planFunc)
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc('execute manip plan', self.drillDemo.commitManipPlan)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))
            self.folder = prevFolder

        self.taskTree.removeAllTasks()
        ##################

        addFunc('Test One', self.planner.onTestOne)
        addFunc('Test Two', self.planner.onTestTwo)

