from director import robotstate
from director import drcargs
from director import transformUtils
from director import ikplanner
from director import ikconstraints
from director import segmentation
from director import visualization as vis
from director import objectmodel as om
from director.debugVis import DebugData
from director import vtkAll as vtk
from director import vtkNumpy as vnp
import numpy as np



def loadFunnelMesh():
    om.removeFromObjectModel(om.findObjectByName('blue_funnel'))

    funnelToWorld = transformUtils.frameFromPositionAndRPY([0.5,0,0.5],[-90,0,0])

    affordanceManager.newAffordanceFromDescription(
      dict(classname='MeshAffordanceItem', Name='blue_funnel',
           pose=transformUtils.poseFromTransform(funnelToWorld),
           Filename='/home/pat/Desktop/blue_funnel.stl'))



def makeReachGoal(name='reach goal'):
    t = transformUtils.frameFromPositionAndRPY([0.7, 0.0, 0.3], [-90, 0, -90])
    return vis.updateFrame(t, 'reach goal').setProperty('Scale', 0.1)


def planNominalPosture():

    #ikPlanner.computePostureGoal(startPose, 'General', 'q_nom')

    startPose = robotSystem.planningUtils.getPlanningStartPose()
    endPose = robotSystem.robotStateJointController.getPose('q_nom')
    ikPlanner.computePostureGoal(startPose, endPose)


def getGraspToHandLink():
    config = drcargs.getDirectorConfig()['endEffectorConfig']
    return transformUtils.frameFromPositionAndRPY(
                          config['graspOffsetFrame'][0],
                          np.degrees(config['graspOffsetFrame'][1]))

_callbackId = None

def planReachGoal(goalFrameName='reach goal', interactive=False):

    goalFrame = om.findObjectByName(goalFrameName).transform
    startPoseName = 'reach_start'
    endPoseName = 'reach_end'

    endEffectorLinkName = 'iiwa_link_ee'
    graspOffsetFrame = getGraspToHandLink()


    startPose = robotSystem.planningUtils.getPlanningStartPose()
    ikPlanner.addPose(startPose, startPoseName)

    constraints = []
    constraints.append(ikPlanner.createPostureConstraint(startPoseName, robotstate.matchJoints('base_')))
    p, q = ikPlanner.createPositionOrientationConstraint(endEffectorLinkName, goalFrame, graspOffsetFrame, positionTolerance=0.0, angleToleranceInDegrees=0.0)
    p.tspan = [1.0, 1.0]
    q.tspan = [1.0, 1.0]


    g = ikconstraints.WorldGazeDirConstraint()
    g.linkName = endEffectorLinkName
    g.targetFrame = goalFrame
    g.targetAxis = [0,1,0]
    g.bodyAxis = list(graspOffsetFrame.TransformVector([0,1,0]))
    g.coneThreshold = 0.0
    g.tspan = [1.0, 1.0]


    constraints.append(p)
    constraints.append(g)

    constraintSet = ikplanner.ConstraintSet(ikPlanner, constraints, endPoseName, startPoseName)

    global _callbackId
    #if _callbackId:
    #    om.findObjectByName(goalFrameName).disconnectFrameModified(_callbackId)

    if interactive:
        def update(frame):
            endPose, info = constraintSet.runIk()
            robotSystem.teleopPanel.showPose(endPose)

        _callbackId = om.findObjectByName(goalFrameName).connectFrameModified(update)
        update(None)

    else:

        robotSystem.teleopPanel.hideTeleopModel()
        constraintSet.runIk()
        print constraintSet.runIkTraj()


def showDebugPoint(p, name='debug point', update=False, visible=True):
    d = DebugData()
    d.addSphere(p, radius=0.01, color=[1,0,0])
    if update:
        vis.updatePolyData(d.getPolyData(), name)
    else:
        vis.showPolyData(d.getPolyData(), name, colorByName='RGB255', visible=visible)


def makeCylinder():
    ''' has properties Radius and Length '''
    desc = dict(classname='CylinderAffordanceItem',
                Name='cylinder',
                pose=transformUtils.poseFromTransform(vtk.vtkTransform()))
    return newAffordanceFromDescription(desc)


def makeBox():
    ''' has property Dimensions '''
    desc = dict(classname='BoxAffordanceItem',
                Name='box',
                pose=transformUtils.poseFromTransform(vtk.vtkTransform()))
    box = newAffordanceFromDescription(desc)
    box.getChildFrame().setProperty('Scale', 0.1)
    return box


def getPointCloud(name='openni point cloud'):
    obj = om.findObjectByName(name)
    return obj.polyData if obj else vtk.vtkPolyData()





def addHSVArrays(polyData, rgbArrayName='rgb_colors'):
    import colorsys
    rgb = vnp.getNumpyFromVtk(polyData, rgbArrayName)/255.0
    hsv = np.array([colorsys.rgb_to_hsv(*t) for t in rgb])
    vnp.addNumpyToVtk(polyData, hsv[:,0].copy(), 'hue')
    vnp.addNumpyToVtk(polyData, hsv[:,1].copy(), 'saturation')
    vnp.addNumpyToVtk(polyData, hsv[:,2].copy(), 'value')


def getMaxZCoordinate(polyData):
    return float(np.nanmax(vnp.getNumpyFromVtk(polyData, 'Points')[:,2]))


def fitSupport(pickPoint=[0.92858565, 0.00213802, 0.30315629]):

    om.removeFromObjectModel(om.findObjectByName('cylinder'))

    polyData = getPointCloud()

    t = vtk.vtkTransform()
    t.Translate(pickPoint)
    polyData = segmentation.cropToBox(polyData, t, [0.3,0.3,0.5])

    addHSVArrays(polyData)

    vis.updatePolyData(polyData, 'crop region', colorByName='rgb_colors', visible=False)

    zMax = getMaxZCoordinate(polyData)

    cyl = makeCylinder()
    cyl.setProperty('Radius', 0.03)
    cyl.setProperty('Length', zMax)

    origin = segmentation.computeCentroid(polyData)
    origin[2] = zMax/2.0

    t = transformUtils.frameFromPositionAndRPY(origin, [0,0,0])
    cyl.getChildFrame().copyFrame(t)


def cropToCylinder(polyData, p1, p2, radius):
    polyData = segmentation.cropToLineSegment(polyData, p1, p2)
    if polyData.GetNumberOfPoints():
        polyData = segmentation.labelDistanceToLine(polyData, p1, p2)
        polyData = segmentation.thresholdPoints(polyData, 'distance_to_line', [0.0, radius])
    return polyData


def getSupportSearchPoint(supportName='cylinder'):

    obj = om.findObjectByName(supportName)
    t = obj.getChildFrame().transform
    zCoord = obj.getProperty('Length')/2.0
    p = np.array(t.TransformPoint([0,0,zCoord]))
    return p


def extractSearchRegionAboveSupport():
    polyData = getPointCloud()
    p1 = getSupportSearchPoint()
    p2 = p1 + np.array([0,0,1.0])*0.3
    polyData = cropToCylinder(polyData, p1, p2, radius=0.1)
    return polyData


def spawnBox():

    t = transformUtils.frameFromPositionAndRPY([0.5,0.0,0.5], [-20,30,0])

    om.removeFromObjectModel(om.findObjectByName('box'))
    obj = makeBox()
    obj.setProperty('Dimensions', [0.06, 0.04, 0.12])
    obj.getChildFrame().copyFrame(t)
    #obj.setProperty('Surface Mode', 'Wireframe')
    obj.setProperty('Color', [1,0,0])


def fitObjectOnSupport():

    polyData = extractSearchRegionAboveSupport()

    origin, edges, wireframe = segmentation.getOrientedBoundingBox(polyData)

    edgeLengths = [float(np.linalg.norm(edge)) for edge in edges]
    axes = [edge / np.linalg.norm(edge) for edge in edges]

    boxCenter = segmentation.computeCentroid(wireframe)

    t = transformUtils.getTransformFromAxes(axes[0], axes[1], axes[2])
    zAxisIndex, zAxis, zAxisSign = transformUtils.findTransformAxis(t, [0,0,1.0])
    xAxisIndex, xAxis, xAxisSign = transformUtils.findTransformAxis(t, [1.0,0,0])

    assert zAxisIndex != xAxisIndex

    #zAxis = zAxis*zAxisSign
    zAxis = [0,0,1.0]
    xAxis = xAxis*xAxisSign
    yAxis = np.cross(zAxis, xAxis)
    xAxis = np.cross(yAxis, zAxis)

    zLength = edgeLengths[zAxisIndex]
    xLength = edgeLengths[xAxisIndex]
    ids = [0,1,2]
    ids.remove(zAxisIndex)
    ids.remove(xAxisIndex)
    yLength = edgeLengths[ids[0]]


    t = transformUtils.getTransformFromAxes(xAxis, yAxis, zAxis)
    t.PostMultiply()
    t.Translate(boxCenter)

    om.removeFromObjectModel(om.findObjectByName('box'))
    obj = makeBox()
    obj.setProperty('Dimensions', [xLength, yLength, zLength])
    obj.getChildFrame().copyFrame(t)
    #obj.getChildFrame().setProperty('Visible', True)
    obj.setProperty('Surface Mode', 'Wireframe')
    obj.setProperty('Color', [1,0,0])



def addGraspFrames():

    obj = om.findObjectByName('box')
    om.removeFromObjectModel(obj.findChild('grasp to world'))
    om.removeFromObjectModel(obj.findChild('pregrasp to world'))

    dims = obj.getProperty('Dimensions')

    objectToWorld = obj.getChildFrame().transform

    graspToObject = transformUtils.frameFromPositionAndRPY([0.0,0.0,dims[2]/2.0 - 0.025], [0,50,0])
    preGraspToGrasp = transformUtils.frameFromPositionAndRPY([-0.08, 0.0, 0.0], [0,0,0])

    graspToWorld = transformUtils.concatenateTransforms([graspToObject, objectToWorld])
    preGraspToWorld = transformUtils.concatenateTransforms([preGraspToGrasp, graspToWorld])

    graspFrame = vis.updateFrame(graspToWorld, 'grasp to world', scale=0.1, parent=obj, visible=False)
    obj.getChildFrame().getFrameSync().addFrame(graspFrame, ignoreIncoming=True)

    preGraspFrame = vis.updateFrame(preGraspToWorld, 'pregrasp to world', scale=0.1, parent=obj, visible=False)
    graspFrame.getFrameSync().addFrame(preGraspFrame, ignoreIncoming=True)


def init(robotSystem_):
    global robotSystem, affordanceManager, ikPlanner, newAffordanceFromDescription

    robotSystem = robotSystem_
    affordanceManager = robotSystem.affordanceManager
    ikPlanner = robotSystem.ikPlanner
    newAffordanceFromDescription = robotSystem.affordanceManager.newAffordanceFromDescription


#init(robotSystem)
#fitSupport()
#showDebugPoint(getSupportSearchPoint())

#fitObjectOnSupport()
#addGraspFrames()
#vis.updatePolyData(extractSearchRegionAboveSupport(), name='search region', color=[0,1,0])

