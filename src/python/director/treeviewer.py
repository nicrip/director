import math
import os
import json
import time
import numpy as np
from collections import namedtuple
from director import lcmUtils

import director.objectmodel as om
import director.applogic as app
from director import transformUtils
from director.debugVis import DebugData
from director import ioUtils
from director import filterUtils
from director.shallowCopy import shallowCopy
from director import vtkAll as vtk
from director import vtkNumpy as vnp
from director import visualization as vis
from director import packagepath

import robotlocomotion as lcmrl

from PythonQt import QtGui

USE_TEXTURE_MESHES = True
USE_SHADOWS = False


class ViewerStatus:
    OK = 0
    MISSING_PATHS = 1
    ERROR_UNKNOWN_FORMAT = -1
    ERROR_UNKNOWN_FORMAT_VERSION = -2
    ERROR_HANDLING_REQUEST = -3
    ERROR_UKNOWN_REQUEST_TYPE = -4


class ViewerResponse(namedtuple("ViewerResponse", ["status", "data"])):
    def toJson(self):
        return dict(status=self.status, **self.data)


def transformFromDict(pose_data):
    return transformUtils.transformFromPose(
        pose_data.get("translation", [0, 0, 0]),
        pose_data.get("quaternion", [1, 0, 0, 0]))


class Geometry(object):
    TextureCache = {}
    PackageMap = None

    @staticmethod
    def createBox(params):
        d = DebugData()
        d.addCube(dimensions=params["lengths"], center=(0, 0, 0))
        return [d.getPolyData()]

    @staticmethod
    def createSphere(params):
        d = DebugData()
        d.addSphere(center=(0, 0, 0), radius=params["radius"])
        return [d.getPolyData()]

    @staticmethod
    def createCylinder(params):
        d = DebugData()
        d.addCylinder(center=(0, 0, 0),
                      axis=(0, 0, 1),
                      radius=params["radius"],
                      length=params["length"])
        return [d.getPolyData()]

    @staticmethod
    def createCapsule(params):
        d = DebugData()
        radius = params["radius"]
        length = params["length"]
        d.addCylinder(center=(0, 0, 0),
                      axis=(0, 0, 1),
                      radius=radius,
                      length=length)
        d.addSphere(center=(0, 0, length / 2.0), radius=radius)
        d.addSphere(center=(0, 0, -length / 2.0), radius=radius)
        return [d.getPolyData()]

    @staticmethod
    def createEllipsoid(params):
        d = DebugData()
        radii = params["radii"]
        d.addEllipsoid(center=(0, 0, 0), radii=radii)
        return [d.getPolyData()]

    @staticmethod
    def createMeshFromFile(params):
        polyDataList = Geometry.loadPolyDataMeshes(params["filename"])
        if "scale" in params:
            polyDataList = Geometry.scaleGeometry(polyDataList,
                                                  params["scale"])
        return polyDataList

    @staticmethod
    def createMeshFromData(params):
        verts = np.asarray(params["vertices"])
        faces = np.asarray(params["faces"])
        return [Geometry.createPolyDataFromMeshArrays(verts, faces)]

    @staticmethod
    def createPointcloud(params):
        polyData = vnp.numpyToPolyData(np.asarray(params["points"]),
                                       createVertexCells=True)
        return [polyData]

    @staticmethod
    def createPlanarLidar(params):
        ranges = np.asarray(params["ranges"])
        angle_stop = (params["angle_start"] +
                      len(ranges) * params["angle_step"])
        angles = np.arange(
            params["angle_start"],
            angle_stop,
            params["angle_step"])
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros(x.shape)
        points = np.vstack((x, y, z)).T
        return [vnp.numpyToPolyData(points, createVertexCells=True)]

    @staticmethod
    def createTriad(params):
        polyData = vis.createAxesPolyData(params.get("scale", 1.0),
                                          params.get("tube", True))
        return [polyData]

    @staticmethod
    def createPolyData(params):
        if params["type"] == "box":
            return Geometry.createBox(params)
        elif params["type"] == "sphere":
            return Geometry.createSphere(params)
        elif params["type"] == "cylinder":
            return Geometry.createCylinder(params)
        elif params["type"] == "capsule":
            return Geometry.createCapsule(params)
        elif params["type"] == "ellipsoid":
            return Geometry.createEllipsoid(params)
        elif params["type"] == "mesh_file":
            return Geometry.createMeshFromFile(params)
        elif params["type"] == "mesh_data":
            return Geometry.createMeshFromData(params)
        elif params["type"] == "pointcloud":
            return Geometry.createPointcloud(params)
        elif params["type"] == "planar_lidar":
            return Geometry.createPlanarLidar(params)
        elif params["type"] == "triad":
            return Geometry.createTriad(params)
        else:
            raise Exception(
                "Unsupported geometry type: {}".format(params["type"]))

    @staticmethod
    def createPolyDataFromMeshArrays(pts, faces):
        pd = vtk.vtkPolyData()
        pd.SetPoints(vtk.vtkPoints())
        pd.GetPoints().SetData(vnp.getVtkFromNumpy(pts.copy()))

        cells = vtk.vtkCellArray()
        for face in faces:
            assert len(face) == 3, "Non-triangular faces are not supported."
            tri = vtk.vtkTriangle()
            tri.GetPointIds().SetId(0, face[0])
            tri.GetPointIds().SetId(1, face[1])
            tri.GetPointIds().SetId(2, face[2])
            cells.InsertNextCell(tri)

        pd.SetPolys(cells)
        return pd

    @staticmethod
    def scaleGeometry(polyDataList, scale):
        if len(scale) == 1:
            scale_x = scale_y = scale_z = scale
        elif len(geom.float_data) == 3:
            scale_x, scale_y, scale_z = scale

        if scale_x != 1.0 or scale_y != 1.0 or scale_z != 1.0:
            t = vtk.vtkTransform()
            t.Scale(scale_x, scale_y, scale_z)
            polyDataList = [filterUtils.transformPolyData(polyData, t) for polyData in polyDataList]

        return polyDataList

    @staticmethod
    def transformGeometry(polyDataList, geom):
        if "transform" in geom:
            t = transformFromDict(geom["transform"])
            polyDataList = [filterUtils.transformPolyData(polyData, t) for polyData in polyDataList]
        return polyDataList

    @staticmethod
    def computeNormals(polyDataList):

        def addNormals(polyData):
            hasNormals = polyData.GetPointData().GetNormals() is not None
            return polyData if hasNormals else filterUtils.computeNormals(polyData)

        return [addNormals(polyData) for polyData in polyDataList]


    @staticmethod
    def getTextureFileName(polyData):
        textureArray = vtk.vtkStringArray.SafeDownCast(polyData.GetFieldData().GetAbstractArray('texture_filename'))
        if not textureArray:
            return None
        return textureArray.GetValue(0)

    @staticmethod
    def loadTextureForMesh(polyData, meshFileName):

        textureFileName = Geometry.getTextureFileName(polyData)
        if textureFileName in Geometry.TextureCache or textureFileName is None:
            return

        if not os.path.isabs(textureFileName):
            baseDir = os.path.dirname(meshFileName)
            imageFile = os.path.join(baseDir, textureFileName)
        else:
            imageFile = textureFileName

        if not os.path.isfile(imageFile):
            print 'cannot find texture file:', textureFileName
            return

        image = ioUtils.readImage(imageFile)
        if not image:
            print 'failed to load image file:', imageFile
            return

        texture = vtk.vtkTexture()
        texture.SetInput(image)
        texture.EdgeClampOn()
        texture.RepeatOn()

        Geometry.TextureCache[textureFileName] = texture

    @staticmethod
    def resolvePackageFilename(filename):

        if not packagepath.PackageMap.isPackageUrl(filename):
            return filename

        if Geometry.PackageMap is None:
            import pydrake
            m = packagepath.PackageMap()
            m.populateFromSearchPaths([pydrake.getDrakePath()])
            m.populateFromEnvironment(['DRAKE_PACKAGE_PATH', 'ROS_PACKAGE_PATH'])
            Geometry.PackageMap = m

        return Geometry.PackageMap.resolveFilename(filename) or filename

    @staticmethod
    def loadPolyDataMeshes(filename):

        filename = Geometry.resolvePackageFilename(filename)
        basename, ext = os.path.splitext(filename)

        preferredExtensions = ['.vtm', '.vtp', '.obj']

        for x in preferredExtensions:
            if os.path.isfile(basename + x):
                filename = basename + x
                break

        if not os.path.isfile(filename):
            print 'warning, cannot find file:', filename
            return []

        if filename.endswith('vtm'):
            polyDataList = ioUtils.readMultiBlock(filename)
        else:
            polyDataList = [ioUtils.readPolyData(filename)]

        if USE_TEXTURE_MESHES:
            for polyData in polyDataList:
                Geometry.loadTextureForMesh(polyData, filename)

        return polyDataList

    @staticmethod
    def createPolyDataForGeometry(geom):
        polyDataList = Geometry.createPolyData(geom)
        polyDataList = Geometry.transformGeometry(polyDataList, geom)
        polyDataList = Geometry.computeNormals(polyDataList)
        return polyDataList

    @staticmethod
    def createGeometry(geom):
        polyDataList = Geometry.createPolyDataForGeometry(geom)

        geometry = []
        for polyData in polyDataList:
            g = Geometry(geom, polyData)
            geometry.append(g)
        return geometry

    @staticmethod
    def addColorChannels(polyData, channels):
        if "intensity" in channels:
            colorBy = "intensity"
            intensity = np.asarray(channels["intensity"]) * 255
            vnp.addNumpyToVtk(polyData,
                              intensity.astype(np.uint8),
                              "intensity")
        if "rgb" in channels:
            colorBy = "rgb"  # default to rgb if provided
            colorArray = np.asarray(channels["rgb"]) * 255
            vnp.addNumpyToVtk(polyData, colorArray.astype(np.uint8), "rgb")

    def __init__(self, geomData, polyData):
        if "channels" in geomData:
            Geometry.addColorChannels(polyData, geomData["channels"])
        self.polyDataItem = vis.PolyDataItem("geometry", polyData, view=None)
        self.polyDataItem._updateColorByProperty()

        color = geomData.get("color", [1, 0, 0, 0.5])
        self.polyDataItem.setProperty('Alpha', color[3])
        self.polyDataItem.actor.SetTexture(
            Geometry.TextureCache.get(
                Geometry.getTextureFileName(polyData)))

        if self.polyDataItem.actor.GetTexture():
            self.polyDataItem.setProperty('Color',
                                          QtGui.QColor(255, 255, 255))
        else:
            self.polyDataItem.setProperty(
                'Color',
                QtGui.QColor(*(255 * np.asarray(color[:3]))))

        if USE_SHADOWS:
            self.polyDataItem.shadowOn()


def findPathToAncestor(fromItem, toItem):
    path = [fromItem]
    while fromItem is not toItem:
        parent = fromItem.parent()
        if parent is None:
            raise ValueError(
                "Cannot find a path from {} to {}".format(fromItem, toItem))
        path.append(parent)
        fromItem = parent
    return path


class TreeViewer(object):
    name = "Remote Tree Viewer"

    def __init__(self, view):

        self.subscribers = []
        self.view = view
        self.itemToPathCache = {}
        self.pathToItemCache = {}
        self.enable()
        self.sendStatusMessage(
            0, ViewerResponse(ViewerStatus.OK, {"ready": True}))

    def _addSubscribers(self):
        self.subscribers.append(lcmUtils.addSubscriber(
            'DIRECTOR_TREE_VIEWER_REQUEST',
            lcmrl.viewer2_comms_t,
            self.onViewerRequest))

    def _removeSubscribers(self):
        for sub in self.subscribers:
            lcmUtils.removeSubscriber(sub)
        self.subscribers = []

    def isEnabled(self):
        return bool(self.subscribers)

    def setEnabled(self, enabled):
        if enabled and not self.isEnabled():
            self._addSubscribers()
        elif not enabled and self.isEnabled():
            self._removeSubscribers()

    def enable(self):
        self.setEnabled(True)

    def disable(self):
        self.setEnabled(False)

    def sendStatusMessage(self, timestamp, response):
        msg = lcmrl.viewer2_comms_t()
        msg.format = "treeviewer_json"
        msg.format_version_major = 1
        msg.format_version_minor = 0
        data = dict(timestamp=timestamp, **response.toJson())
        msg.data = json.dumps(data)
        msg.num_bytes = len(msg.data)
        lcmUtils.publish('DIRECTOR_TREE_VIEWER_RESPONSE', msg)

    def decodeCommsMsg(self, msg):
        if msg.format == "treeviewer_json":
            if msg.format_version_major == 1 and msg.format_version_minor == 0:
                data = json.loads(msg.data)
                return data, ViewerResponse(ViewerStatus.OK, {})
            else:
                return None, ViewerResponse(ViewerStatus.ERROR_UNKNOWN_FORMAT_VERSION,
                                            {"supported_formats": {
                                                 "treeviewer_json": ["1.0"]
                                            }})
        else:
            return None, ViewerResponse(ViewerStatus.ERROR_UNKNOWN_FORMAT,
                                        {"supported_formats": {
                                             "treeviewer_json": ["1.0"]
                                        }})

    def onViewerRequest(self, msg):
        tic = time.time()
        data, response = self.decodeCommsMsg(msg)
        if data is None:
            self.sendStatusMessage(msg.utime,
                                   [responses])
        else:
            tic = time.time()
            responses = self.handleViewerRequest(data)
            tic = time.time()
            self.sendStatusMessage(msg.utime,
                                   responses)

    def handleViewerRequest(self, data):
        deletedPaths = set()
        addedGeometries = set()
        setTransforms = set()
        missingPaths = set()
        for command in data["delete"]:
            deletedPaths.add(tuple(self.handleDeletePath(command)))
        for command in data["load"]:
            addedGeometries.add(tuple(self.handleAddGeometry(command)))
        for command in data["draw"]:
            path, missingGeometry = self.handleSetTransform(command)
            setTransforms.add(tuple(path))
            if missingGeometry:
                missingPaths.add(tuple(path))
        result = {
            "deleted_paths": list(deletedPaths),
            "added_geometries": list(addedGeometries),
            "set_transforms": list(setTransforms),
            "missing_paths": list(missingPaths)
        }
        self.view.render()
        # print "result:", result
        if not missingPaths:
            return ViewerResponse(ViewerStatus.OK, result)
        else:
            return ViewerResponse(ViewerStatus.MISSING_PATHS, result)

    def handleAddGeometry(self, command):
        path = command["path"]
        vtkGeoms = Geometry.createGeometry(command["geometry"])
        return self.addGeometry(path, vtkGeoms)

    def addGeometry(self, path, geomItems):
        folder = self.getPathFolder(path)
        ancestors = findPathToAncestor(folder, self.getRootFolder())
        geomTransform = vtk.vtkTransform()
        for item in reversed(ancestors):
            if not hasattr(item, "transform"):
                item.transform = vtk.vtkTransform()
                item.transform.PostMultiply()
            geomTransform.Concatenate(item.transform)

        for geom in geomItems:
            existing_item = folder.findChild("geometry")
            item = geom.polyDataItem
            if existing_item is not None:
                for prop in existing_item.propertyNames():
                    item.setProperty(prop, existing_item.getProperty(prop))
                om.removeFromObjectModel(existing_item)
            else:
                item.setProperty("Point Size", 2)
                availableColorModes = set(
                    item.getPropertyAttribute('Color By', 'enumNames'))
                for colorBy in ["rgb", "intensity", "Axes"]:
                    if colorBy in availableColorModes:
                        item.setProperty("Color By", colorBy)
                        break

            item.addToView(self.view)
            om.addToObjectModel(item, parentObj=folder)
            item.actor.SetUserTransform(geomTransform)

        return path

    def getPathForItem(self, item):
        return [x.getProperty("Name") for x in reversed(findPathToAncestor(
            item, self.getRootFolder())[:-1])]

    def handleSetTransform(self, command):
        return self._setTransform(command["path"],
                                  transformFromDict(command["transform"]))

    def _setTransform(self, path, transform):
        folder = self.getPathFolder(path)
        if not hasattr(folder, "transform"):
            folder.transform = transform
        else:
            folder.transform.SetMatrix(transform.GetMatrix())
        return path, len(folder.children()) == 0

    def handleDeletePath(self, command):
        path = command["path"]
        item = self.getPathFolder(path)
        if item is not None:
            om.removeFromObjectModel(item)
        return path

    def getRootFolder(self):
        path = tuple()
        if path in self.pathToItemCache:
            return self.pathToItemCache[path]
        else:
            folder = om.getOrCreateContainer(
                self.name.lower(),
                parentObj=om.findObjectByName('scene'))
            self.pathToItemCache[path] = folder
            self.itemToPathCache[folder] = path
            folder.connectRemovedFromObjectModel(self.onItemRemoved)
            return folder

    def onItemRemoved(self, objModel, item):
        if item in self.itemToPathCache:
            path = self.itemToPathCache[item]
            del self.itemToPathCache[item]
            if path in self.pathToItemCache:
                del self.pathToItemCache[path]

    def getPathFolder(self, path):
        path = tuple(path)
        if path in self.pathToItemCache:
            # print "hit for path:", path
            return self.pathToItemCache[path]
        else:
            # print "miss for path:", path
            folder = self.getRootFolder()
            for element in path:
                folder = om.getOrCreateContainer(element, parentObj=folder)
                folder.connectRemovedFromObjectModel(self.onItemRemoved)
            self.pathToItemCache[path] = folder
            self.itemToPathCache[folder] = path
            return folder
