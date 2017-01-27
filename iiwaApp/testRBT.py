import os
import sys

sys.path.append(os.path.join(os.environ['DRAKE_BASE'], 'build/install/lib/python2.7/dist-packages'))
sys.path.append(os.path.join(os.environ['DRAKE_BASE'], 'build/install/lib/python2.7/site-packages'))



import numpy as np
import pydrake


r = pydrake.rbtree.RigidBodyTree()

fname = os.path.join(pydrake.getDrakePath(), "examples/kuka_iiwa_arm/urdf/iiwa14_no_collision.urdf")
fname = '/home/pat/source/drake/drake/drake/examples/kuka_iiwa_arm/urdf/iiwa14_no_collision.urdf'

urdfString = open(fname, 'r').read()

packageMap = pydrake.rbtree.mapStringString()

baseDir = os.path.dirname(fname)

print 'baseDir:', baseDir

print 'loading'
r = pydrake.rbtree.RigidBodyTree()
r.addRobotFromURDFString(urdfString, packageMap, baseDir)
print 'do kin'
kinsol = r.doKinematics(np.zeros((7,1)), np.zeros((7,1)))
print 'done'
