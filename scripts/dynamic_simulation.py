#!/usr/bin/env python
import openravepy as orpy
import numpy as np
import IPython
import rospy
import tf.transformations as tr

from introduction_to_humanoid_robotics import display

if __name__ == '__main__':
  np.set_printoptions(precision=6, suppress=True)

  env = orpy.Environment()
  object_xml = 'objects/rigid_box.kinbody.xml'
  if not env.Load(object_xml):
    rospy.logerr('Failed to load: {}'.format(object_xml))
    rospy.logerr('Did you run: catkin_make install?')
    exit()
  env.SetViewer('qtcoin')
  viewer = env.GetViewer()
  Tcamera = np.array([[ 0.761479,  0.633475, -0.137328,  0.022017],
                      [ 0.629124, -0.671298,  0.391869, -0.060326],
                      [ 0.156052, -0.384796, -0.909714,  0.153348],
                      [ 0.      ,  0.      ,  0.      ,  1.      ]])
  # viewer.SetCamera(Tcamera)
  rave_box = env.GetKinBody('rigid_box')
  display.set_body_transparency(rave_box, 0.7)
  # h_box = display.draw_axes(env, rave_box.GetTransform(), dist=0.02, linewidth=4)
  h_box = display.drawTransform(env, rave_box.GetTransform(), length=0.02, linewidth=0.001)

  # Load IPython for debug
  IPython.embed()
  env.Reset()
  env.Destroy()


