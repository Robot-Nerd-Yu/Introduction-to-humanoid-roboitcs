#!/usr/bin/env python
import openravepy as orpy
import numpy as np
import IPython
import rospy
import tf.transformations as tr

def draw_axes(env, transform, dist=1, linewidth=1):
  h = orpy.misc.DrawAxes(env, transform, dist=dist, linewidth=linewidth)
  return h

def set_body_transparency(kinbody, value=0.5):
  try:
    for link in kinbody.GetLinks():
     for geom in link.GetGeometries():
       geom.SetTransparency(value)
  except Exception, e:
    rospy.logerr('Set kinbody transparency failed due to: '.format(e))

def drawTransform(env, T, length=0.01, linewidth=0.0005):
  """draws a set of arrows around a coordinate system
  """
  return [env.drawarrow(p1=T[0:3, 3], p2=T[0:3, 3] + length * T[0:3, 0], linewidth=linewidth, color=[1.0, 0.0, 0.0]),
          env.drawarrow(p1=T[0:3, 3], p2=T[0:3, 3] + length * T[0:3, 1], linewidth=linewidth, color=[0.0, 1.0, 0.0]),
          env.drawarrow(p1=T[0:3, 3], p2=T[0:3, 3] + length * T[0:3, 2], linewidth=linewidth, color=[0.0, 0.0, 1.0])]
