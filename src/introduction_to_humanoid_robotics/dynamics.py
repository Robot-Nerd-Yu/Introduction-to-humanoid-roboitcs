#!/usr/bin/env python
import openravepy as orpy
import numpy as np
import IPython
import rospy
import tf.transformations as tr

def get_body_angular_momentum(link, omega):
  R = link.GetTransform()[:3, :3]
  I_local = link.GetLocalInertia()
  I = np.dot(R, np.dot(I_local, R.T))
  L = np.dot(I, omega)
  return L

