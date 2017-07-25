#!/usr/bin/env python
import openravepy as orpy
import numpy as np
import IPython
import rospy
import tf.transformations as tr

def get_body_angular_momentum(link, omega):
  """
  Given the angular velocity, compute the anglular momentum.

  Paremeters:
  -----------
  link: OpenRAVE.kinbodys
    The rigid body from which to get its angular momentum.
  omega: numpy.array, shape=(3,)
    The vector of angular velocity.

  Returns:
  --------
  L: numpy.array, shape=(3,)
    Angular mometum.
  I: numpy.array, shape=(3,3)
    Inertial Matrix.
  omegad: numpy.array, shape=(3,)
    Angular accelaration.
  """
  R = link.GetTransform()[:3, :3]
  I_local = link.GetLocalInertia()
  assert R is not None
  assert I_local is not None
  I = np.dot(R, np.dot(I_local, R.T))
  L = np.dot(I, omega)
  omegad = - np.dot(np.linalg.inv(I), np.cross(omega, L))
  return L, I, omegad

def cross_mat(omega):
  """
  Compute the skew symmetric matrix.
  """
  omega = np.asarray(omega)
  assert omega.shape[0] == 3
  wx, wy, wz = omega
  sn = np.array([[0, -wz, wy],
                 [wz, 0, -wx],
                 [-wy, wx, 0]])
  return sn

def rodrigues(r):
  """
  Rodrigues' formula.
  """
  r = np.asarray(r)
  assert r.shape[0] == 3
  theta = tr.vector_norm(r)
  a = tr.unit_vector(r)
  sn = cross_mat(a)
  R = np.eye(3) + np.sin(theta)*sn + (1-np.cos(theta))*np.dot(sn, sn)
  return R