#!/usr/bin/env python
import openravepy as orpy
import numpy as np
import IPython
import rospy
import tf.transformations as tr
import time

from introduction_to_humanoid_robotics import display, dynamics

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
  Tcamera = np.array([[ 0.49375 ,  0.861569, -0.117938,  0.009389],
                      [ 0.824399, -0.4206  ,  0.378764, -0.027655],
                      [ 0.276727, -0.284243, -0.917948,  0.080582],
                      [ 0.      ,  0.      ,  0.      ,  1.      ]])

  viewer.SetCamera(Tcamera)
  rave_box = env.GetKinBody('rigid_box')
  display.set_body_transparency(rave_box, 0.7)
  # h_box = display.draw_axes(env, rave_box.GetTransform(), dist=0.02, linewidth=4)
  h_box = display.drawTransform(env, rave_box.GetTransform(), length=0.02, linewidth=0.001)

  # Start the simulation
  link = rave_box.GetLinks()[0]
  if link is None:
    rospy.logerr('There is no rigid link in this body.')
    exit(1)

  dt = 0.02
  T = 5.0
  trans = link.GetTransform()
  omega = np.array([1.0, 1.0, 1.0])
  vect_t = np.arange(0, T, dt).tolist() + [T]
  omega_list = [omega, ]
  L_list = []
  for t in range(1, len(vect_t)):
    L, I, omegad = dynamics.get_body_angular_momentum(link=link, omega=omega)
    L_list.append(L)
    R = link.GetTransform()[:3, :3]
    R = np.dot(dynamics.rodrigues(omega*dt), R)
    omega += omegad * dt
    omega_list.append(omega)

    trans[:3,:3] = R
    rave_box.SetTransform(trans)
    time.sleep(dt)

  # Load IPython for debug
  IPython.embed()
  env.Reset()
  env.Destroy()


