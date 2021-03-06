from numpy import array, dot
from math import sin, cos, exp, sqrt, pi, exp

from Robot3WD import Robot

class myRobot(Robot):
    def __init__(self, sampling_period):
        Robot.__init__(self, sampling_period)

    # -------------------------------------------------------------------------#
    # Pre-Lab work for Experiment 2                                            #
    # -------------------------------------------------------------------------#

    def forward_kinematics(self, wheel_angular_velocities, theta):
        L = self._L
        wheel_radius = self._wheel_radius
        m1 = dot((wheel_radius),(array([[(2.0/3.0)*sin(theta), ((1.0/sqrt(3.0))*cos(theta))-((1.0/3.0)*sin(theta)), -((1.0/sqrt(3.0))*cos(theta))-((1.0/3.0)*sin(theta))], [-(2.0/3.0)*cos(theta), ((1.0/sqrt(3.0))*sin(theta))+((1.0/3.0)*cos(theta)), -((1.0/sqrt(3.0))*sin(theta))+((1.0/3.0)*cos(theta))], [-(1.0/(3.0*L)), -(1.0/(3.0*L)), -(1.0/(3.0*L))]])))
        p_dot = dot(m1,wheel_angular_velocities)
	return p_dot

    def inverse_kinematics(self, p_dot, theta):
        L = self._L
        wheel_radius = self._wheel_radius
        m1 = dot((1.0/wheel_radius),(array([[sin(theta), -cos(theta), -L], [sin((pi/3.0)-theta), cos((pi/3.0)-theta), -L], [-sin((pi/3.0)+theta), cos((pi/3.0)+theta), -L]])))
        wheel_angular_velocities = dot(m1,p_dot)
	return wheel_angular_velocities

    def move_left(self, vx, theta):
        p_dot = array([-vx, 0.0, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)

    def move_forward(self, vy, theta):
        p_dot = array([0.0, vy, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)
        
    def move_backward(self, vy, theta):
        p_dot = array([0.0, -vy, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)
        
    def move_right(self, vx, theta):
        p_dot = array([vx, 0.0, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)
        
    def rotate_CCW(self, w, theta):
        p_dot = array([0.0, 0.0, w]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)
        
    def rotate_CW(self, w, theta):
        p_dot = array([0.0, 0.0, -w]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)
