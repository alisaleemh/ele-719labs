from numpy import array, dot
from math import sin, cos, exp, sqrt, pi, exp

from Robot3WD import Robot

class myRobot(Robot):
    def __init__(self, sampling_period):
        Robot.__init__(self, sampling_period)

    # -------------------------------------------------------------------------#
    # Pre-Lab work for Experiment 2                                            #
    # -------------------------------------------------------------------------#

    def inverse_kinematics(self, p_dot, theta):
        L = self._L
        wheel_radius = self._wheel_radius
        R= array([[sin(theta), -cos(theta), -L],
                  [sin((pi/3.0)-theta), cos((pi/3.0)-theta), -L],
                  [-sin((pi/3.0) + theta), cos((pi/3.0) + theta), -L]])

        M_Inverse=(1/wheel_radius)*R
        P=array(p_dot).T
        wheel_angular_velocities=dot(M_Inverse, P)
        return wheel_angular_velocities

    def forward_kinematics(self, wheel_angular_velocities, theta):
        L = self._L
        wheel_radius = self._wheel_radius
        R= array([[ (2.0/3.0)*sin(theta),(sqrt(3.0)/3.0)*cos(theta)-(1.0/3.0)*sin(theta), (-sqrt(3.0)/(3.0))*cos(theta)-(1.0/3.0)*sin(theta)],
                  [(-2.0/3.0)*cos(theta),(sqrt(3.0)/3.0)*sin(theta)+(1.0/3.0)*cos(theta), (-sqrt(3.0)/3.0)*sin(theta)+(1.0/3.0)*cos(theta)],
                  [(-1.0)/(3.0*L), -(1.0/(3.0*L)), -(1.0/(3.0*L))]])

        M_Forward=(wheel_radius)*R
        A=array(wheel_angular_velocities).T
        #p_dot=dot(M_Forward, wheel_angular_velocities)
        p_dot=dot(M_Forward, A)
        return p_dot

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

    def HMatrix(self,q):
        H = array([[cos(q[2]) ,-sin(q[2]) ,q[0] ], [sin(q[2]) ,cos(q[2]) ,q[1] ], [0 ,0 ,1 ]])
        return H

   
    def Vraw_to_distance(self,Vraw):
        d = (0.62089)*(exp((-0.0540)*(sqrt(Vraw))))
        return d
