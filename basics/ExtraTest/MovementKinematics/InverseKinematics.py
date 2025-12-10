from .RobotModel import RobotModel

class InverseKinematics:
    def __init__(self, robot, max_fwd_speed = 0.4):
        self.robot = robot
        self.maxSpeed = 8

        self.max_fwd_speed = 0.4                        # maximum achievable x_dot (m/s), forward speed
        self.max_rotation_speed = (self.max_fwd_speed / self.robot.half_wheelbase)               # maximum achievable theta_dot (rad/s), rotational speed

        # Inverse kinematics: [left_wheel, right_wheel] = Ainv @ [dx, dtheta]
        self.Ainv = np.array([
            [1/self.robot.wheel_radius, -self.robot.half_wheelbase/self.robot.wheel_radius],
            [1/self.robot.wheel_radius,  self.robot.half_wheelbase/self.robot.wheel_radius]
        ])

    def getWheelSpeeds(self, robotSpeeds):
        wheelSpeeds = self.Ainv @ robotSpeeds     # matrix multiplication: converts [xdot, thetadot] to [pdl, pdr]
        return np.clip(wheelSpeeds, -self.maxSpeed, self.maxSpeed)       # keep it between -9.7 and +9.7, the max wheel speeds in rad/s

    def phi_influence(self, yValue):
        limit = 0.30                                            # meters to limit influence
        if (yValue < limit and yValue > 0):
            theta_influence = self.max_fwd_speed*0.7*(limit - yValue)       # give theta push only if object is near
        elif (yValue > -limit and yValue < 0):
            theta_influence = -1*self.max_fwd_speed*0.7*(limit - yValue)    # give theta push only if object is near
        else:
            theta_influence = 0
        robotSpeeds = np.array([0, theta_influence])
        wheelSpeeds = self.Ainv @ robotSpeeds
        return clip(wheelSpeeds, -self.maxSpeed, self.maxSpeed)