"""go1wrapper

Go1 pybullet interface using pinocchio's convention.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""
import numpy as np
import pybullet

from bullet_utils.wrapper import PinBulletWrapper
from robot_properties_go1.config import Go1Config

dt = 1e-3


class Go1Robot(PinBulletWrapper):
    def __init__(
        self,
        pos=None,
        orn=None,
        useFixedBase=False,
        init_sliders_pose=4
        * [
            0,
        ],
    ):

        # Load the robot
        if pos is None:
            pos = [0.0, 0, 0.40]
        if orn is None:
            orn = pybullet.getQuaternionFromEuler([0, 0, 0])

        pybullet.setAdditionalSearchPath(Go1Config.resources.package_path)
        self.urdf_path = Go1Config.urdf_path
        self.robotId = pybullet.loadURDF(
            self.urdf_path,
            pos,
            orn,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=useFixedBase,
        )
        pybullet.getBasePositionAndOrientation(self.robotId)

        # Create the robot wrapper in pinocchio.
        self.pin_robot = Go1Config.buildRobotWrapper()

        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        for ji in range(num_joints):
            pybullet.changeDynamics(
                self.robotId,
                ji,
                linearDamping=0.04,
                angularDamping=0.04,
                restitution=0.0,
                lateralFriction=0.5,
            )

        self.slider_a = pybullet.addUserDebugParameter(
            "a", 0, 1, init_sliders_pose[0]
        )
        self.slider_b = pybullet.addUserDebugParameter(
            "b", 0, 1, init_sliders_pose[1]
        )
        self.slider_c = pybullet.addUserDebugParameter(
            "c", 0, 1, init_sliders_pose[2]
        )
        self.slider_d = pybullet.addUserDebugParameter(
            "d", 0, 1, init_sliders_pose[3]
        )

        self.base_link_name = "base"
        self.end_eff_ids = []
        self.end_effector_names = []
        controlled_joints = []

        for leg in ["FL", "FR", "RL", "RR"]:
            controlled_joints += [leg + "_hip_joint", leg + "_thigh_joint", leg + "_calf_joint"]
            self.end_eff_ids.append(
                self.pin_robot.model.getFrameId(leg + "_foot_fixed")
            )
            self.end_effector_names.append(leg + "_foot_fixed")

        self.joint_names = controlled_joints
        self.nb_ee = len(self.end_effector_names)

        # self.hl_index = self.pin_robot.model.getFrameId("RL_ANKLE")
        # self.hr_index = self.pin_robot.model.getFrameId("RR_ANKLE")
        # self.fl_index = self.pin_robot.model.getFrameId("FL_ANKLE")
        # self.fr_index = self.pin_robot.model.getFrameId("FR_ANKLE")

        # Creates the wrapper by calling the super.__init__.
        super(Go1Robot, self).__init__(
            self.robotId,
            self.pin_robot,
            controlled_joints,
            ["FL_foot_fixed", "FR_foot_fixed", "RR_foot_fixed", "RL_foot_fixed"],
        )

    def forward_robot(self, q=None, dq=None):
        if not q:
            q, dq = self.get_state()
        elif not dq:
            raise ValueError("Need to provide q and dq or non of them.")

        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)

    def get_slider_position(self, letter):
        try:
            if letter == "a":
                return pybullet.readUserDebugParameter(self.slider_a)
            if letter == "b":
                return pybullet.readUserDebugParameter(self.slider_b)
            if letter == "c":
                return pybullet.readUserDebugParameter(self.slider_c)
            if letter == "d":
                return pybullet.readUserDebugParameter(self.slider_d)
        except Exception:
            # In case of not using a GUI.
            return 0.

    def reset_to_initial_state(self) -> None:
        """Reset robot state to the initial configuration (based on Go1Config)."""
        q0 = np.matrix(Go1Config.initial_configuration).T
        dq0 = np.matrix(Go1Config.initial_velocity).T
        self.reset_state(q0, dq0)

class Go1RobotWithoutPybullet():
    """
    Similar to the class above, but without PyBullet. Used for ROS + Gazebo and Isaac projects
    """
    def __init__(self):

        self.urdf_path = Go1Config.urdf_path

        # Create the robot wrapper in pinocchio.
        self.pin_robot = Go1Config.buildRobotWrapper()

        self.base_link_name = "base"
        self.end_eff_ids = []
        self.end_effector_names = []
        controlled_joints = []

        for leg in ["FL", "FR", "RL", "RR"]:
            controlled_joints += [leg + "_hip_joint", leg + "_thigh_joint", leg + "_calf_joint"]
            self.end_eff_ids.append(
                self.pin_robot.model.getFrameId(leg + "_foot_fixed")
            )
            self.end_effector_names.append(leg + "_foot_fixed")

        self.joint_names = controlled_joints
        self.nb_ee = len(self.end_effector_names)

        # self.hl_index = self.pin_robot.model.getFrameId("RL_ANKLE")
        # self.hr_index = self.pin_robot.model.getFrameId("RR_ANKLE")
        # self.fl_index = self.pin_robot.model.getFrameId("FL_ANKLE")
        # self.fr_index = self.pin_robot.model.getFrameId("FR_ANKLE")

    def forward_robot(self, q=None, dq=None):
        if q is None:
            q, dq = self.get_state()
        elif dq is None:
            raise ValueError("Need to provide q and dq or non of them.")

        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)

    def reset_to_initial_state(self) -> None:
        """Reset robot state to the initial configuration (based on Go1Config)."""
        q0 = np.array(Go1Config.initial_configuration)
        dq0 = np.array(Go1Config.initial_velocity)
        self.reset_state(q0, dq0)
    
    def update_pinocchio(self, q, dq):
        """Updates the pinocchio robot.
        This includes updating:
        - kinematics
        - joint and frame jacobian
        - centroidal momentum
        Args:
          q: Pinocchio generalized position vector.
          dq: Pinocchio generalize velocity vector.
        """
        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)
