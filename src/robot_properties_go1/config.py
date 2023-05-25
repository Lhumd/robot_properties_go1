"""config

Store the configuration of the Go1 family robots.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import numpy as np
from math import pi
from os.path import join, dirname
from os import environ
import pinocchio as se3
from pinocchio.utils import zero
from pinocchio.robot_wrapper import RobotWrapper
from robot_properties_go1.resources import Resources

class Go1Abstract(object):
    """ Abstract class used for all Go1 robots. """

    # PID gains
    kp = 20.0
    kd = 0.5
    ki = 0.0

    # The Kt constant of the motor [Nm/A]: tau = I * Kt.
    motor_torque_constant = 0.025 #Florian

    # Control time period.
    control_period = 0.001
    dt = control_period

    # MaxCurrent = 12 # Ampers
    max_current = 2 #Florian

    # Maximum torques.
    max_torque = motor_torque_constant * max_current

    # Maximum control one can send, here the control is the current.
    max_control = max_current

    # ctrl_manager_current_to_control_gain I am not sure what it does so 1.0.
    ctrl_manager_current_to_control_gain = 1.0

    max_qref = pi

    base_link_name = "base_link"
    end_effector_names = ["HL_ANKLE", "HR_ANKLE", "FL_ANKLE", "FR_ANKLE"]

    rot_base_to_imu = np.identity(3)
    r_base_to_imu = np.zeros(3)

    @classmethod
    def buildRobotWrapper(cls):
        # Rebuild the robot wrapper instead of using the existing model to
        # also load the visuals.
        print("URDF path", cls.urdf_path)
        print("Meshes path", cls.meshes_path)
        robot = RobotWrapper.BuildFromURDF(
            cls.urdf_path, cls.meshes_path, se3.JointModelFreeFlyer()
        )
        robot.model.rotorInertia[6:] = cls.motor_inertia
        robot.model.rotorGearRatio[6:] = cls.motor_gear_ration
        return robot

    def joint_name_in_single_string(self):
        joint_names = ""
        for name in self.robot_model.names[2:]:
            joint_names += name + " "
        return joint_names


class Go1Config(Go1Abstract):
    robot_family = "go1"
    robot_name = "go1"

    print("robot_path ", Resources(robot_name))
    resources = Resources(robot_name)
    meshes_path = resources.meshes_path
    dgm_yaml_path = resources.dgm_yaml_path
    urdf_path = resources.urdf_path
    ctrl_path = resources.imp_ctrl_yaml_path

    # The inertia of a single blmc_motor.
    motor_inertia = 0.0000045 #Florian

    # The motor gear ratio.
    motor_gear_ration = 9.0 #Florian

    # pinocchio model.
    print("URDF path", urdf_path)
    print("Meshes path", meshes_path)
    pin_robot_wrapper = RobotWrapper.BuildFromURDF(
        urdf_path, meshes_path, se3.JointModelFreeFlyer()
    )
    pin_robot_wrapper.model.rotorInertia[6:] = motor_inertia
    pin_robot_wrapper.model.rotorGearRatio[6:] = motor_gear_ration
    pin_robot = pin_robot_wrapper

    robot_model = pin_robot_wrapper.model
    mass = np.sum([i.mass for i in robot_model.inertias])
    base_name = robot_model.frames[2].name

    # End effectors informations
    shoulder_ids = []
    end_eff_ids = []
    shoulder_names = []
    end_effector_names = []
    for leg in ["FL", "FR", "RL", "RR"]:
        shoulder_ids.append(robot_model.getFrameId(leg + "_hip_joint"))
        shoulder_names.append(leg + "_thigh_joint")
        end_eff_ids.append(robot_model.getFrameId(leg + "_foot_fixed"))
        end_effector_names.append(leg + "_foot_fixed")

    nb_ee = len(end_effector_names)
    hl_index = robot_model.getFrameId("HL_ANKLE")
    hr_index = robot_model.getFrameId("HR_ANKLE")
    fl_index = robot_model.getFrameId("FL_ANKLE")
    fr_index = robot_model.getFrameId("RR_ANKLE")

    # The number of motors, here they are the same as there are only revolute
    # joints.
    nb_joints = robot_model.nv - 6

    joint_names = [
        "FL_hip_joint",
        "FL_thigh_joint",
        "FL_calf_joint",
        "FR_hip_joint",
        "FR_thigh_joint",
        "FR_calf_joint",
        "RL_hip_joint",
        "RL_thigh_joint",
        "RL_calf_joint",
        "RR_hip_joint",
        "RR_thigh_joint",
        "RR_calf_joint",
    ]

    # Mapping between the ctrl vector in the device and the urdf indexes.
    urdf_to_dgm = tuple(range(12))

    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(
        zip(
            robot_model.names[1:],
            robot_model.lowerPositionLimit,
            robot_model.upperPositionLimit,
        )
    ):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]

    # Define the initial state.
    initial_configuration = (
        [0.0, 0.0, 0.35, 0.0, 0.0, 0.0, 1.0]
        + [0.2, 1.0, -1.5]
        + [-0.2, 1.0, -1.5]
        + [0.2, 0.8, -1.5]
        + [-0.2, 0.8, -1.5]
    )
    initial_velocity = (8 + 4 + 6) * [
        0,
    ]

    q0 = zero(robot_model.nq)
    q0[:] = initial_configuration
    v0 = zero(robot_model.nv)
    a0 = zero(robot_model.nv)

    base_p_com = [0.0, 0.0, -0.02]#Florian

    rot_base_to_imu = np.identity(3)
    r_base_to_imu = np.array([0.10407, -0.00635, 0.01540])#Florian
