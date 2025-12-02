import casadi                                                                       
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin                             
from pinocchio import casadi as cpin                
from pinocchio.robot_wrapper import RobotWrapper    
from pinocchio.visualize import MeshcatVisualizer   
from utils.weighted_moving_filter import WeightedMovingFilter
from g1_29_visualizer import G1_29_Visualizer

class G1_29_ArmIK:
    def __init__(self, Unit_Test = False, Visualization=False):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        self.Unit_Test = Unit_Test

        folder_path = "../assets/data/"
        file_name = "g1_inspire_gen4.urdf"

        if not self.Unit_Test:
            self.robot = pin.RobotWrapper.BuildFromURDF((folder_path+file_name), folder_path)
        else:
            self.robot = pin.RobotWrapper.BuildFromURDF((folder_path+file_name), folder_path) # for test

        self.mixed_jointsToLockIDs = [
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
        ]

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )

        self.reduced_robot.model.addFrame(
            pin.Frame('L_ee',
                      self.reduced_robot.model.getJointId('left_wrist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.05,0,0]).T),
                      pin.FrameType.OP_FRAME)
        )
        
        self.reduced_robot.model.addFrame(
            pin.Frame('R_ee',
                      self.reduced_robot.model.getJointId('right_wrist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.05,0,0]).T),
                      pin.FrameType.OP_FRAME)
        )

        self.joint_index_map = {}
        for i, name in enumerate(self.reduced_robot.model.names):
            self.joint_index_map[name] = i-1
        
        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1) 
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # Get the hand joint ID and define the error function
        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - self.cTf_l[:3,3],
                    self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3,3]
                )
            ],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.L_hand_id].rotation @ self.cTf_l[:3,:3].T),
                    cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ self.cTf_r[:3,:3].T)
                )
            ],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)   # for smooth
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        self.opti.minimize(50 * self.translational_cost + self.rotation_cost + 0.02 * self.regularization_cost + 0.1 * self.smooth_cost)

        opts = {
            'ipopt':{
                'print_level':0,
                'max_iter':50,
                'tol':1e-6
            },
            'print_time':False,# print or not
            'calc_lam_p':False # https://github.com/casadi/casadi/wiki/FAQ:-Why-am-I-getting-%22NaN-detected%22in-my-optimization%3F
        }
        self.opti.solver("ipopt", opts)

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), self.reduced_robot.model.nq)
        
        self.visualizer = G1_29_Visualizer(self.reduced_robot) if Visualization else None

        # Current and init positions of joints and wrists
        self.curr_q = self.init_data
        self.curr_LH_joints = np.zeros(6)
        self.curr_RH_joints = np.zeros(6)

        self.L_tf_init = pin.SE3(
            pin.Quaternion(1, 0, 0, 0),
            np.array([0.25, +0.25, 0.1]),
        ).homogeneous
        self.R_tf_init = pin.SE3(
            pin.Quaternion(1, 0, 0, 0),
            np.array([0.25, -0.25, 0.1]),
        ).homogeneous

        self.L_tf_target = self.L_tf_init.copy()
        self.R_tf_target = self.R_tf_init.copy()


    # If the robot arm is not the same size as your arm :)
    def scale_arms(self, human_left_pose, human_right_pose, human_arm_length=0.60, robot_arm_length=0.75):
        scale_factor = robot_arm_length / human_arm_length
        robot_left_pose = human_left_pose.copy()
        robot_right_pose = human_right_pose.copy()
        robot_left_pose[:3, 3] *= scale_factor
        robot_right_pose[:3, 3] *= scale_factor
        return robot_left_pose, robot_right_pose

    def solve_ik(self, left_wrist, right_wrist, current_lr_arm_motor_q = None, current_lr_arm_motor_dq = None):
        if current_lr_arm_motor_q is not None:
            self.init_data = current_lr_arm_motor_q
        self.opti.set_initial(self.var_q, self.init_data)

        # left_wrist, right_wrist = self.scale_arms(left_wrist, right_wrist)

        self.opti.set_value(self.param_tf_l, left_wrist)
        self.opti.set_value(self.param_tf_r, right_wrist)
        self.opti.set_value(self.var_q_last, self.init_data) # for smooth

        try:
            sol = self.opti.solve()
            # sol = self.opti.solve_limited()

            sol_q = self.opti.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            return sol_q, sol_tauff
        
        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")

            sol_q = self.opti.debug.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            print(f"sol_q:{sol_q} \nmotorstate: \n{current_lr_arm_motor_q} \nleft_pose: \n{left_wrist} \nright_pose: \n{right_wrist}")

            # return sol_q, sol_tauff
            return current_lr_arm_motor_q, np.zeros(self.reduced_robot.model.nv)
    

    # Hand Joint Order
    # 0. thumb 1 (lateral movement)
    # 1. thumb 2 (open / close)
    # 2. index
    # 3. middle
    # 4. ring
    # 5. little
    def move_arms_and_hands(self, L_tf_target, R_tf_target, left_hand_q, right_hand_q):
        sol_q, _ = self.solve_ik(L_tf_target, R_tf_target)

        fingers = ["thumb", "thumb", "index", "middle", "ring", "little"]
        sides = ['left', 'right']
        inputs = [left_hand_q, right_hand_q]

        for side, input in zip(sides, inputs):
            if input is None:
                continue
            assert(len(input) == 6)

            for i, finger in enumerate(fingers):
                if i == 0:  # set thumb 1 joints
                    joint_name = f"{side}_thumb_1_joint"
                    sol_q[self.joint_index_map[joint_name]] = input[i]

                if i == 1:  # set thumb 2 joints
                    joint_name = f"{side}_thumb_{{}}_joint"
                    sol_q[self.joint_index_map[joint_name.format(2)]] = input[i]
                    sol_q[self.joint_index_map[joint_name.format(3)]] = input[i] * 0.4
                    sol_q[self.joint_index_map[joint_name.format(4)]] = input[i] * 0.4
                    
                else:       # set remaining finger joints
                    joint_name = f"{side}_{finger}_{{}}_joint"
                    sol_q[self.joint_index_map[joint_name.format(1)]] = input[i]
                    sol_q[self.joint_index_map[joint_name.format(2)]] = input[i] * 1.05

        if self.visualizer is not None:
            self.visualizer.update(sol_q, L_tf_target, R_tf_target)

