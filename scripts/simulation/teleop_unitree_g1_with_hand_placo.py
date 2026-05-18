import os
from typing import Any, Dict

import tyro

from xrobotoolkit_teleop.simulation.placo_teleop_controller import PlacoTeleopController
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


class G1DexHandPlacoController(PlacoTeleopController):
    """
    Placo-only controller for Unitree G1 with dexterous hands.
    This exactly matches the behavior of teleop_unitree_g1_placo.py,
    but adds logic to animate the finger joints based on trigger input.
    """

    def _send_command(self):
        # Update finger joints based on XR trigger before drawing in Meshcat
        for gripper_name, config in self.manipulator_config.items():
            if "gripper_config" not in config:
                continue
            
            g_config = config["gripper_config"]
            if g_config["type"] == "dexterous":
                trigger_val = self.xr_client.get_key_value_by_name(g_config["gripper_trigger"])
                
                # Apply interpolation
                for j_name, open_p, close_p in zip(
                    g_config["joint_names"], 
                    g_config["open_pos"], 
                    g_config["close_pos"]
                ):
                    target_pos = open_p + (close_p - open_p) * trigger_val
                    self.placo_robot.set_joint(j_name, target_pos)

        # Draw the updated state in Meshcat
        super()._send_command()


def main(
    robot_urdf_path: str = os.path.join(ASSET_PATH, "unitree/g1/g1_dual_arm_with_hand.urdf"),
    scale_factor: float = 1,
):
    """
    Main function to run the Unitree G1 dexterous hand teleoperation with Placo visualization.
    """
    left_hand_joints = [
        "left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
        "left_hand_middle_0_joint", "left_hand_middle_1_joint",
        "left_hand_index_0_joint", "left_hand_index_1_joint"
    ]
    left_open = [0.0, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
    left_close = [0.5, 0.8, 1.2, -1.5, -1.5, -1.5, -1.5]

    right_hand_joints = [
        "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
        "right_hand_middle_0_joint", "right_hand_middle_1_joint",
        "right_hand_index_0_joint", "right_hand_index_1_joint"
    ]
    right_open = [0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
    right_close = [-0.5, -0.8, -1.2, 1.5, 1.5, 1.5, 1.5]

    # Define dual arm configuration matching teleop_unitree_g1_placo.py perfectly
    config = {
        "left_arm": {
            "link_name": "left_hand_palm_link",
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "motion_tracker": {
                "serial": "PC2310MLKB041941G",
                "link_target": "left_elbow_link",
            },
            "gripper_config": {
                "type": "dexterous",
                "gripper_trigger": "left_trigger",
                "joint_names": left_hand_joints,
                "open_pos": left_open,
                "close_pos": left_close,
            },
        },
        "right_arm": {
            "link_name": "right_hand_palm_link",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            # "motion_tracker": {
            #     "serial": "PC2310MLKB041978G",
            #     "link_target": "right_elbow_link",
            # },
            "gripper_config": {
                "type": "dexterous",
                "gripper_trigger": "right_trigger",
                "joint_names": right_hand_joints,
                "open_pos": right_open,
                "close_pos": right_close,
            },
        },
    }

    # Create and initialize the teleoperation controller
    controller = G1DexHandPlacoController(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
        floating_base=False,
    )

    # Add joint regularization task to keep arms in natural position
    joints_task = controller.solver.add_joints_task()

    # Define exact default joint positions from teleop_unitree_g1_placo.py
    default_joints = {
        "left_shoulder_pitch_joint": 0.3,
        "left_shoulder_roll_joint": 0.2,
        "left_shoulder_yaw_joint": 0.0,
        "left_elbow_joint": 1.0,
        "left_wrist_roll_joint": 0.0,
        "left_wrist_pitch_joint": 0.0,
        "left_wrist_yaw_joint": 0.0,
        "right_shoulder_pitch_joint": 0.3,
        "right_shoulder_roll_joint": -0.2,
        "right_shoulder_yaw_joint": 0.0,
        "right_elbow_joint": 1.0,
        "right_wrist_roll_joint": 0.0,
        "right_wrist_pitch_joint": 0.0,
        "right_wrist_yaw_joint": 0.0,
    }

    joints_task.set_joints(default_joints)
    joints_task.configure("joints_regularization", "soft", 1e-4)

    print("Starting Unitree G1 Placo-only Dexterous Hand Teleoperation...")
    print("Control mapping:")
    print("  - Left/Right Grip: Activate arm control")
    print("  - Left/Right Trigger: Close fingers")

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
