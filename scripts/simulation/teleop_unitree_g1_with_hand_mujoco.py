import os
from typing import Any, Dict

import numpy as np
import tyro
import meshcat.transformations as tf
import mujoco

from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


class G1DexHandMujocoController(MujocoTeleopController):
    """
    MuJoCo teleoperation controller for Unitree G1 with dexterous hands.
    """

    def _robot_setup(self):
        """
        Extended setup to initialize mocap trackers at the elbow positions.
        """
        super()._robot_setup()
        
        # Explicitly center the motion tracker targets on the elbows for the initial L-shape.
        for name, config in self.manipulator_config.items():
            if "motion_tracker" in config:
                tracker_config = config["motion_tracker"]
                link_target = tracker_config["link_target"]
                vis_target = tracker_config["vis_target"]
                
                # Get the elbow id
                elbow_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, link_target)
                if elbow_id != -1:
                    elbow_pos = self.mj_data.xpos[elbow_id].copy()
                    
                    # Set the mocap body position
                    mocap_body_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, vis_target)
                    mocap_id = self.mj_model.body_mocapid[mocap_body_id]
                    
                    if mocap_id != -1:
                        self.mj_data.mocap_pos[mocap_id] = elbow_pos
                        print(f"Centered {vis_target} on {link_target} at {elbow_pos}")

    def _update_ik(self):
        """
        Prevent target drift when control is not active.
        """
        super()._update_ik()
        
        for name, config in self.manipulator_config.items():
            trigger_val = self.xr_client.get_key_value_by_name(config["control_trigger"])
            if trigger_val < 0.5:
                # Sync placo target to current MuJoCo pose once when deactivating
                ee_xyz, ee_quat_wxyz = self._get_link_pose(config["link_name"])
                T_world = tf.quaternion_matrix(ee_quat_wxyz)
                T_world[:3, 3] = ee_xyz
                self.effector_task[name].T_world_frame = T_world


def main(
    xml_path: str = os.path.join(ASSET_PATH, "unitree/g1/g1_dual_arm_with_hand.xml"),
    robot_urdf_path: str = os.path.join(ASSET_PATH, "unitree/g1/g1_dual_arm_with_hand.urdf"),
    scale_factor: float = 1.0,
    visualize_placo: bool = False,
):
    """
    Main function for G1 MuJoCo teleop with desk-height robot.
    """
    config = {
        "left_arm": {
            "link_name": "left_hand_palm_link",
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "vis_target": "left_target",
            "motion_tracker": {
                "serial": "PC2310MLKB041941G",
                "link_target": "left_elbow_link",
                "vis_target": "left_tracker_target",
            },
        },
        "right_arm": {
            "link_name": "right_hand_palm_link",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "vis_target": "right_target",
            "motion_tracker": {
                "serial": "PC2310MLKB041978G",
                "link_target": "right_elbow_link",
                "vis_target": "right_tracker_target",
            },
        },
    }

    # Initial L-shape (Desk Height Z=0.4)
    # Pitch(0), Roll(1), Yaw(2), Elbow(3)
    safe_qpos = np.zeros(35)
    # Left Arm
    safe_qpos[0] = 0.3   # pitch
    safe_qpos[1] = 0.2   # roll
    safe_qpos[3] = 1.0   # elbow
    # Right Arm
    safe_qpos[14] = 0.3  # pitch
    safe_qpos[15] = -0.2 # roll
    safe_qpos[17] = 1.0  # elbow
    # Cube: [x,y,z, qw,qx,qy,qz]
    safe_qpos[28:35] = [0.5, 0.0, 0.445, 1.0, 0.0, 0.0, 0.0]

    controller = G1DexHandMujocoController(
        xml_path=xml_path,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        mj_qpos_init=safe_qpos,
        floating_base=False,
        dt=0.002,
    )

    controller.sync_end_effector_poses_to_placo_tasks()

    joints_task = controller.solver.add_joints_task()
    default_joints = {
        "left_shoulder_pitch_joint": 0.3, "left_shoulder_roll_joint": 0.2, "left_elbow_joint": 1.0,
        "right_shoulder_pitch_joint": 0.3, "right_shoulder_roll_joint": -0.2, "right_elbow_joint": 1.0,
    }
    joints_task.set_joints(default_joints)
    joints_task.configure("joints_regularization", "soft", 1e-4)

    print("Starting Unitree G1 MuJoCo Teleoperation...")
    print("Hands are correctly attached. Trackers are centered on elbows.")
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
