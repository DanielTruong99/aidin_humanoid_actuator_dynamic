# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates how to use the differential inverse kinematics controller with the simulator.

The differential IK controller can be configured in different modes. It uses the Jacobians computed by
PhysX. This helps perform parallelized computation of the inverse kinematics.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p scripts/tutorials/05_controllers/run_diff_ik.py

"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on using the differential IK controller.")
parser.add_argument("--robot", type=str, default="ur10", help="Name of the robot.")
parser.add_argument("--num_envs", type=int, default=128, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.markers import VisualizationMarkers
from isaaclab.markers.config import FRAME_MARKER_CFG, CUBOID_MARKER_CFG
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.math import subtract_frame_transforms, quat_rotate


from isaac_lab_for_ik.controllers.custom_differential_ik import CustomDifferentialIKController
##
# Pre-defined configs
##
from isaaclab_assets import FRANKA_PANDA_HIGH_PD_CFG, UR10_CFG  # isort:skip
from isaac_lab_for_ik.assets import LEGPARKOUR_IK_CFG, LEGPARKOUR_IK_VISUAL_CFG # isort:skip

import rclpy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from isaacsim.core.utils.extensions import enable_extension
import pandas as pd
import os


enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
rclpy.init()
node = rclpy.create_node("diff_ik_node")
# create ros2 joint state publisher
joint_state_publisher = node.create_publisher(
    JointState, "/joint_states", 15
)
path_publisher = node.create_publisher(
    Path, "/path", 15
)


@configclass
class TableTopSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # articulation
    robot = LEGPARKOUR_IK_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # robot visualization
    # robot_visual = LEGPARKOUR_IK_VISUAL_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot/Visuals")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability.
    robot = scene["robot"]

    # Create controller
    diff_ik_cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls", ik_params={"lambda_val": 0.01/2})
    diff_ik_controller = CustomDifferentialIKController(diff_ik_cfg, num_envs=scene.num_envs, device=sim.device)

    # Markers
    point_marker_cfg = CUBOID_MARKER_CFG.copy(); point_marker_cfg.markers["cuboid"].size = (0.003, 0.003, 0.003)
    point_marker_cfg.markers["cuboid"].visual_material.diffuse_color = (0.0, 0.0, 1.0)
    frame_marker_cfg = FRAME_MARKER_CFG.copy()
    frame_marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
    ee_marker = VisualizationMarkers(frame_marker_cfg.replace(prim_path="/Visuals/ee_current_pose"))
    point_marker_cfg.markers["cuboid"].visual_material.diffuse_color = (1.0, 0.0, 0.0)
    ee_pos_tracing_marker = VisualizationMarkers(point_marker_cfg.replace(prim_path="/Visuals/ee_current_pos_tracing"))
    point_marker_cfg.markers["cuboid"].visual_material.diffuse_color = (0.0, 0.0, 1.0)
    goal_marker = VisualizationMarkers(point_marker_cfg.replace(prim_path="/Visuals/ee_goal"))
    

    # Define goals for the arm
    #! Eclipse trajectory
    # a = 0.11; b = 0.1;
    # w = 0.7; n = 300;
    # t = torch.linspace(0, 10, n, device=sim.device)
    # xo = 0.15; zo = 0.39; 
    # x = a * torch.cos(w * t) + xo;
    # z = b * torch.sin(w * t) + zo;
    # y = 0.1175;
    # pos = torch.cat([x.unsqueeze(1), torch.ones(n, 1, device=sim.device) * y, z.unsqueeze(1)], dim=1)
    # orient = torch.zeros(n, 4, device=sim.device); orient[:, 0] = 1.0
    # ee_goals = torch.cat([pos, orient], dim=1)
    # ee_goals = torch.tensor(ee_goals, device=sim.device)

    #! Eclipsoid trajectory
    a = 0.11; b = 0.1; c = 0.08;
    n = 2000;
    t = torch.linspace(0, 10, n);
    A_phi = 12; w_phi = 1.7;
    A_theta = 3.4; w_theta = 0.265;
    xo = 0.11; zo = 0.45; yo = 0.1175;
    phi_0 = -A_phi/w_phi; theta_0 = -A_phi/w_phi;
    phi = -A_phi/w_phi * torch.cos(w_phi * t) + A_phi/w_phi + phi_0;
    theta = -A_theta/w_theta * torch.cos(w_theta * t) + A_theta/w_theta + theta_0;
    x = a * torch.cos(theta) * torch.cos(phi) + xo;
    y = b * torch.cos(theta) * torch.sin(phi) + yo;
    z = c * torch.sin(theta) + zo;
    pos = torch.cat([x.unsqueeze(1), y.unsqueeze(1), z.unsqueeze(1)], dim=1)
    orient = torch.zeros(n, 4, device=sim.device); orient[:, 0] = 1.0
    ee_goals = torch.cat([pos, orient], dim=1)
    ee_goals = torch.tensor(ee_goals, device=sim.device)


    # Track the given command
    current_goal_idx = 0

    # Create buffers to store actions
    ik_commands = torch.zeros(scene.num_envs, diff_ik_controller.action_dim, device=robot.device)
    ik_commands[:] = ee_goals[current_goal_idx]

    # Specify robot-specific parameters
    robot_entity_cfg = SceneEntityCfg(
        "robot",
        joint_names=["L_hip_joint", "L_hip2_joint", "L_thigh_joint", "L_calf_joint", "L_toe_joint"],
        body_names=["L_toe"],
    )
    right_leg_entity_cfg = SceneEntityCfg(
        "robot",
        joint_names=["R_hip_joint", "R_hip2_joint", "R_thigh_joint", "R_calf_joint", "R_toe_joint"],
        body_names=["R_toe"],
    )

    # Resolving the scene entities
    robot_entity_cfg.resolve(scene)
    right_leg_entity_cfg.resolve(scene)

    # Obtain the frame index of the end-effector
    # For a fixed base robot, the frame index is one less than the body index. This is because
    # the root body is not included in the returned Jacobians.
    if robot.is_fixed_base:
        ee_jacobi_idx = robot_entity_cfg.body_ids[0] - 1
    else:
        ee_jacobi_idx = robot_entity_cfg.body_ids[0]

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count_planner = 0; count_marker = 0; count_ros2_publish = 0
    ee_tracer_vis = torch.zeros(50, 3, device=sim.device)
    ee_tracer_vis = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:3].unsqueeze(1).expand(-1, 50, -1).squeeze()

    # reset
    # sampling time 100hz; planner_max_count = 1/100 / sim_dt
    # sampling time for markers 10hz; marker_max_count = 1/10 / sim_dt
    planner_max_count = (1/100) / sim_dt
    marker_max_count = (1/50) / sim_dt
    ros2_publish_max_count = (1/50) / sim_dt
    position_error = 9999.9
    stage = 0
    data = {}

    # Simulation loop
    while simulation_app.is_running():
        if count_planner % planner_max_count == 0:
            # reset time
            count_planner = 0
            
            # reset actions
            ik_commands[:] = ee_goals[current_goal_idx]; ik_commands[:, 0:3] = ik_commands[:, 0:3] - robot.data.root_state_w[:, 0:3]
            joint_pos = robot.data.joint_pos[:, robot_entity_cfg.joint_ids]
            joint_pos_des = joint_pos.clone()

            # set the command
            diff_ik_controller.reset()
            diff_ik_controller.set_command(ik_commands)

            # change goal
            current_goal_idx = (current_goal_idx + 1) % len(ee_goals)

        # publish joint state
        if count_ros2_publish % ros2_publish_max_count == 0:
            joint_state_msg = JointState()
            joint_state_msg.name = robot_entity_cfg.joint_names
            joint_state_msg.header.stamp = node.get_clock().now().to_msg()
            joint_state_msg.position = robot.data.joint_pos[:, robot_entity_cfg.joint_ids].cpu().numpy().flatten().tolist()
            joint_state_publisher.publish(joint_state_msg)

            #! write the joint state to csv
            if current_goal_idx > 10 and current_goal_idx < len(ee_goals) - 2:
                # Prepare data
                joint_names = robot_entity_cfg.joint_names
                joint_positions = robot.data.joint_pos[:, robot_entity_cfg.joint_ids].cpu().numpy().flatten().tolist()
                timestamp = node.get_clock().now().nanoseconds / 1e9
                
                if "timestamp" not in data:
                    data["timestamp"] = []
                data["timestamp"].append(timestamp)
                for i, name in enumerate(joint_names):
                    if name not in data:
                        data[name] = []
                    data[name].append(joint_positions[i])


            elif current_goal_idx > len(ee_goals) - 2:
                # Save joint states to a pandas DataFrame and export to CSV
                first_time = data["timestamp"][0]
                data["timestamp"] = [time - first_time for time in data["timestamp"]]

                data_frame = pd.DataFrame(data)
                csv_path = "joint_states.csv"
                data_frame.to_csv(csv_path, index=False)
                data = {}

                

            #! publish the path topic
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = node.get_clock().now().to_msg()
            path_msg.poses = []
            for i in range(ee_tracer_vis.shape[0]):
                pose = ee_tracer_vis[i, :]
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = "map"
                pose_msg.header.stamp = node.get_clock().now().to_msg()
                pose_msg.pose.position.x = pose[0].item()
                pose_msg.pose.position.y = pose[1].item()
                pose_msg.pose.position.z = pose[2].item() - 1.0
                path_msg.poses.append(pose_msg)
            path_publisher.publish(path_msg)

        # set the tracing position marker
        if count_marker % marker_max_count == 0:
            ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7];
            ee_pose_w[:, 0:3] = ee_pose_w[:, 0:3] + quat_rotate(ee_pose_w[:, 3:7], torch.tensor([[0.0, 0.013, 0.0]], device=sim.device))
            ee_tracer_vis = torch.roll(ee_tracer_vis, shifts=1, dims=0)
            ee_tracer_vis[0, :] = ee_pose_w[:, 0:3]
        


        # obtain quantities from simulation
        jacobian = robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, robot_entity_cfg.joint_ids]
        ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7];
        ee_pose_w[:, 0:3] = ee_pose_w[:, 0:3] + quat_rotate(ee_pose_w[:, 3:7], torch.tensor([[0.0, 0.013, 0.0]], device=sim.device))
        root_pose_w = robot.data.root_state_w[:, 0:7]
        joint_pos = robot.data.joint_pos[:, robot_entity_cfg.joint_ids]

        # compute frame in root frame
        ee_pos_b, ee_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
        )
        position_error = torch.norm(ee_pos_b, dim=1)

        # compute the joint commands
        joint_pos_des = diff_ik_controller.compute(ee_pos_b, ee_quat_b, jacobian, joint_pos)
            

        # apply actions
        robot.set_joint_position_target(joint_pos_des, joint_ids=robot_entity_cfg.joint_ids)
        scene.write_data_to_sim()

        # perform step
        sim.step()

        # update sim-time
        count_planner += 1
        count_marker += 1
        count_ros2_publish += 1

        # update buffers
        scene.update(sim_dt)

        # update marker positions
        ee_marker.visualize(ee_pose_w[:, 0:3], ee_pose_w[:, 3:7])
        ee_pos_tracing_marker.visualize(ee_tracer_vis[:, 0:3])
        goal_marker.visualize(ee_goals[:, 0:3], ee_goals[:, 3:7])


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(dt=1/200.0, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

    # Design scene
    scene_cfg = TableTopSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    
    # Play the simulator
    sim.reset()

    # Now we are ready!
    # Run the simulator
    print("[INFO]: Setup complete...")
    run_simulator(sim, scene)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
