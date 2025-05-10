import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.prims import SingleXFormPrim
# from isaacsim.core.prims.articulation import Articulation
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.dynamic_control import _dynamic_control
from isaacsim.sensors.physics import IMUSensor
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, euler_angles_to_quats
from isaacsim.core.prims import Articulation

my_world = World(stage_units_in_meters=1.0, physics_dt = 0.005, rendering_dt = 0.002)
scene : Scene = my_world.scene
scene.add_default_ground_plane(z_position=-1.0)

simulation_app.update()


# Setup buffers
joint_cmd = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
kps = np.array([1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000])
kds = np.array([5, 5, 5, 5, 5, 5, 5, 5, 5, 5])


# Spawn Humanoid, set initial pose, initialize controller
humanoid_asset_path = "usd/leg00/leg00.usd"
add_reference_to_stage(usd_path=humanoid_asset_path, prim_path="/World/Humanoid")
humanoid: Robot = my_world.scene.add(Robot(prim_path="/World/Humanoid", name="my_humanoid"))
humanoid.set_world_pose(position=np.array([0.0, 0.0, 0.91]) / get_stage_units()
                        , orientation=euler_angles_to_quats(np.array([0, 0, 0])))

humanoid_articulation = Articulation(prim_paths_expr="/World/Humanoid/base")
# humanoid_articulation.set_joint_positions(np.array([0.0, 0.0, 0.0, 0.0, -0.2, -0.2, 0.25, 0.25, 0.0, 0.0]))
# humanoid_articulation.set_friction_coefficients(np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]))

my_world.reset()
humanoid.get_articulation_controller().set_gains(kps, kds)

joint_init = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
joint_kneel = np.array([0, 0, 0, 0, -0.5735, -0.5735, 1.147, 1.147, -0.5735, -0.5735])


reset_needed = False
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False

        
        # Apply joint commands
        # humanoid.get_articulation_controller().apply_action(
        #     ArticulationAction(joint_positions=joint_cmd)
        # )
        

simulation_app.close()
