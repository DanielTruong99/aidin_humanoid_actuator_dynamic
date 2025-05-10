import os
import isaaclab.sim as sim_utils
from isaaclab.actuators import ActuatorNetMLPCfg, DCMotorCfg, ImplicitActuatorCfg, IdealPDActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaac_lab_for_ik.assets import LOCAL_ASSETS_DATA_DIR

LEGPARKOUR_USD_PATH = f"{LOCAL_ASSETS_DATA_DIR}/Robots/Aidin/leg00/leg00.usd"

LEGPARKOUR_IK_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=LEGPARKOUR_USD_PATH,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            # kinematic_enabled=True,
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=10.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=4,
            fix_root_link=True,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.0),
        joint_pos={
            'R_hip_joint': 0.0,
            'R_hip2_joint': 0.0,
            'R_thigh_joint': 0.0,
            'R_calf_joint': 0.0,  # 0.6
            'R_toe_joint': 0.0,
            'L_hip_joint': 0.0,
            'L_hip2_joint': 0.0,
            'L_thigh_joint': 0.0,
            'L_calf_joint': 0.0,  # 0.6
            'L_toe_joint': 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": IdealPDActuatorCfg(
            joint_names_expr=[".*_hip_joint", ".*_hip2_joint", ".*_thigh_joint", ".*_calf_joint"],
            effort_limit=300.0,
            velocity_limit=100.0,
            stiffness={
                "L_hip_joint": 300.0,
                "L_hip2_joint": 300.0,
                "L_thigh_joint": 300.0,
                "L_calf_joint": 300.0,
            },
            damping={
                ".*_hip_joint": 10.0,
                ".*_hip2_joint": 10.0,
                ".*_thigh_joint": 10.0,
                ".*_calf_joint": 10.0,
            },
        ),
        "feet": IdealPDActuatorCfg(
            joint_names_expr=[".*_toe_joint"],
            effort_limit=50.0,
            velocity_limit=10.0,
            stiffness={"L_toe_joint": 100.0},
            damping={"L_toe_joint": 10.0},
        ),
    },
)

LEGPARKOUR_IK_VISUAL_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=LEGPARKOUR_USD_PATH,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            kinematic_enabled=True,
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=10.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=4,
            fix_root_link=True,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.0),
        joint_pos={
            'R_hip_joint': 0.0,
            'R_hip2_joint': 0.0,
            'R_thigh_joint': 0.0,
            'R_calf_joint': 0.0,  # 0.6
            'R_toe_joint': 0.0,
            'L_hip_joint': 0.0,
            'L_hip2_joint': 0.0,
            'L_thigh_joint': 0.0,
            'L_calf_joint': 0.0,  # 0.6
            'L_toe_joint': 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": IdealPDActuatorCfg(
            joint_names_expr=[".*_hip_joint", ".*_hip2_joint", ".*_thigh_joint", ".*_calf_joint"],
            effort_limit=300.0,
            velocity_limit=100.0,
            stiffness={
                "L_hip_joint": 200.0,
                "L_hip2_joint": 200.0,
                "L_thigh_joint": 200.0,
                "L_calf_joint": 150.0,
            },
            damping={
                ".*_hip_joint": 5.0,
                ".*_hip2_joint": 5.0,
                ".*_thigh_joint": 5.0,
                ".*_calf_joint": 5.0,
            },
        ),
        "feet": IdealPDActuatorCfg(
            joint_names_expr=[".*_toe_joint"],
            effort_limit=50.0,
            velocity_limit=10.0,
            stiffness={"L_toe_joint": 100.0},
            damping={"L_toe_joint": 5.0},
        ),
    },
)

