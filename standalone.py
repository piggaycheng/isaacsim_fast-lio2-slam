#!/home/yu/isaacsim-6.1.0/python.sh

import argparse

from isaacsim import SimulationApp


OFFICE_ASSET_PATH = "/Isaac/Environments/Office/office.usd"
CARTER_ASSET_PATH = "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
SURROUNDING_BUILDINGS_PRIM_PATH = "/Root/SM_Buildings"
CARTER_PRIM_PATH = "/World/Carter"
CARTER_SPAWN_POSITION = [0.0, 0.0, 0.05]
LINEAR_JOG_SPEED = 0.5
ANGULAR_JOG_SPEED = 1.2
KIT_EXTRA_ARGS = [
    "--/rtx/post/dlss/execMode=0",
    "--/app/renderer/skipGpuRenderProducts=false",
    "--/rtx/rendermode=RealTime",
]

parser = argparse.ArgumentParser(description="Launch Isaac Sim with the Office environment.")
parser.add_argument("--headless", action="store_true", help="Run without the Isaac Sim GUI.")
parser.add_argument("--test", action="store_true", help="Load the stage and exit after ten frames.")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp(
    {
        "headless": args.headless,
        "extra_args": KIT_EXTRA_ARGS,
    }
)

import carb
import isaacsim.core.experimental.utils.app as app_utils
import omni
import omni.appwindow
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.experimental.utils.stage import is_stage_loading
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot
from isaacsim.storage.native import get_assets_root_path, is_file
from pxr import UsdGeom


pressed_keys = set()
input_interface = None
keyboard = None
keyboard_subscription = None


def on_keyboard_event(event, *_) -> bool:
    key = event.input.name
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        pressed_keys.add(key)
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        pressed_keys.discard(key)
    return True


def get_jog_command() -> list[float]:
    if "SPACE" in pressed_keys:
        return [0.0, 0.0]

    forward = int(bool(pressed_keys & {"W", "UP"}))
    backward = int(bool(pressed_keys & {"S", "DOWN"}))
    left = int(bool(pressed_keys & {"A", "LEFT"}))
    right = int(bool(pressed_keys & {"D", "RIGHT"}))
    return [
        (forward - backward) * LINEAR_JOG_SPEED,
        (left - right) * ANGULAR_JOG_SPEED,
    ]


try:
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        raise RuntimeError(
            "Could not find the Isaac Sim assets root. Check the asset server or local asset configuration."
        )

    office_usd_path = assets_root_path + OFFICE_ASSET_PATH
    if not is_file(office_usd_path):
        raise FileNotFoundError(f"Office environment asset was not found: {office_usd_path}")

    omni.usd.get_context().open_stage(office_usd_path)

    simulation_app.update()
    simulation_app.update()
    while is_stage_loading():
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    buildings_prim = stage.GetPrimAtPath(SURROUNDING_BUILDINGS_PRIM_PATH)
    if not buildings_prim.IsValid():
        raise RuntimeError(
            f"Surrounding buildings prim was not found: {SURROUNDING_BUILDINGS_PRIM_PATH}"
        )
    UsdGeom.Imageable(buildings_prim).MakeInvisible()

    carter_usd_path = assets_root_path + CARTER_ASSET_PATH
    if not is_file(carter_usd_path):
        raise FileNotFoundError(f"Nova Carter asset was not found: {carter_usd_path}")

    carter = WheeledRobot(
        paths=CARTER_PRIM_PATH,
        wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
        usd_path=carter_usd_path,
        positions=CARTER_SPAWN_POSITION,
    )
    controller = DifferentialController(wheel_radius=0.04295, wheel_base=0.4132)

    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    physics_scenes = SimulationManager.get_physics_scenes()
    if not physics_scenes:
        raise RuntimeError("Isaac Sim did not create a physics scene")
    physics_scenes[0].set_enabled_gpu_dynamics(False)

    if not args.headless:
        app_window = omni.appwindow.get_default_app_window()
        input_interface = carb.input.acquire_input_interface()
        keyboard = app_window.get_keyboard()
        keyboard_subscription = input_interface.subscribe_to_keyboard_events(
            keyboard,
            on_keyboard_event,
        )

    print(f"Loaded Office environment: {office_usd_path}")
    print(f"Hidden surrounding buildings: {SURROUNDING_BUILDINGS_PRIM_PATH}")
    print(f"Added Nova Carter: {CARTER_PRIM_PATH}")
    if not args.headless:
        print("Jog controls: W/S or Up/Down = forward/backward, A/D or Left/Right = turn, Space = stop")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(10):
        simulation_app.update()

    if args.test:
        start_position = carter.get_world_poses()[0].numpy()[0]
        for _ in range(60):
            carter.apply_wheel_actions(controller.forward(command=[0.2, 0.0]))
            simulation_app.update()
        carter.apply_wheel_actions(controller.forward(command=[0.0, 0.0]))
        end_position = carter.get_world_poses()[0].numpy()[0]
        distance_moved = float(((end_position - start_position) ** 2).sum() ** 0.5)
        if distance_moved < 0.01:
            raise RuntimeError(f"Nova Carter jog test failed; moved only {distance_moved:.4f} m")
        print(f"Nova Carter jog test passed: moved {distance_moved:.3f} m")
    else:
        while simulation_app.is_running():
            carter.apply_wheel_actions(controller.forward(command=get_jog_command()))
            simulation_app.update()
finally:
    if input_interface is not None and keyboard_subscription is not None:
        input_interface.unsubscribe_to_keyboard_events(keyboard, keyboard_subscription)
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    simulation_app.close()