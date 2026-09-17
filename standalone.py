#!/home/yu/isaacsim-6.1.0/python.sh

import argparse

from isaacsim import SimulationApp


OFFICE_ASSET_PATH = "/Isaac/Environments/Office/office.usd"
SURROUNDING_BUILDINGS_PRIM_PATH = "/Root/SM_Buildings"
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

import omni
from isaacsim.core.experimental.utils.stage import is_stage_loading
from isaacsim.storage.native import get_assets_root_path, is_file
from pxr import UsdGeom


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

    print(f"Loaded Office environment: {office_usd_path}")
    print(f"Hidden surrounding buildings: {SURROUNDING_BUILDINGS_PRIM_PATH}")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    if args.test:
        for _ in range(10):
            simulation_app.update()
    else:
        while simulation_app.is_running():
            simulation_app.update()
finally:
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    simulation_app.close()