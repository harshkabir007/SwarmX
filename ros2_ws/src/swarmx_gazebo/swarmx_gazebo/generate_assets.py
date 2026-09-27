"""Regenerate the Gazebo world and the Nav2 map from the shared warehouse model.

    ros2 run swarmx_gazebo generate_assets --world-out <dir> --map-out <dir>

Both files come from ``swarmx_core.warehouse.Warehouse`` - the same model the
fleet agents plan on - so the simulator, the localization map and the
coordination graph can never drift apart.
"""
import argparse
import os
import sys

from swarmx_core.warehouse import Warehouse


def main(argv=None) -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    src_root = os.path.abspath(os.path.join(here, "..", ".."))
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--world-out", default=os.path.join(src_root, "swarmx_gazebo", "worlds"))
    ap.add_argument("--map-out", default=os.path.join(src_root, "swarmx_navigation", "maps"))
    ap.add_argument("--resolution", type=float, default=0.05)
    args = ap.parse_args(argv)
    wh = Warehouse()
    os.makedirs(args.world_out, exist_ok=True)
    os.makedirs(args.map_out, exist_ok=True)
    world = os.path.join(args.world_out, "swarmx_warehouse.sdf")
    with open(world, "w") as f:
        f.write(wh.to_sdf("swarmx_warehouse"))
    pgm, yml = wh.to_pgm(os.path.join(args.map_out, "warehouse"), args.resolution)
    print(f"wrote {world}\nwrote {pgm}\nwrote {yml}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
