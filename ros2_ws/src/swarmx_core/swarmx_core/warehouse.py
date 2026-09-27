"""Warehouse model shared by every layer of SwarmX.

The same layout drives the lightweight simulator, the Gazebo world (SDF), the
Nav2 occupancy map (PGM/YAML) and the fleet graph used by the planners, so all
of them agree on where shelves, aisles and stations are.

Grid convention: cell (x, y) covers [x, x+1) x [y, y+1) metres, y points "north".
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

Cell = Tuple[int, int]

FREE, SHELF, WALL = 0, 1, 2


@dataclass(frozen=True)
class Zone:
    """A single-lane choke point (narrow aisle segment).

    Robots coordinate access to zones with the decentralized zone-lock protocol
    in :mod:`swarmx_core.zones`. ``ends`` maps an end label to the cell just
    *outside* the zone at that end.
    """

    zid: str
    cells: Tuple[Cell, ...]
    ends: Dict[str, Cell] = field(hash=False, compare=False)

    def end_of(self, outside_cell: Cell) -> Optional[str]:
        for label, c in self.ends.items():
            if c == outside_cell:
                return label
        return None


class Warehouse:
    """Procedurally generated rack warehouse.

    Default layout (34 x 20 m)::

        top cross aisle      rows 16-18 (3 m wide)
        racks + aisles       rows 11-15
        middle cross aisle   rows 9-10  (2 m wide)
        racks + aisles       rows 4-8
        bottom cross aisle   rows 1-3   (3 m wide)

    Left open area (x 1-4) holds the drop-off stations and robot depot, the
    right open area (x 29-32) holds the charging docks. Narrow aisles are 1 m
    wide (single lane for a 0.6 m robot) and form the choke points.
    """

    def __init__(self, width: Optional[int] = None, height: int = 20, n_aisles: int = 8):
        # default layout: 8 aisles -> 34 x 20 m; any aisle count scales the floor (3 m per aisle)
        width = width if width is not None else 3 * n_aisles + 10
        self.width = width
        self.height = height
        self.cell_size = 1.0
        self.grid: List[List[int]] = [[FREE] * width for _ in range(height)]
        self.rack_rows = [(4, 8), (11, 15)]
        self.aisle_xs: List[int] = []
        self._build(n_aisles)
        self.zones: Dict[str, Zone] = {}
        self.cell_zone: Dict[Cell, str] = {}
        self._build_zones()
        self.pickups: List[Cell] = [c for z in self.zones.values() for c in z.cells]
        self.dropoffs: List[Cell] = [(1, 4), (1, 8), (1, 11), (1, 15)]
        w = self.width
        self.chargers: List[Cell] = [(w - 2, 3), (w - 2, 7), (w - 2, 12), (w - 2, 16)]
        # parking bays sit between the cross-aisle traffic lanes, clear of drop-off approaches
        self.depot: List[Cell] = [(x, y) for y in (6, 13, 5, 14, 7, 12) for x in (3, 4)] + \
                                 [(x, y) for y in (6, 13, 5, 14) for x in (w - 4, w - 3)]

    # ------------------------------------------------------------------ build
    def _build(self, n_aisles: int) -> None:
        w, h = self.width, self.height
        for x in range(w):
            self.grid[0][x] = WALL
            self.grid[h - 1][x] = WALL
        for y in range(h):
            self.grid[y][0] = WALL
            self.grid[y][w - 1] = WALL
        # racks: x=5 single rack, then [aisle, shelf, shelf] * n, closing single rack
        x = 5
        rack_cols = [x]
        for _ in range(n_aisles):
            self.aisle_xs.append(x + 1)
            rack_cols += [x + 2, x + 3]
            x += 3
        rack_cols = rack_cols[:-1]  # last pair becomes a single closing rack
        for (y0, y1) in self.rack_rows:
            for y in range(y0, y1 + 1):
                for cx in rack_cols:
                    self.grid[y][cx] = SHELF

    def _build_zones(self) -> None:
        for ax in self.aisle_xs:
            for (y0, y1), tag in zip(self.rack_rows, ("S", "N")):
                zid = f"A{ax}{tag}"
                cells = tuple((ax, y) for y in range(y0, y1 + 1))
                zone = Zone(zid, cells, {"S": (ax, y0 - 1), "N": (ax, y1 + 1)})
                self.zones[zid] = zone
                for c in cells:
                    self.cell_zone[c] = zid

    # ---------------------------------------------------------------- queries
    def in_bounds(self, c: Cell) -> bool:
        return 0 <= c[0] < self.width and 0 <= c[1] < self.height

    def is_free(self, c: Cell) -> bool:
        return self.in_bounds(c) and self.grid[c[1]][c[0]] == FREE

    def zone_of(self, c: Cell) -> Optional[str]:
        return self.cell_zone.get(c)

    def cell_of(self, x: float, y: float) -> Cell:
        return int(x // self.cell_size), int(y // self.cell_size)

    @staticmethod
    def center(c: Cell) -> Tuple[float, float]:
        return c[0] + 0.5, c[1] + 0.5

    def obstacle_cells(self) -> Iterable[Cell]:
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y][x] != FREE:
                    yield (x, y)

    def nearby_wall_points(self, px: float, py: float, radius: float,
                           extra_blocked: Iterable[Cell] = ()) -> List[Tuple[float, float]]:
        """Closest points on every obstacle cell within ``radius`` of (px, py)."""
        pts = []
        cx, cy = int(px), int(py)
        r = int(radius) + 1
        extra = set(extra_blocked)
        for y in range(cy - r, cy + r + 1):
            for x in range(cx - r, cx + r + 1):
                if self.in_bounds((x, y)) and self.grid[y][x] == FREE and (x, y) not in extra:
                    continue
                qx = min(max(px, x), x + 1.0)
                qy = min(max(py, y), y + 1.0)
                if (qx - px) ** 2 + (qy - py) ** 2 <= radius * radius:
                    pts.append((qx, qy))
        return pts

    def lanes(self) -> Dict[Cell, int]:
        """One-way traffic lanes for the cross aisles (+1 eastbound, -1 westbound).

        Used by the traditional block-control baseline, which - like real AGV
        installations - needs directional lanes to avoid head-on gridlock.
        """
        x0, x1 = min(self.aisle_xs) - 1, max(self.aisle_xs) + 1
        rows = {1: +1, 2: +1, 3: -1, 9: +1, 10: -1, 16: +1, 17: -1, 18: -1}
        return {(x, y): d for y, d in rows.items() for x in range(x0, x1 + 1) if self.is_free((x, y))}

    def to_dict(self) -> dict:
        return {
            "width": self.width,
            "height": self.height,
            "grid": self.grid,
            "zones": {z.zid: [list(c) for c in z.cells] for z in self.zones.values()},
            "dropoffs": self.dropoffs,
            "chargers": self.chargers,
        }

    # ---------------------------------------------------------------- exports
    def to_pgm(self, path_prefix: str, resolution: float = 0.05) -> Tuple[str, str]:
        """Write a Nav2 map (``.pgm`` + ``.yaml``) at ``resolution`` m/pixel."""
        scale = int(round(self.cell_size / resolution))
        wpx, hpx = self.width * scale, self.height * scale
        rows = []
        for py in range(hpx - 1, -1, -1):  # PGM row 0 is the top of the map
            cy = py // scale
            row = bytearray()
            for px in range(wpx):
                row.append(0 if self.grid[cy][px // scale] != FREE else 254)
            rows.append(bytes(row))
        pgm = f"{path_prefix}.pgm"
        with open(pgm, "wb") as f:
            f.write(f"P5\n{wpx} {hpx}\n255\n".encode())
            for r in rows:
                f.write(r)
        yml = f"{path_prefix}.yaml"
        import os
        with open(yml, "w") as f:
            f.write(f"image: {os.path.basename(pgm)}\nmode: trinary\nresolution: {resolution}\n"
                    "origin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
        return pgm, yml

    def to_sdf(self, world_name: str = "swarmx_warehouse") -> str:
        """Gazebo Harmonic world with merged shelf/wall boxes."""
        boxes = self._merged_boxes()
        models = []
        for i, (x0, y0, x1, y1, kind) in enumerate(boxes):
            sx, sy = x1 - x0, y1 - y0
            h = 2.0 if kind == SHELF else 1.0
            color = "0.85 0.55 0.2 1" if kind == SHELF else "0.5 0.5 0.55 1"
            models.append(f"""
    <model name="{'shelf' if kind == SHELF else 'wall'}_{i}">
      <static>true</static>
      <pose>{x0 + sx / 2:.3f} {y0 + sy / 2:.3f} {h / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx} {sy} {h}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      </link>
    </model>""")
        markers = []
        for name, cells, color in (("dropoff", self.dropoffs, "0.1 0.8 0.2 1"),
                                   ("charger", self.chargers, "0.2 0.4 1.0 1")):
            for i, (cx, cy) in enumerate(cells):
                markers.append(f"""
    <model name="{name}_{i}">
      <static>true</static>
      <pose>{cx + 0.5} {cy + 0.5} 0.005 0 0 0</pose>
      <link name="link"><visual name="v"><geometry><box><size>0.9 0.9 0.01</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link>
    </model>""")
        return f"""<?xml version="1.0"?>
<!-- Generated by swarmx_core.warehouse -- do not edit by hand. -->
<sdf version="1.9">
  <world name="{world_name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual>
      </link>
    </model>{''.join(models)}{''.join(markers)}
  </world>
</sdf>
"""

    def _merged_boxes(self) -> List[Tuple[int, int, int, int, int]]:
        """Greedy rectangle merge of obstacle cells to keep the SDF small."""
        used = [[False] * self.width for _ in range(self.height)]
        boxes = []
        for y in range(self.height):
            for x in range(self.width):
                kind = self.grid[y][x]
                if kind == FREE or used[y][x]:
                    continue
                x1 = x
                while x1 + 1 < self.width and self.grid[y][x1 + 1] == kind and not used[y][x1 + 1]:
                    x1 += 1
                y1 = y
                while y1 + 1 < self.height and all(
                        self.grid[y1 + 1][i] == kind and not used[y1 + 1][i] for i in range(x, x1 + 1)):
                    y1 += 1
                for yy in range(y, y1 + 1):
                    for xx in range(x, x1 + 1):
                        used[yy][xx] = True
                boxes.append((x, y, x1 + 1, y1 + 1, kind))
        return boxes

    def ascii(self, marks: Optional[Dict[Cell, str]] = None) -> str:
        marks = marks or {}
        sym = {FREE: ".", SHELF: "#", WALL: "X"}
        lines = []
        for y in range(self.height - 1, -1, -1):
            lines.append("".join(marks.get((x, y), sym[self.grid[y][x]]) for x in range(self.width)))
        return "\n".join(lines)
