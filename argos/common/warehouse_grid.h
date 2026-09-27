/*
 * SwarmX - shared warehouse grid for the ARGoS large-scale experiments.
 *
 * Loaded from the text map written by scripts/generate_experiment.py, which in
 * turn comes from swarmx_core.warehouse.Warehouse - the same model used by the
 * Python simulator, Gazebo and Nav2.
 */
#ifndef SWARMX_WAREHOUSE_GRID_H
#define SWARMX_WAREHOUSE_GRID_H

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace swarmx {

struct Cell {
  int x = 0, y = 0;
  bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
  bool operator!=(const Cell& o) const { return !(*this == o); }
  bool operator<(const Cell& o) const { return x < o.x || (x == o.x && y < o.y); }
};

struct Zone {
  int id = 0;
  int x = 0, y0 = 0, y1 = 0;   // vertical single-lane aisle segment [y0, y1]
  Cell south() const { return Cell{x, y0 - 1}; }
  Cell north() const { return Cell{x, y1 + 1}; }
  bool contains(const Cell& c) const { return c.x == x && c.y >= y0 && c.y <= y1; }
};

class WarehouseGrid {
 public:
  int width = 0, height = 0;
  double cell = 1.0;            // metres per cell
  std::vector<std::string> rows;  // rows[y][x]: '.' free, '#' shelf, 'X' wall
  std::vector<Zone> zones;
  std::vector<Cell> pickups, dropoffs, depot;
  std::vector<int> zone_of;       // per cell, -1 if none
  std::vector<int> lane;          // per cell: +1 eastbound, -1 westbound, 0 free
  double lane_penalty = 2.0;      // extra cost per cell driven against a lane (soft lanes)
  bool hard_lanes = false;        // traditional one-way lanes

  void Load(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open map " + path);
    std::string tag;
    in >> tag >> width >> height >> cell;
    rows.assign(height, std::string());
    for (int y = 0; y < height; ++y) in >> rows[y];
    int n;
    in >> tag >> n;
    zones.resize(n);
    for (int i = 0; i < n; ++i) { zones[i].id = i; in >> zones[i].x >> zones[i].y0 >> zones[i].y1; }
    auto read_cells = [&](std::vector<Cell>& v) {
      in >> tag >> n;
      v.resize(n);
      for (int i = 0; i < n; ++i) in >> v[i].x >> v[i].y;
    };
    read_cells(pickups);
    read_cells(dropoffs);
    read_cells(depot);
    lane.assign(width * height, 0);
    if (in >> tag >> n)
      for (int i = 0; i < n; ++i) { int x, y, d; in >> x >> y >> d; lane[y * width + x] = d; }
    zone_of.assign(width * height, -1);
    for (const Zone& z : zones)
      for (int y = z.y0; y <= z.y1; ++y) zone_of[y * width + z.x] = z.id;
  }

  bool InBounds(const Cell& c) const { return c.x >= 0 && c.y >= 0 && c.x < width && c.y < height; }
  bool Free(const Cell& c) const { return InBounds(c) && rows[c.y][c.x] == '.'; }
  int ZoneOf(const Cell& c) const { return InBounds(c) ? zone_of[c.y * width + c.x] : -1; }
  Cell CellOf(double x, double y) const {
    return Cell{static_cast<int>(std::floor(x / cell)), static_cast<int>(std::floor(y / cell))};
  }
  void Center(const Cell& c, double& x, double& y) const { x = (c.x + 0.5) * cell; y = (c.y + 0.5) * cell; }

  /* Closest points (metres) on obstacle cells within `radius` of (px, py). */
  void WallPoints(double px, double py, double radius, const std::vector<Cell>& extra,
                  std::vector<std::pair<double, double>>& out) const {
    out.clear();
    Cell c = CellOf(px, py);
    int r = static_cast<int>(radius / cell) + 1;
    for (int y = c.y - r; y <= c.y + r; ++y)
      for (int x = c.x - r; x <= c.x + r; ++x) {
        Cell q{x, y};
        bool obst = !Free(q) || std::find(extra.begin(), extra.end(), q) != extra.end();
        if (!obst) continue;
        double qx = std::min(std::max(px, x * cell), (x + 1) * cell);
        double qy = std::min(std::max(py, y * cell), (y + 1) * cell);
        if ((qx - px) * (qx - px) + (qy - py) * (qy - py) <= radius * radius) out.emplace_back(qx, qy);
      }
  }

  /* 8-connected A* with corner-cutting protection and per-zone entry penalties (metres). */
  bool AStar(const Cell& s, const Cell& g, std::vector<Cell>& path, const std::vector<double>& zone_penalty,
             const std::vector<Cell>& blocked) const {
    path.clear();
    if (!Free(g)) return false;
    auto idx = [&](const Cell& c) { return c.y * width + c.x; };
    auto is_blocked = [&](const Cell& c) { return !Free(c) || std::find(blocked.begin(), blocked.end(), c) != blocked.end(); };
    std::vector<double> gs(width * height, std::numeric_limits<double>::infinity());
    std::vector<int> parent(width * height, -1);
    using QE = std::pair<double, int>;
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> open;
    auto h = [&](const Cell& c) {
      double dx = std::abs(c.x - g.x), dy = std::abs(c.y - g.y);
      return (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
    };
    gs[idx(s)] = 0.0;
    open.push({h(s), idx(s)});
    static const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    while (!open.empty()) {
      auto [f, ci] = open.top();
      open.pop();
      Cell c{ci % width, ci / width};
      if (c == g) {
        for (int i = ci; i != -1; i = parent[i]) path.push_back(Cell{i % width, i / width});
        std::reverse(path.begin(), path.end());
        return true;
      }
      if (f - h(c) > gs[ci] + 1e-9) continue;
      int zc = ZoneOf(c);
      for (int k = 0; k < 8; ++k) {
        Cell n{c.x + DX[k], c.y + DY[k]};
        if (is_blocked(n) && n != g) continue;
        if (DX[k] && DY[k] && (is_blocked(Cell{c.x + DX[k], c.y}) || is_blocked(Cell{c.x, c.y + DY[k]}))) continue;
        double cost = gs[ci] + ((DX[k] && DY[k]) ? std::sqrt(2.0) : 1.0);
        if (DX[k]) {
          int lc = lane[ci], ln = lane[idx(n)];
          if (lc * DX[k] < 0 || ln * DX[k] < 0) {
            if (hard_lanes) continue;
            cost += lane_penalty;
          }
        }
        int zn = ZoneOf(n);
        if (zn >= 0 && zn != zc && zn < static_cast<int>(zone_penalty.size())) cost += zone_penalty[zn];
        int ni = idx(n);
        if (cost < gs[ni]) {
          gs[ni] = cost;
          parent[ni] = ci;
          open.push({cost + h(n), ni});
        }
      }
    }
    return false;
  }
};

}  // namespace swarmx
#endif
