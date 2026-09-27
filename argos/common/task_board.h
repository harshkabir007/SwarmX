/*
 * SwarmX - warehouse task board (the WMS) for the ARGoS experiments.
 *
 * It only publishes *what* has to be moved and records pick/delivery events
 * reported by robots at the stations (like a barcode scan). It never assigns
 * robots: allocation is a distributed auction among the robots over the
 * range-and-bearing radio.
 */
#ifndef SWARMX_TASK_BOARD_H
#define SWARMX_TASK_BOARD_H

#include <map>
#include <string>
#include <vector>

#include "warehouse_grid.h"

namespace swarmx {

struct Task {
  int id = 0;
  Cell pickup, dropoff;
  double created = 0.0;
  int picked_by = -1;      // robot index, -1 = still on the shelf
  double delivered = -1.0;  // time, -1 = not yet
};

class TaskBoard {
 public:
  static TaskBoard& Get() { static TaskBoard b; return b; }
  std::vector<Task> tasks;
  const WarehouseGrid* grid = nullptr;
  double now = 0.0;
  std::map<int, std::string> status;  // per-robot debug line (written by controllers)

  bool Open(int i) const { return tasks[i].picked_by < 0 && tasks[i].delivered < 0; }
  /* A robot at the pick face scans the item; first scan wins. */
  bool TryPick(int i, int robot) {
    if (!Open(i)) return false;
    tasks[i].picked_by = robot;
    return true;
  }
  void Deliver(int i) { if (tasks[i].delivered < 0) tasks[i].delivered = now; }
  int Delivered() const {
    int n = 0;
    for (const Task& t : tasks) n += t.delivered >= 0;
    return n;
  }
};

}  // namespace swarmx
#endif
