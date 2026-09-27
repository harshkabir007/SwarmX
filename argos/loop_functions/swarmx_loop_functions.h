/*
 * SwarmX ARGoS loop functions: task source (WMS) + ground-truth metrics.
 * Writes a JSON summary (makespan, total completion time, collisions, closest
 * pass, deliveries) and a CSV time series at the end of the experiment.
 */
#ifndef SWARMX_LOOP_FUNCTIONS_H
#define SWARMX_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <fstream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "../common/task_board.h"
#include "../common/warehouse_grid.h"

using namespace argos;

class CSwarmXLoopFunctions : public CLoopFunctions {
 public:
  void Init(TConfigurationNode& t_tree) override;
  void Reset() override;
  void PreStep() override;
  void PostStep() override;
  bool IsExperimentFinished() override;
  void PostExperiment() override;

 private:
  void MakeTasks();
  swarmx::WarehouseGrid m_cGrid;
  std::vector<CFootBotEntity*> m_vBots;
  std::string m_strOutput, m_strMode;
  int m_nTasks = 50;
  UInt32 m_unSeed = 1;
  double m_fRadius = 0.085;
  UInt64 m_unCollisions = 0;
  double m_fMinSep = 1e9;
  std::set<std::pair<int, int>> m_setContact;
  std::ofstream m_cCSV;
};

#endif
