#include "swarmx_loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>

#include <cmath>
#include <iomanip>
#include <random>

void CSwarmXLoopFunctions::Init(TConfigurationNode& t_tree) {
  std::string map;
  TConfigurationNode& cfg = GetNode(t_tree, "swarmx");
  GetNodeAttribute(cfg, "map", map);
  GetNodeAttributeOrDefault(cfg, "tasks", m_nTasks, m_nTasks);
  GetNodeAttributeOrDefault(cfg, "seed", m_unSeed, m_unSeed);
  GetNodeAttributeOrDefault(cfg, "output", m_strOutput, std::string("swarmx_argos_result.json"));
  GetNodeAttributeOrDefault(cfg, "mode", m_strMode, std::string("swarmx"));
  m_cGrid.Load(map);
  swarmx::TaskBoard::Get().grid = &m_cGrid;
  for (auto& kv : GetSpace().GetEntitiesByType("foot-bot"))
    m_vBots.push_back(any_cast<CFootBotEntity*>(kv.second));
  MakeTasks();
  m_cCSV.open(m_strOutput + ".csv");
  m_cCSV << "time_s,delivered,collisions,min_separation_m\n";
}

void CSwarmXLoopFunctions::MakeTasks() {
  std::mt19937 rng(m_unSeed);
  swarmx::TaskBoard& tb = swarmx::TaskBoard::Get();
  tb.tasks.clear();
  for (int i = 0; i < m_nTasks; ++i) {
    swarmx::Task t;
    t.id = i;
    t.pickup = m_cGrid.pickups[rng() % m_cGrid.pickups.size()];
    t.dropoff = m_cGrid.dropoffs[rng() % m_cGrid.dropoffs.size()];
    tb.tasks.push_back(t);
  }
}

void CSwarmXLoopFunctions::Reset() {
  m_unCollisions = 0;
  m_fMinSep = 1e9;
  m_setContact.clear();
  MakeTasks();
}

void CSwarmXLoopFunctions::PreStep() {
  swarmx::TaskBoard::Get().now = GetSpace().GetSimulationClock() / static_cast<double>(CPhysicsEngine::GetInverseSimulationClockTick());
}

void CSwarmXLoopFunctions::PostStep() {
  std::set<std::pair<int, int>> contact;
  for (size_t i = 0; i < m_vBots.size(); ++i) {
    const CVector3& a = m_vBots[i]->GetEmbodiedEntity().GetOriginAnchor().Position;
    for (size_t j = i + 1; j < m_vBots.size(); ++j) {
      const CVector3& b = m_vBots[j]->GetEmbodiedEntity().GetOriginAnchor().Position;
      double d = std::hypot(a.GetX() - b.GetX(), a.GetY() - b.GetY());
      m_fMinSep = std::min(m_fMinSep, d);
      if (d < 2 * m_fRadius - 0.002) contact.insert({static_cast<int>(i), static_cast<int>(j)});
    }
  }
  for (const auto& p : contact)
    if (!m_setContact.count(p)) ++m_unCollisions;
  m_setContact = contact;
  if (GetSpace().GetSimulationClock() % 100 == 0)
    m_cCSV << swarmx::TaskBoard::Get().now << "," << swarmx::TaskBoard::Get().Delivered() << "," << m_unCollisions
           << "," << m_fMinSep << "\n";
}

bool CSwarmXLoopFunctions::IsExperimentFinished() {
  return swarmx::TaskBoard::Get().Delivered() == static_cast<int>(swarmx::TaskBoard::Get().tasks.size());
}

void CSwarmXLoopFunctions::PostExperiment() {
  const swarmx::TaskBoard& tb = swarmx::TaskBoard::Get();
  double makespan = 0, total = 0;
  int done = 0;
  for (const auto& t : tb.tasks) {
    double end = t.delivered >= 0 ? t.delivered : tb.now;  // unfinished tasks charged the time limit
    total += end - t.created;
    if (t.delivered >= 0) { ++done; makespan = std::max(makespan, t.delivered); }
  }
  if (done < static_cast<int>(tb.tasks.size())) makespan = tb.now;
  std::ofstream out(m_strOutput);
  out << std::fixed << std::setprecision(3) << "{\"mode\": \"" << m_strMode << "\", \"robots\": " << m_vBots.size()
      << ", \"tasks\": " << tb.tasks.size() << ", \"delivered\": " << done << ", \"makespan_s\": " << makespan
      << ", \"total_completion_s\": " << total << ", \"collisions\": " << m_unCollisions
      << ", \"min_separation_m\": " << m_fMinSep << ", \"seed\": " << m_unSeed << "}\n";
  m_cCSV.close();
  for (const auto& t : tb.tasks)
    if (t.delivered < 0)
      LOG << "[swarmx] undelivered task " << t.id << " pickup (" << t.pickup.x << "," << t.pickup.y << ") dropoff ("
          << t.dropoff.x << "," << t.dropoff.y << ") picked_by " << t.picked_by << std::endl;
  if (done < static_cast<int>(tb.tasks.size()))
    for (const auto& kv : tb.status) LOG << "[swarmx] robot " << kv.first << ": " << kv.second << std::endl;
  LOG << "[swarmx] " << done << "/" << tb.tasks.size() << " delivered, makespan " << makespan << " s, collisions "
      << m_unCollisions << std::endl;
}

REGISTER_LOOP_FUNCTIONS(CSwarmXLoopFunctions, "swarmx_loop_functions")
