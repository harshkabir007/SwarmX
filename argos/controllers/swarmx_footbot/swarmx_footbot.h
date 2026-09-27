/*
 * SwarmX foot-bot controller (ARGoS3) - C++ port of the SwarmX coordination core
 * for large-scale swarm experiments.
 *
 *   communication : range-and-bearing radio only (local, line-of-sight, lossy)
 *   allocation    : distributed auction (CBBA with bundle size 1) - claim the
 *                   cheapest task nobody nearby out-bids you on
 *   planning      : A* on the warehouse grid with busy-aisle penalties
 *   choke points  : decentralized aisle locks (direction-aware convoys, FIFO
 *                   by expected entry time, settle window)
 *   local motion  : ORCA + RSS safe-following guard, neighbours taken straight
 *                   from range/bearing (no shared localization needed)
 *   mode=stopwait : traditional baseline - stop when a robot is ahead, request
 *                   an aisle only at its mouth, exclusive, no re-routing
 */
#ifndef SWARMX_FOOTBOT_H
#define SWARMX_FOOTBOT_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include <map>
#include <vector>

#include "../../common/task_board.h"
#include "../../common/warehouse_grid.h"

using namespace argos;

class CSwarmXFootbot : public CCI_Controller {
 public:
  enum class EState { IDLE, TO_PICKUP, PICKING, TO_DROPOFF, DROPPING };

  struct SNeighbour {
    double x = 0, y = 0, vx = 0, vy = 0;   // world frame (from range/bearing + own pose)
    int task = -1;
    float bid = 0;
    int zone = -1, zstate = 0, zmode = 0;  // zstate: 0 none, 1 claim, 2 hold; zmode: 0 SN, 1 NS, 2 XX
    float zkey = 0;
    bool stationary = false;
    int stop_for = -1;
    UInt32 heard = 0;
  };

  void Init(TConfigurationNode& t_node) override;
  void ControlStep() override;
  void Reset() override;
  void Destroy() override {}

  /* read by the loop functions (ground truth / metrics only) */
  int Index() const { return m_nIndex; }
  EState State() const { return m_eState; }
  int ZoneWaitTicks() const { return m_nZoneWaitTicks; }

 private:
  void ReadNeighbours();
  void Broadcast();
  void Allocate();
  bool PlanTo(const swarmx::Cell& goal);
  int UpdateZones(int limit);
  void Drive(int limit);
  void SetWheels(double vx, double vy);
  double DistAlong(int idx) const;
  bool NearCell(const swarmx::Cell& c, double tol) const;

  CCI_DifferentialSteeringActuator* m_pcWheels = nullptr;
  CCI_RangeAndBearingActuator* m_pcRABA = nullptr;
  CCI_RangeAndBearingSensor* m_pcRABS = nullptr;
  CCI_PositioningSensor* m_pcPos = nullptr;
  CCI_LEDsActuator* m_pcLEDs = nullptr;
  CRandom::CRNG* m_pcRNG = nullptr;

  swarmx::WarehouseGrid m_cGrid;
  bool m_bStopWait = false;
  int m_nIndex = 0;
  UInt32 m_unTick = 0;
  double m_fDt = 0.1;
  double m_fMaxSpeed = 0.25, m_fRadius = 0.085, m_fMargin = 0.02, m_fAxle = 0.14;
  double m_fX = 0, m_fY = 0, m_fTh = 0, m_fVx = 0, m_fVy = 0;

  std::map<int, SNeighbour> m_mapNbrs;
  EState m_eState = EState::IDLE;
  int m_nTask = -1;
  float m_fBid = 0;
  UInt32 m_unServiceUntil = 0;
  std::vector<swarmx::Cell> m_vRoute;
  int m_nRouteIdx = 0;
  int m_nZone = -1, m_nZoneState = 0, m_nZoneMode = 2, m_nZoneStart = -1, m_nZoneEnd = -1;
  float m_fZoneKey = 0;
  UInt32 m_unClaimTick = 0;
  int m_nZoneWaitTicks = 0;
  int m_nStopFor = -1;
  double m_fBestRemain = 1e9;
  UInt32 m_unProgressTick = 0, m_unJitterUntil = 0;
  double m_fJx = 0, m_fJy = 0;
};

#endif
