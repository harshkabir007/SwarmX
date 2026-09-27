/*
 * SwarmX foot-bot controller - see swarmx_footbot.h.
 *
 * Radio payload (32 bytes, little endian):
 *   0  u16 robot index          2  f32 vx           6  f32 vy
 *  10  i16 task (-1 none)      12  f32 bid         16  i16 zone (-1 none)
 *  18  u8  zone state          19  u8  zone mode   20  f32 zone key
 *  24  u8  flags (1 stationary) 25 i16 stop_for    27..31 reserved
 * Neighbour positions are NOT in the payload: they come from the physical
 * range-and-bearing measurement, exactly like an IR/UWB ranging module.
 */
#include "swarmx_footbot.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/vector2.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

using V = std::pair<double, double>;
const double EPS = 1e-5;

double Det(const V& a, const V& b) { return a.first * b.second - a.second * b.first; }
double Dot(const V& a, const V& b) { return a.first * b.first + a.second * b.second; }
V Sub(const V& a, const V& b) { return {a.first - b.first, a.second - b.second}; }
V Add(const V& a, const V& b) { return {a.first + b.first, a.second + b.second}; }
V Mul(const V& a, double s) { return {a.first * s, a.second * s}; }
double AbsSq(const V& a) { return Dot(a, a); }
V Norm(const V& a) {
  double n = std::sqrt(AbsSq(a));
  return n > EPS ? V{a.first / n, a.second / n} : V{0, 0};
}

struct Line { V p, d; };
struct Nbr { V pos, vel; double r, resp; };

/* ---- RVO2 linear programs (van den Berg et al.), same as swarmx_core/orca.py ---- */
bool LP1(const std::vector<Line>& L, size_t no, double R, const V& opt, bool dir, V& res) {
  double dot = Dot(L[no].p, L[no].d);
  double disc = dot * dot + R * R - AbsSq(L[no].p);
  if (disc < 0) return false;
  double sq = std::sqrt(disc), tl = -dot - sq, tr = -dot + sq;
  for (size_t i = 0; i < no; ++i) {
    double den = Det(L[no].d, L[i].d), num = Det(L[i].d, Sub(L[no].p, L[i].p));
    if (std::fabs(den) <= EPS) { if (num < 0) return false; continue; }
    double t = num / den;
    if (den >= 0) tr = std::min(tr, t); else tl = std::max(tl, t);
    if (tl > tr) return false;
  }
  double t;
  if (dir) t = Dot(opt, L[no].d) > 0 ? tr : tl;
  else t = std::min(std::max(Dot(L[no].d, Sub(opt, L[no].p)), tl), tr);
  res = Add(L[no].p, Mul(L[no].d, t));
  return true;
}

size_t LP2(const std::vector<Line>& L, double R, const V& opt, bool dir, V& res) {
  if (dir) res = Mul(opt, R);
  else if (AbsSq(opt) > R * R) res = Mul(Norm(opt), R);
  else res = opt;
  for (size_t i = 0; i < L.size(); ++i)
    if (Det(L[i].d, Sub(L[i].p, res)) > 0) {
      V r;
      if (!LP1(L, i, R, opt, dir, r)) return i;
      res = r;
    }
  return L.size();
}

void LP3(const std::vector<Line>& L, size_t nObst, size_t begin, double R, V& res) {
  double dist = 0;
  for (size_t i = begin; i < L.size(); ++i) {
    if (Det(L[i].d, Sub(L[i].p, res)) <= dist) continue;
    std::vector<Line> proj(L.begin(), L.begin() + nObst);
    for (size_t j = nObst; j < i; ++j) {
      Line ln;
      double det = Det(L[i].d, L[j].d);
      if (std::fabs(det) <= EPS) {
        if (Dot(L[i].d, L[j].d) > 0) continue;
        ln.p = Mul(Add(L[i].p, L[j].p), 0.5);
      } else {
        ln.p = Add(L[i].p, Mul(L[i].d, Det(L[j].d, Sub(L[i].p, L[j].p)) / det));
      }
      ln.d = Norm(Sub(L[j].d, L[i].d));
      proj.push_back(ln);
    }
    V r;
    if (LP2(proj, R, V{-L[i].d.second, L[i].d.first}, true, r) >= proj.size()) res = r;
    dist = Det(L[i].d, Sub(L[i].p, res));
  }
}

V Orca(const V& pos, const V& vel, const V& pref, double r, double vmax, const std::vector<Nbr>& nb,
       const std::vector<std::pair<double, double>>& walls, double th = 2.0, double tho = 0.5, double dt = 0.1) {
  std::vector<Line> L;
  for (const auto& w : walls) {
    V rel = Sub(V{w.first, w.second}, pos);
    double d = std::sqrt(AbsSq(rel));
    if (d < EPS) continue;
    V n{rel.first / d, rel.second / d};
    L.push_back(Line{Mul(n, (d - r) / tho), V{-n.second, n.first}});
  }
  size_t nObst = L.size();
  for (const Nbr& o : nb) {
    V rp = Sub(o.pos, pos), rv = Sub(vel, o.vel), u, d;
    double ds = AbsSq(rp), cr = r + o.r, crs = cr * cr;
    if (ds > crs) {
      V w = Sub(rv, Mul(rp, 1.0 / th));
      double wls = AbsSq(w), d1 = Dot(w, rp);
      if (d1 < 0 && d1 * d1 > crs * wls) {
        double wl = std::sqrt(wls);
        V uw = Mul(w, 1.0 / wl);
        d = V{uw.second, -uw.first};
        u = Mul(uw, cr / th - wl);
      } else {
        double leg = std::sqrt(ds - crs);
        if (Det(rp, w) > 0) d = V{(rp.first * leg - rp.second * cr) / ds, (rp.first * cr + rp.second * leg) / ds};
        else d = V{-(rp.first * leg + rp.second * cr) / ds, -(-rp.first * cr + rp.second * leg) / ds};
        u = Sub(Mul(d, Dot(rv, d)), rv);
      }
    } else {
      V w = Sub(rv, Mul(rp, 1.0 / dt));
      double wl = std::sqrt(AbsSq(w));
      V uw = wl > EPS ? Mul(w, 1.0 / wl) : V{1, 0};
      d = V{uw.second, -uw.first};
      u = Mul(uw, cr / dt - wl);
    }
    L.push_back(Line{Add(vel, Mul(u, o.resp)), d});
  }
  V res;
  size_t fail = LP2(L, vmax, pref, false, res);
  if (fail < L.size()) LP3(L, nObst, fail, vmax, res);
  return res;
}

void PutF(CByteArray& b, size_t at, float f) { UInt8 raw[4]; std::memcpy(raw, &f, 4); for (int i = 0; i < 4; ++i) b[at + i] = raw[i]; }
float GetF(const CByteArray& b, size_t at) { UInt8 raw[4]; for (int i = 0; i < 4; ++i) raw[i] = b[at + i]; float f; std::memcpy(&f, raw, 4); return f; }
void PutI16(CByteArray& b, size_t at, int v) { uint16_t u = static_cast<uint16_t>(static_cast<int16_t>(v)); b[at] = u & 0xFF; b[at + 1] = u >> 8; }
int GetI16(const CByteArray& b, size_t at) { return static_cast<int16_t>(static_cast<uint16_t>(b[at] | (b[at + 1] << 8))); }

bool ModesConflict(int a, int b) { return a == 2 || b == 2 || a != b; }

}  // namespace

/* ======================================================================== */
void CSwarmXFootbot::Init(TConfigurationNode& t_node) {
  m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
  m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
  m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
  m_pcPos = GetSensor<CCI_PositioningSensor>("positioning");
  std::string map, mode = "swarmx";
  GetNodeAttribute(t_node, "map", map);
  GetNodeAttributeOrDefault(t_node, "mode", mode, mode);
  GetNodeAttributeOrDefault(t_node, "max_speed", m_fMaxSpeed, m_fMaxSpeed);
  m_bStopWait = (mode == "stopwait");
  m_cGrid.Load(map);
  const std::string& id = GetId();
  m_nIndex = std::stoi(id.substr(id.find_first_of("0123456789")));
  m_pcRNG = CRandom::CreateRNG("argos");
  Reset();
}

void CSwarmXFootbot::Reset() {
  m_eState = EState::IDLE;
  m_nTask = -1;
  m_vRoute.clear();
  m_nRouteIdx = 0;
  m_nZone = -1;
  m_nZoneState = 0;
  m_mapNbrs.clear();
  m_unTick = 0;
}

/* ------------------------------------------------------------------ comms */
void CSwarmXFootbot::ReadNeighbours() {
  for (const auto& pkt : m_pcRABS->GetReadings()) {
    if (pkt.Data.Size() < 27) continue;
    int idx = GetI16(pkt.Data, 0);
    double rng = pkt.Range / 100.0;  // cm -> m
    double ang = m_fTh + pkt.HorizontalBearing.GetValue();
    SNeighbour& n = m_mapNbrs[idx];
    n.x = m_fX + rng * std::cos(ang);
    n.y = m_fY + rng * std::sin(ang);
    n.vx = GetF(pkt.Data, 2);
    n.vy = GetF(pkt.Data, 6);
    n.task = GetI16(pkt.Data, 10);
    n.bid = GetF(pkt.Data, 12);
    n.zone = GetI16(pkt.Data, 16);
    n.zstate = pkt.Data[18];
    n.zmode = pkt.Data[19];
    n.zkey = GetF(pkt.Data, 20);
    n.stationary = pkt.Data[24] & 1;
    n.stop_for = GetI16(pkt.Data, 25);
    n.heard = m_unTick;
  }
  for (auto it = m_mapNbrs.begin(); it != m_mapNbrs.end();)
    it = (m_unTick - it->second.heard > 15) ? m_mapNbrs.erase(it) : std::next(it);
}

void CSwarmXFootbot::Broadcast() {
  CByteArray b(32, 0);
  PutI16(b, 0, m_nIndex);
  PutF(b, 2, static_cast<float>(m_fVx));
  PutF(b, 6, static_cast<float>(m_fVy));
  PutI16(b, 10, m_nTask);
  PutF(b, 12, m_fBid);
  PutI16(b, 16, m_nZone);
  b[18] = static_cast<UInt8>(m_nZoneState);
  b[19] = static_cast<UInt8>(m_nZoneMode);
  PutF(b, 20, m_fZoneKey);
  b[24] = (m_eState == EState::PICKING || m_eState == EState::DROPPING) ? 1 : 0;
  PutI16(b, 25, m_nStopFor);
  m_pcRABA->SetData(b);
}

/* ------------------------------------------------------------- allocation */
void CSwarmXFootbot::Allocate() {
  swarmx::TaskBoard& tb = swarmx::TaskBoard::Get();
  if (m_eState == EState::TO_PICKUP && !tb.Open(m_nTask)) {  // someone scanned it first
    m_eState = EState::IDLE;
    m_nTask = -1;
  }
  if (m_eState == EState::TO_PICKUP) {  // out-bid by a neighbour on the same task?
    for (const auto& kv : m_mapNbrs) {
      const SNeighbour& n = kv.second;
      if (n.task == m_nTask && (n.bid < m_fBid - 1e-4 || (std::fabs(n.bid - m_fBid) <= 1e-4 && kv.first < m_nIndex))) {
        m_eState = EState::IDLE;
        m_nTask = -1;
        break;
      }
    }
  }
  if (m_eState != EState::IDLE) return;
  swarmx::Cell here = m_cGrid.CellOf(m_fX, m_fY);
  int best = -1;
  float bbid = 1e9f;
  for (size_t i = 0; i < tb.tasks.size(); ++i) {
    if (!tb.Open(i)) continue;
    const swarmx::Task& t = tb.tasks[i];
    float bid = static_cast<float>(std::abs(t.pickup.x - here.x) + std::abs(t.pickup.y - here.y));
    bool beaten = false;
    for (const auto& kv : m_mapNbrs)
      if (kv.second.task == static_cast<int>(i) &&
          (kv.second.bid < bid - 1e-4 || (std::fabs(kv.second.bid - bid) <= 1e-4 && kv.first < m_nIndex))) beaten = true;
    if (!beaten && bid < bbid) { bbid = bid; best = static_cast<int>(i); }
  }
  if (best < 0) return;
  if (PlanTo(tb.tasks[best].pickup)) {
    m_nTask = best;
    m_fBid = bbid;
    m_eState = EState::TO_PICKUP;
  }
}

bool CSwarmXFootbot::PlanTo(const swarmx::Cell& goal) {
  std::vector<double> pen(m_cGrid.zones.size(), 0.0);
  if (!m_bStopWait)  // congestion-aware routing from neighbours' announced aisle use
    for (const auto& kv : m_mapNbrs)
      if (kv.second.zone >= 0 && kv.second.zstate > 0) pen[kv.second.zone] += 6.0;
  std::vector<swarmx::Cell> blocked;
  swarmx::Cell here = m_cGrid.CellOf(m_fX, m_fY);
  if (!m_cGrid.Free(here)) {  // nudged onto a corner: start from the nearest free cell
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        if (m_cGrid.Free(swarmx::Cell{here.x + dx, here.y + dy})) { here = swarmx::Cell{here.x + dx, here.y + dy}; dx = 2; break; }
  }
  if (!m_cGrid.AStar(here, goal, m_vRoute, pen, blocked)) return false;
  m_nRouteIdx = 0;
  m_fBestRemain = 1e9;
  m_unProgressTick = m_unTick;
  return true;
}

double CSwarmXFootbot::DistAlong(int idx) const {
  if (m_vRoute.empty()) return 0;
  idx = std::min<int>(std::max(idx, m_nRouteIdx), m_vRoute.size() - 1);
  double cx, cy;
  m_cGrid.Center(m_vRoute[m_nRouteIdx], cx, cy);
  double d = std::hypot(cx - m_fX, cy - m_fY);
  for (int i = m_nRouteIdx; i < idx; ++i)
    d += std::hypot(m_vRoute[i + 1].x - m_vRoute[i].x, m_vRoute[i + 1].y - m_vRoute[i].y) * m_cGrid.cell;
  return d;
}

bool CSwarmXFootbot::NearCell(const swarmx::Cell& c, double tol) const {
  double cx, cy;
  m_cGrid.Center(c, cx, cy);
  return std::hypot(cx - m_fX, cy - m_fY) < tol;
}

/* ----------------------------------------------------------- aisle locks */
int CSwarmXFootbot::UpdateZones(int limit) {
  swarmx::Cell here = m_cGrid.CellOf(m_fX, m_fY);
  int hz = m_cGrid.ZoneOf(here);
  if (m_nZone >= 0 && m_nZoneState == 2 && hz != m_nZone) {
    const swarmx::Zone& z = m_cGrid.zones[m_nZone];
    double cx = (z.x + 0.5) * m_cGrid.cell;
    double y0 = z.y0 * m_cGrid.cell, y1 = (z.y1 + 1) * m_cGrid.cell;
    double dy = std::max({y0 - m_fY, 0.0, m_fY - y1}), dx = std::fabs(m_fX - cx) - 0.5 * m_cGrid.cell;
    if (std::hypot(std::max(dx, 0.0), dy) > m_fRadius + 0.03) { m_nZone = -1; m_nZoneState = 0; }  // released
  }
  if (m_nZoneState == 2) return limit;  // inside / committed: go
  // find the next zone on the route
  int zs = -1, ze = -1, zid = -1;
  for (int i = m_nRouteIdx; i < static_cast<int>(m_vRoute.size()); ++i) {
    int z = m_cGrid.ZoneOf(m_vRoute[i]);
    if (z >= 0) { zid = z; zs = i; ze = i; while (ze + 1 < static_cast<int>(m_vRoute.size()) && m_cGrid.ZoneOf(m_vRoute[ze + 1]) == z) ++ze; break; }
  }
  if (zid < 0) { m_nZone = -1; m_nZoneState = 0; return limit; }
  const swarmx::Zone& z = m_cGrid.zones[zid];
  int mode = 2;
  if (!m_bStopWait && zs > 0 && ze + 1 < static_cast<int>(m_vRoute.size())) {
    bool fromS = m_vRoute[zs - 1] == z.south(), toN = m_vRoute[ze + 1] == z.north();
    bool fromN = m_vRoute[zs - 1] == z.north(), toS = m_vRoute[ze + 1] == z.south();
    if (fromS && toN) mode = 0; else if (fromN && toS) mode = 1;
  }
  double dEntry = DistAlong(zs);
  int stop = std::max(zs - 2, m_nRouteIdx);
  double lookahead = m_bStopWait ? 0.6 * m_cGrid.cell : 6.0 * m_cGrid.cell;
  if (m_bStopWait && !(m_nRouteIdx >= zs - 2 && NearCell(m_vRoute[std::max(zs - 2, 0)], 0.25 * m_cGrid.cell))
      && m_nRouteIdx < zs - 2) {
    return std::min(limit, stop);  // traditional: drive to the mouth, stop, only then request
  }
  if (dEntry > lookahead) return limit;
  if (m_nZone != zid || m_nZoneState == 0) {
    m_nZone = zid;
    m_nZoneState = 1;
    m_nZoneMode = mode;
    m_fZoneKey = static_cast<float>(m_unTick * m_fDt + dEntry / (0.8 * m_fMaxSpeed));
    m_unClaimTick = m_unTick;
  }
  bool ok = m_unTick - m_unClaimTick >= 3;  // settle window (claim visible to neighbours)
  for (const auto& kv : m_mapNbrs) {
    const SNeighbour& n = kv.second;
    if (n.zone != zid || !ModesConflict(mode, n.zmode)) continue;
    if (n.zstate == 2) ok = false;
    if (n.zstate == 1 && (n.zkey < m_fZoneKey || (n.zkey == m_fZoneKey && kv.first < m_nIndex))) ok = false;
  }
  if (ok) {
    if (dEntry <= 1.6 * m_cGrid.cell || m_nRouteIdx >= zs - 1) m_nZoneState = 2;
    return limit;
  }
  ++m_nZoneWaitTicks;
  return std::min(limit, stop);
}

/* ---------------------------------------------------------------- motion */
void CSwarmXFootbot::Drive(int limit) {
  // advance along the route
  while (m_nRouteIdx < std::min<int>(limit, m_vRoute.size() - 1) && NearCell(m_vRoute[m_nRouteIdx], 0.35 * m_cGrid.cell))
    ++m_nRouteIdx;
  V pref{0, 0};
  if (!m_vRoute.empty()) {
    double tx, ty;
    m_cGrid.Center(m_vRoute[std::min<int>(m_nRouteIdx, m_vRoute.size() - 1)], tx, ty);
    double dx = tx - m_fX, dy = ty - m_fY, d = std::hypot(dx, dy), remain = DistAlong(limit);
    if (d > 1e-4 && remain > 0.01) {
      double sp = std::min(m_fMaxSpeed, std::sqrt(2.0 * 0.5 * std::max(remain - 0.01, 0.0)));
      pref = V{dx / d * sp, dy / d * sp};
    }
  }
  if (m_unTick < m_unJitterUntil) pref = Add(pref, V{m_fJx, m_fJy});
  std::vector<Nbr> nb;
  for (const auto& kv : m_mapNbrs) {
    const SNeighbour& n = kv.second;
    if (std::hypot(n.x - m_fX, n.y - m_fY) > 1.5) continue;
    nb.push_back(Nbr{V{n.x, n.y}, V{n.vx, n.vy}, m_fRadius + m_fMargin, (n.stationary || m_bStopWait) ? 1.0 : 0.5});
  }
  // aisles I do not hold are walls (no entry without a lock)
  std::vector<swarmx::Cell> extra;
  swarmx::Cell here = m_cGrid.CellOf(m_fX, m_fY);
  int hz = m_cGrid.ZoneOf(here);
  for (int x = here.x - 2; x <= here.x + 2; ++x)
    for (int y = here.y - 2; y <= here.y + 2; ++y) {
      int z = m_cGrid.ZoneOf(swarmx::Cell{x, y});
      if (z >= 0 && z != hz && !(z == m_nZone && m_nZoneState == 2)) extra.push_back(swarmx::Cell{x, y});
    }
  std::vector<std::pair<double, double>> walls;
  m_cGrid.WallPoints(m_fX, m_fY, 0.4, extra, walls);
  V v;
  m_nStopFor = -1;
  if (m_bStopWait) {
    // traditional: stop if any robot is within the protective field ahead
    double sp = std::sqrt(AbsSq(pref));
    bool stop = false;
    if (sp > 1e-3) {
      for (const auto& kv : m_mapNbrs) {
        double rx = kv.second.x - m_fX, ry = kv.second.y - m_fY;
        double fwd = (rx * pref.first + ry * pref.second) / sp, lat = std::fabs(rx * pref.second - ry * pref.first) / sp;
        if (fwd > 0 && fwd < 3.5 * m_fRadius && lat < 2.4 * m_fRadius) { stop = true; m_nStopFor = kv.first; }
      }
    }
    v = stop ? V{0, 0} : Orca(V{m_fX, m_fY}, V{m_fVx, m_fVy}, pref, m_fRadius + m_fMargin, m_fMaxSpeed, {}, walls);
  } else {
    v = Orca(V{m_fX, m_fY}, V{m_fVx, m_fVy}, pref, m_fRadius + m_fMargin, m_fMaxSpeed, nb, walls);
    // RSS safe-following guard (bounded deceleration + latency)
    const double a = 0.6, tau = 0.2;
    for (const Nbr& n : nb) {
      V rel = Sub(n.pos, V{m_fX, m_fY});
      double d = std::sqrt(AbsSq(rel));
      if (d < 1e-6) continue;
      V u = Mul(rel, 1.0 / d);
      double mine = Dot(v, u);
      if (mine <= 0) continue;
      double theirs = Dot(n.vel, u), away = std::max(theirs, 0.0), toward = std::max(-theirs, 0.0);
      double gap = d - 2 * m_fRadius - 0.02 + away * away / (2 * a) - toward * tau - toward * toward / (2 * a);
      double vmax = gap <= 0 ? 0.0 : a * (-tau + std::sqrt(tau * tau + 2 * gap / a));
      if (mine > vmax) v = Sub(v, Mul(u, mine - vmax));
    }
  }
  // livelock / deadlock breaker
  double remain = DistAlong(m_vRoute.empty() ? 0 : m_vRoute.size() - 1);
  bool wants = std::sqrt(AbsSq(pref)) > 0.01;
  if (!wants || remain < m_fBestRemain - 0.05) { m_fBestRemain = remain; m_unProgressTick = m_unTick; }
  else if (m_unTick - m_unProgressTick > 100 && hz < 0) {
    CRadians ang = m_pcRNG->Uniform(CRange<CRadians>(-CRadians::PI, CRadians::PI));
    m_fJx = 0.5 * m_fMaxSpeed * std::cos(ang.GetValue());
    m_fJy = 0.5 * m_fMaxSpeed * std::sin(ang.GetValue());
    m_unJitterUntil = m_unTick + 20;
    m_unProgressTick = m_unTick;
    m_fBestRemain = remain;
    if (m_eState == EState::TO_PICKUP) PlanTo(swarmx::TaskBoard::Get().tasks[m_nTask].pickup);
    else if (m_eState == EState::TO_DROPOFF) PlanTo(swarmx::TaskBoard::Get().tasks[m_nTask].dropoff);
  }
  SetWheels(v.first, v.second);
}

void CSwarmXFootbot::SetWheels(double vx, double vy) {
  double sp = std::hypot(vx, vy), v = 0, w = 0;
  if (sp > 0.005) {
    double err = std::atan2(std::sin(std::atan2(vy, vx) - m_fTh), std::cos(std::atan2(vy, vx) - m_fTh));
    w = std::max(-3.0, std::min(3.0, 4.0 * err));
    v = std::fabs(err) > M_PI / 2 ? 0.0 : sp * std::cos(err);
  }
  double left = (v - w * m_fAxle / 2) * 100.0, right = (v + w * m_fAxle / 2) * 100.0;  // cm/s
  m_pcWheels->SetLinearVelocity(left, right);
}

/* ------------------------------------------------------------ main loop */
void CSwarmXFootbot::ControlStep() {
  ++m_unTick;
  const auto& r = m_pcPos->GetReading();
  CRadians z, y, x;
  r.Orientation.ToEulerAngles(z, y, x);
  double nx = r.Position.GetX(), ny = r.Position.GetY();
  if (m_unTick > 1) { m_fVx = (nx - m_fX) / m_fDt; m_fVy = (ny - m_fY) / m_fDt; }
  m_fX = nx; m_fY = ny; m_fTh = z.GetValue();
  ReadNeighbours();
  swarmx::TaskBoard& tb = swarmx::TaskBoard::Get();
  Allocate();

  int limit = m_vRoute.empty() ? 0 : static_cast<int>(m_vRoute.size()) - 1;
  switch (m_eState) {
    case EState::TO_PICKUP:
      if (m_nRouteIdx >= limit && NearCell(tb.tasks[m_nTask].pickup, 0.3 * m_cGrid.cell)) {
        if (tb.TryPick(m_nTask, m_nIndex)) { m_eState = EState::PICKING; m_unServiceUntil = m_unTick + 20; }
        else { m_eState = EState::IDLE; m_nTask = -1; }
      }
      break;
    case EState::PICKING:
      if (m_unTick >= m_unServiceUntil) { PlanTo(tb.tasks[m_nTask].dropoff); m_eState = EState::TO_DROPOFF; }
      break;
    case EState::TO_DROPOFF:
      if (m_nRouteIdx >= limit && NearCell(tb.tasks[m_nTask].dropoff, 0.3 * m_cGrid.cell)) {
        m_eState = EState::DROPPING;
        m_unServiceUntil = m_unTick + 15;
      }
      break;
    case EState::DROPPING:
      if (m_unTick >= m_unServiceUntil) { tb.Deliver(m_nTask); m_nTask = -1; m_eState = EState::IDLE; m_vRoute.clear(); }
      break;
    default:
      break;
  }
  bool service = (m_eState == EState::PICKING || m_eState == EState::DROPPING);
  if (m_vRoute.empty() || service) {
    SetWheels(0, 0);
    if (m_eState == EState::IDLE) m_pcLEDs->SetAllColors(CColor::GRAY50);
  } else {
    limit = UpdateZones(limit);
    Drive(limit);
    m_pcLEDs->SetAllColors(m_eState == EState::TO_DROPOFF ? CColor::ORANGE : CColor::BLUE);
  }
  Broadcast();
  {
    char buf[200];
    swarmx::Cell c = m_cGrid.CellOf(m_fX, m_fY);
    std::snprintf(buf, sizeof(buf), "state=%d task=%d cell=(%d,%d) route=%d/%zu zone=%d zstate=%d stop_for=%d nbrs=%zu",
                  static_cast<int>(m_eState), m_nTask, c.x, c.y, m_nRouteIdx, m_vRoute.size(), m_nZone, m_nZoneState,
                  m_nStopFor, m_mapNbrs.size());
    tb.status[m_nIndex] = buf;
  }
}

REGISTER_CONTROLLER(CSwarmXFootbot, "swarmx_footbot_controller")
