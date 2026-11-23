// Ring-buffer incremental Euclidean distance field (2D) with FIESTA-style waves, capped at dmax.
//
// Port of the v1 implementation (esdf_local_costmap_benchmarking/esdf_code/ring_esdf_v1.hpp) with ONE
// functional change: the window may be non-square (Nx x Ny), as Nav2's local costmap may be.
// The algorithm is otherwise unchanged:
//   * world cell (x, y) lives in slot (x mod Nx, y mod Ny); a surviving cell never moves in memory
//   * per cell: squared integer distance, closest obstacle (WORLD coordinates), dependency-list links
//   * insert / delete / update queues (waves), distances capped at dmax
//   * 5-step window shift: unlink + orphan repair, wipe, insert entering obstacles, pull for entering
//     free cells, then the waves (run by update())
//   * full rebuild when the window jumps by >= Nx or >= Ny cells
//
// Usage per costmap update (this is what EsdfLayer does):
//   if (origin moved)  field.shift(new_ox, new_oy, occAt);
//   field.syncOccupancy(occAt);      // compare window with stored occupancy, queue the differences
//   field.update();                  // run the waves
// where occAt(x, y) -> 0/1 is the occupancy of WORLD cell (x, y) inside the (new) window.
#pragma once
#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <utility>
#include <vector>

namespace nav2_esdf
{

class RingEsdf
{
public:
  static constexpr int INF2 = INT_MAX / 2;   // "no obstacle within dmax"
  static constexpr int UND = INT_MIN;        // "no closest obstacle"

  struct Cell
  {
    int d2;           // squared distance (cells^2) to the closest obstacle, INF2 if none
    int cx, cy;       // closest obstacle, world cell coordinates (UND if none)
    int prev, next;   // dependency list of the closest obstacle (slot indices)
    int head;         // if this cell is an obstacle: first dependent
    uint8_t occ;
  };
  struct QE { int x, y, sx, sy, d2; };

  int Nx, Ny;
  float dmax;       // cap in cells (>= 1e5 means uncapped)
  int dmax2;
  int ox = 0, oy = 0;   // world cell of the window's lower-left corner
  std::vector<Cell> c;
  long long pops = 0;
  int last_entering = 0, last_orphans = 0;
  bool last_full_rebuild = false;
  bool pull_on_pop = false;   // optional per-pop pull check (off, as in v1)

  RingEsdf(int nx, int ny, float d_max_cells)
  : Nx(nx), Ny(ny), dmax(d_max_cells)
  {
    dmax2 = d_max_cells >= 1e5f ? INT_MAX / 4 : static_cast<int>(std::floor(static_cast<double>(d_max_cells) * d_max_cells + 1e-6));
    c.assign(static_cast<size_t>(Nx) * Ny, Cell{INF2, UND, UND, -1, -1, -1, 0});
  }

  // ---- addressing ----
  inline int mdx(int a) const {int r = a % Nx; return r < 0 ? r + Nx : r;}
  inline int mdy(int a) const {int r = a % Ny; return r < 0 ? r + Ny : r;}
  inline int slot(int x, int y) const {return mdx(x) + Nx * mdy(y);}
  inline bool inWin(int x, int y) const {return x >= ox && x < ox + Nx && y >= oy && y < oy + Ny;}
  static inline bool inRect(int x, int y, int ax, int ay, int w, int h)
  {
    return x >= ax && x < ax + w && y >= ay && y < ay + h;
  }
  static inline int sq(int x0, int y0, int x1, int y1)
  {
    int dx = x0 - x1, dy = y0 - y1;
    return dx * dx + dy * dy;
  }
  inline int wrapx(int s) const {return s < 0 ? s + Nx : (s >= Nx ? s - Nx : s);}
  inline int wrapy(int s) const {return s < 0 ? s + Ny : (s >= Ny ? s - Ny : s);}
  inline int worldX(int s) const {return ox + mdx(s % Nx - ox);}
  inline int worldY(int s) const {return oy + mdy(s / Nx - oy);}

  // ---- read-out ----
  inline bool hasObstacle(int s) const {return c[s].cx != UND;}
  // squared distance in cells^2 of the world cell; INF2 if none within dmax or outside the window
  int d2At(int x, int y) const {return inWin(x, y) ? c[slot(x, y)].d2 : INF2;}
  // closest obstacle of a world cell; false if none within dmax or outside the window
  bool closest(int x, int y, int & obx, int & oby) const
  {
    if (!inWin(x, y)) {return false;}
    const Cell & k = c[slot(x, y)];
    if (k.cx == UND) {return false;}
    obx = k.cx; oby = k.cy;
    return true;
  }
  long long memoryBytes() const {return static_cast<long long>(c.size()) * sizeof(Cell);}

  // ---- dependency lists ----
  inline void unlink(int s)
  {
    Cell & k = c[s];
    if (k.cx == UND) {return;}
    int o = slot(k.cx, k.cy);
    if (k.prev != -1) {c[k.prev].next = k.next;} else if (c[o].head == s) {c[o].head = k.next;}
    if (k.next != -1) {c[k.next].prev = k.prev;}
    k.prev = k.next = -1;
  }
  inline void link(int s, int obx, int oby)
  {
    Cell & k = c[s]; k.cx = obx; k.cy = oby;
    int o = slot(obx, oby);
    k.prev = -1; k.next = c[o].head;
    if (c[o].head != -1) {c[c[o].head].prev = s;}
    c[o].head = s;
  }
  inline void setClosest(int s, int obx, int oby, int d2) {unlink(s); link(s, obx, oby); c[s].d2 = d2;}

  // ---- queues and buffers (reused across calls) ----
  std::vector<QE> q; size_t qh = 0;
  std::vector<std::pair<int, int>> insq, delq;
  std::vector<int> deps, orphans;
  std::vector<std::pair<int, int>> leaving, entering;
  inline void push(int x, int y, int sx, int sy, int d2) {q.push_back({x, y, sx, sy, d2});}
  inline void pushXY(int x, int y, int d2) {push(x, y, mdx(x), mdy(y), d2);}

  // Occupancy change of a cell inside the window.
  void setOcc(int x, int y, uint8_t v)
  {
    int s = slot(x, y);
    if (c[s].occ == v) {return;}
    c[s].occ = v;
    if (v) {insq.push_back({x, y});} else {delq.push_back({x, y});}
  }

  // Compare the window with the stored occupancy and queue every difference (insert / delete).
  // Returns the number of changed cells. Cost O(Nx*Ny) with a tiny constant; callers time it separately.
  template<class F>
  int syncOccupancy(F occAt)
  {
    int changed = 0;
    for (int y = oy; y < oy + Ny; ++y) {
      Cell * row = &c[static_cast<size_t>(Nx) * mdy(y)];
      int sx = mdx(ox);
      for (int x = ox; x < ox + Nx; ++x) {
        const uint8_t v = occAt(x, y) ? 1 : 0;
        if (row[sx].occ != v) {setOcc(x, y, v); ++changed;}
        if (++sx == Nx) {sx = 0;}
      }
    }
    return changed;
  }

  // Best candidate obstacle among the 8 neighbours' closest obstacles. valid(nx,ny) filters neighbours,
  // okObs(qx,qy) filters candidate obstacles. Returns false if none within dmax.
  template<class V, class O>
  inline bool best(int x, int y, int sx, int sy, V valid, O okObs, int & bx, int & by, int & bd2) const
  {
    bd2 = INF2;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (!dx && !dy) {continue;}
        int nx = x + dx, ny = y + dy;
        if (!valid(nx, ny)) {continue;}
        const Cell & k = c[wrapx(sx + dx) + Nx * wrapy(sy + dy)];
        if (k.cx == UND) {continue;}
        if (!okObs(k.cx, k.cy)) {continue;}
        int d2 = sq(x, y, k.cx, k.cy);
        if (d2 < bd2) {bd2 = d2; bx = k.cx; by = k.cy;}
      }
    }
    return bd2 <= dmax2;
  }

  // Run the queued inserts, deletes and the propagation waves.
  void update()
  {
    auto inw = [&](int a, int b) {return inWin(a, b);};
    auto obsOk = [&](int qx, int qy) {return inWin(qx, qy) && c[slot(qx, qy)].occ;};
    for (auto & p : insq) {
      int s = slot(p.first, p.second);
      if (!c[s].occ) {continue;}
      setClosest(s, p.first, p.second, 0);
      pushXY(p.first, p.second, 0);
    }
    insq.clear();
    for (auto & p : delq) {
      int s = slot(p.first, p.second);
      if (c[s].occ) {continue;}
      deps.clear();
      for (int d = c[s].head; d != -1; d = c[d].next) {deps.push_back(d);}
      for (int d : deps) {Cell & k = c[d]; k.prev = k.next = -1; k.cx = k.cy = UND; k.d2 = INF2;}
      c[s].head = -1;
      for (int d : deps) {
        int sx = d % Nx, sy = d / Nx;
        int x = worldX(d), y = worldY(d);
        int bx = 0, by = 0, bd2;
        if (best(x, y, sx, sy, inw, obsOk, bx, by, bd2)) {link(d, bx, by); c[d].d2 = bd2; push(x, y, sx, sy, bd2);}
      }
    }
    delq.clear();
    while (qh < q.size()) {
      QE e = q[qh++];
      int s = e.sx + Nx * e.sy;
      Cell & k = c[s];
      if (e.d2 != k.d2 || k.cx == UND) {continue;}
      ++pops;
      if (pull_on_pop) {
        int bx = 0, by = 0, bd2;
        if (best(e.x, e.y, e.sx, e.sy, inw, obsOk, bx, by, bd2) && bd2 < k.d2) {
          setClosest(s, bx, by, bd2); push(e.x, e.y, e.sx, e.sy, bd2); continue;
        }
      }
      const int qx = k.cx, qy = k.cy;
      for (int dy = -1; dy <= 1; ++dy) {
        int ny = e.y + dy; if (ny < oy || ny >= oy + Ny) {continue;}
        int nsy = wrapy(e.sy + dy);
        for (int dx = -1; dx <= 1; ++dx) {
          if (!dx && !dy) {continue;}
          int nx = e.x + dx; if (nx < ox || nx >= ox + Nx) {continue;}
          int nd2 = sq(nx, ny, qx, qy);
          if (nd2 > dmax2) {continue;}
          int nsx = wrapx(e.sx + dx), ns = nsx + Nx * nsy;
          if (nd2 < c[ns].d2) {setClosest(ns, qx, qy, nd2); push(nx, ny, nsx, nsy, nd2);}
        }
      }
    }
    q.clear(); qh = 0;
  }

  // A \ B for two Nx x Ny rectangles with lower-left corners (ax,ay) and (bx,by): only the strip cells.
  void diff(int ax, int ay, int bx, int by, std::vector<std::pair<int, int>> & out) const
  {
    out.clear();
    const int ylo = std::min(ay + Ny, std::max(ay, by));
    const int yhi = std::max(ay, std::min(ay + Ny, by + Ny));
    for (int x = ax; x < ax + Nx; ++x) {
      if (x < bx || x >= bx + Nx) {
        for (int y = ay; y < ay + Ny; ++y) {out.push_back({x, y});}
      } else {
        for (int y = ay; y < ylo; ++y) {out.push_back({x, y});}
        for (int y = yhi; y < ay + Ny; ++y) {out.push_back({x, y});}
      }
    }
  }

  // Move the window to (nox, noy). occAt(x,y) = occupancy of world cell (x,y) in the NEW window,
  // read only for the cells that enter. Queues work; update() runs the waves. Returns entering cells.
  template<class F>
  int shift(int nox, int noy, F occAt)
  {
    last_full_rebuild = false;
    if (nox == ox && noy == oy) {last_entering = last_orphans = 0; return 0;}
    const int pox = ox, poy = oy;
    if (std::abs(nox - ox) >= Nx || std::abs(noy - oy) >= Ny) {
      fullReset(nox, noy, occAt);
      last_entering = Nx * Ny; last_orphans = 0; last_full_rebuild = true;
      return Nx * Ny;
    }
    diff(pox, poy, nox, noy, leaving);
    diff(nox, noy, pox, poy, entering);
    auto kept = [&](int x, int y) {return inRect(x, y, pox, poy, Nx, Ny) && inRect(x, y, nox, noy, Nx, Ny);};
    // 1a. unlink leaving non-obstacle cells
    for (auto & p : leaving) {
      int s = slot(p.first, p.second); const Cell & k = c[s];
      if (!(k.occ && k.cx == p.first && k.cy == p.second)) {unlink(s);}
    }
    // 1b. orphan the staying dependents of leaving obstacles
    orphans.clear();
    for (auto & p : leaving) {
      int s = slot(p.first, p.second);
      if (!c[s].occ) {continue;}
      for (int d = c[s].head; d != -1; ) {
        int nd = c[d].next; if (d != s) {orphans.push_back(d);}
        c[d].prev = c[d].next = -1; d = nd;
      }
      c[s].head = -1;
    }
    last_orphans = static_cast<int>(orphans.size()); last_entering = static_cast<int>(entering.size());
    for (int d : orphans) {c[d].cx = c[d].cy = UND; c[d].d2 = INF2;}
    // 1c. repair orphans from kept neighbours with kept obstacles
    auto keptObs = [&](int qx, int qy) {return kept(qx, qy) && c[slot(qx, qy)].occ;};
    for (int d : orphans) {
      int sx = d % Nx, sy = d / Nx;
      int x = pox + mdx(sx - pox), y = poy + mdy(sy - poy);
      int bx = 0, by = 0, bd2;
      if (best(x, y, sx, sy, kept, keptObs, bx, by, bd2)) {link(d, bx, by); c[d].d2 = bd2; push(x, y, sx, sy, bd2);}
    }
    // 2. wipe recycled slots
    for (auto & p : leaving) {c[slot(p.first, p.second)] = Cell{INF2, UND, UND, -1, -1, -1, 0};}
    ox = nox; oy = noy;
    // 3. insert entering obstacles
    for (auto & p : entering) {
      if (!occAt(p.first, p.second)) {continue;}
      int s = slot(p.first, p.second);
      c[s].occ = 1; link(s, p.first, p.second); c[s].d2 = 0; pushXY(p.first, p.second, 0);
    }
    // 4. pull for entering free cells from kept neighbours
    for (auto & p : entering) {
      int sx = mdx(p.first), sy = mdy(p.second), s = sx + Nx * sy;
      if (c[s].occ) {continue;}
      int bx = 0, by = 0, bd2;
      if (best(p.first, p.second, sx, sy, kept, keptObs, bx, by, bd2)) {
        link(s, bx, by); c[s].d2 = bd2; push(p.first, p.second, sx, sy, bd2);
      }
    }
    return static_cast<int>(entering.size());
  }

  // Throw everything away and queue every obstacle of the window at (nox, noy). Waves run in update().
  template<class F>
  void fullReset(int nox, int noy, F occAt)
  {
    std::fill(c.begin(), c.end(), Cell{INF2, UND, UND, -1, -1, -1, 0});
    q.clear(); qh = 0; insq.clear(); delq.clear();
    ox = nox; oy = noy;
    for (int y = oy; y < oy + Ny; ++y) {
      for (int x = ox; x < ox + Nx; ++x) {if (occAt(x, y)) {setOcc(x, y, 1);}}
    }
  }

  // Build from scratch (start-up, resize, reset): fullReset + waves.
  template<class F>
  void reset(int nox, int noy, F occAt) {fullReset(nox, noy, occAt); update(); last_full_rebuild = true;}

  // Debug: number of bookkeeping violations (lists, links, stored distances). 0 = consistent.
  long long selfCheck() const
  {
    long long bad = 0; const int S = Nx * Ny;
    std::vector<int> seen(S, 0);
    for (int o = 0; o < S; ++o) {
      int steps = 0;
      for (int m = c[o].head; m != -1; m = c[m].next) {
        if (++steps > S) {++bad; break;}
        if (c[m].cx != worldX(o) || c[m].cy != worldY(o)) {++bad;}
        if (c[m].next != -1 && c[c[m].next].prev != m) {++bad;}
        ++seen[m];
      }
    }
    for (int s = 0; s < S; ++s) {
      const Cell & k = c[s];
      if (k.occ && (k.cx != worldX(s) || k.cy != worldY(s) || k.d2 != 0)) {++bad;}
      if (k.cx == UND) {if (seen[s]) {++bad;} continue;}
      if (!inWin(k.cx, k.cy) || !c[slot(k.cx, k.cy)].occ) {++bad;}
      if (seen[s] != 1) {++bad;}
      if (k.d2 != sq(worldX(s), worldY(s), k.cx, k.cy)) {++bad;}
    }
    return bad;
  }
};

}  // namespace nav2_esdf
