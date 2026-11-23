// Standalone self-test of RingEsdf (no ROS): checks the non-square port against brute force,
// using exactly the protocol the costmap layer uses: shift(), syncOccupancy(), update().
//
//   static world, moving window  -> distances must equal brute force EXACTLY (squared integers)
//   moving obstacles             -> FIESTA approximation allowed: max error <= 0.1 cells, no finite/inf mismatch
//   every step                   -> selfCheck() == 0 (lists and links consistent), jumps >= N cells rebuild
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
#include "nav2_esdf/ring_esdf.hpp"

using nav2_esdf::RingEsdf;

struct World
{
  int W, H;
  std::vector<uint8_t> g;
  World(int w, int h) : W(w), H(h), g(static_cast<size_t>(w) * h, 0) {}
  uint8_t at(int x, int y) const {return (x < 0 || y < 0 || x >= W || y >= H) ? 0 : g[static_cast<size_t>(y) * W + x];}
  void set(int x, int y, uint8_t v) {if (x >= 0 && y >= 0 && x < W && y < H) {g[static_cast<size_t>(y) * W + x] = v;}}
};

static void clutter(World & w, std::mt19937 & r)
{
  auto U = [&](int a, int b) {return std::uniform_int_distribution<int>(a, b)(r);};
  for (int i = 0; i < 60; ++i) {  // walls
    int L = U(30, 160), x = U(0, w.W - 1), y = U(0, w.H - 1);
    bool horiz = U(0, 1);
    for (int k = 0; k < L; ++k) {for (int t = 0; t < 2; ++t) {horiz ? w.set(x + k, y + t, 1) : w.set(x + t, y + k, 1);}}
  }
  for (int i = 0; i < 200; ++i) {  // boxes and posts
    int bw = U(2, 12), bh = U(2, 12), x = U(0, w.W - 1), y = U(0, w.H - 1);
    for (int a = 0; a < bw; ++a) {for (int b = 0; b < bh; ++b) {w.set(x + a, y + b, 1);}}
  }
}

// brute force squared distance of every window cell (INF2 beyond dmax2)
static void brute(const World & w, int ox, int oy, int Nx, int Ny, int dmax2, std::vector<int> & out)
{
  std::vector<std::pair<int, int>> obs;
  for (int y = oy; y < oy + Ny; ++y) {for (int x = ox; x < ox + Nx; ++x) {if (w.at(x, y)) {obs.push_back({x, y});}}}
  out.assign(static_cast<size_t>(Nx) * Ny, RingEsdf::INF2);
  for (int y = 0; y < Ny; ++y) {
    for (int x = 0; x < Nx; ++x) {
      int best = RingEsdf::INF2;
      for (auto & o : obs) {
        int dx = ox + x - o.first, dy = oy + y - o.second, d2 = dx * dx + dy * dy;
        if (d2 < best) {best = d2;}
      }
      if (best <= dmax2) {out[static_cast<size_t>(y) * Nx + x] = best;}
    }
  }
}

struct Stats {double max_err = 0; long long inf_mismatch = 0, struct_bad = 0, wrong_exact = 0; int steps = 0, rebuilds = 0;};

static Stats run(int Nx, int Ny, float dmax_cells, bool dynamic, int steps, unsigned seed, int max_step)
{
  std::mt19937 r(seed);
  auto U = [&](int a, int b) {return std::uniform_int_distribution<int>(a, b)(r);};
  World w(700, 700); clutter(w, r);
  RingEsdf f(Nx, Ny, dmax_cells);
  int ox = 300, oy = 300;
  auto occ = [&](int x, int y) {return w.at(x, y);};
  f.reset(ox, oy, occ);
  Stats st; std::vector<int> ref;
  for (int s = 0; s < steps; ++s) {
    int dx = U(-max_step, max_step), dy = U(-max_step, max_step);
    if (U(0, 40) == 0) {dx = U(Nx, Nx + 15) * (U(0, 1) ? 1 : -1);}     // occasional jump -> full rebuild
    ox = std::max(5, std::min(700 - Nx - 5, ox + dx)); oy = std::max(5, std::min(700 - Ny - 5, oy + dy));
    if (dynamic) {                                                      // moving blobs and flicker near the window
      for (int k = 0; k < 8; ++k) {
        int cx = ox + U(0, Nx - 1), cy = oy + U(0, Ny - 1);
        w.set(cx, cy, U(0, 1));
        w.set(cx + 1, cy, w.at(cx, cy));
      }
    }
    f.shift(ox, oy, occ);
    f.syncOccupancy(occ);
    f.update();
    if (f.last_full_rebuild) {++st.rebuilds;}
    brute(w, ox, oy, Nx, Ny, f.dmax2, ref);
    for (int y = 0; y < Ny; ++y) {
      for (int x = 0; x < Nx; ++x) {
        int a = f.d2At(ox + x, oy + y), b = ref[static_cast<size_t>(y) * Nx + x];
        bool fa = a < RingEsdf::INF2, fb = b < RingEsdf::INF2;
        if (fa != fb) {++st.inf_mismatch; continue;}
        if (fa) {
          st.max_err = std::max(st.max_err, std::fabs(std::sqrt(double(a)) - std::sqrt(double(b))));
          if (a != b) {++st.wrong_exact;}
        }
      }
    }
    st.struct_bad += f.selfCheck();
    ++st.steps;
  }
  return st;
}

int main()
{
  struct Case {int nx, ny; float dmax; bool dyn; int step; const char * name;};
  const Case cases[] = {
    {60, 60, 40.f, false, 4, "square 60x60, static, dmax 2 m"},
    {60, 40, 40.f, false, 4, "non-square 60x40, static, dmax 2 m"},
    {37, 53, 40.f, false, 8, "non-square 37x53 (odd), static, fast (8 cells/step)"},
    {80, 30, 1e6f, false, 3, "non-square 80x30, static, uncapped"},
    {60, 40, 40.f, true, 4, "non-square 60x40, dynamic obstacles"},
    {53, 37, 20.f, true, 6, "non-square 53x37, dynamic obstacles, dmax 1 m"},
  };
  int failed = 0;
  for (const auto & c : cases) {
    Stats s = run(c.nx, c.ny, c.dmax, c.dyn, 400, 7, c.step);
    bool ok = s.struct_bad == 0 && s.inf_mismatch == 0 && (c.dyn ? s.max_err <= 0.1 : (s.wrong_exact == 0));
    std::printf("%s  %-52s steps %d rebuilds %2d | max err %.4f cells | not-exact cells %lld | inf-mismatch %lld | struct violations %lld\n",
      ok ? "PASS" : "FAIL", c.name, s.steps, s.rebuilds, s.max_err, s.wrong_exact, s.inf_mismatch, s.struct_bad);
    failed += !ok;
  }
  std::printf(failed ? "\nSELFTEST FAILED (%d)\n" : "\nSELFTEST OK\n", failed);
  return failed ? 1 : 0;
}
