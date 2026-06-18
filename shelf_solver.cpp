#include "shelf_solver.hpp"

#include "embedded_maps.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

namespace shelf {
    namespace {
        struct CovGroup {
            int n = 0;
            std::array<std::pair<int, int>, 8> off{};
        };

        struct Instance {
            int W = 0, H = 0;
            std::vector<std::uint8_t> isFree;
            int s = -1, t = -1;
            std::array<std::pair<int, int>, 4> dirs; // movement priority (high->low)
            std::vector<CovGroup> cov; // shelf rotations; score = best group
            Neighborhood nb = Neighborhood::N8;
            bool two_cell = false; // shelves are 1x2 dominoes (see header)
            std::vector<int> freeCells;
            std::vector<int> vars; // free \ {s,t}

            int id(int x, int y) const { return y * W + x; }
            int cx(int c) const { return c % W; }
            int cy(int c) const { return c / W; }
            bool inb(int x, int y) const { return x >= 0 && x < W && y >= 0 && y < H; }
        };

        constexpr std::array<std::pair<int, int>, 4> ORDER_NESW = {{{0, -1}, {1, 0}, {0, 1}, {-1, 0}}};

        constexpr std::array<std::pair<int, int>, 8> MOORE8 = {
            {{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}}
        };
        constexpr std::array<std::pair<int, int>, 4> ORTHO4 = {{{0, -1}, {1, 0}, {0, 1}, {-1, 0}}};

        std::vector<CovGroup> covFor(Neighborhood nb) {
            auto G = [](std::initializer_list<std::pair<int, int> > l) {
                CovGroup g;
                g.n = static_cast<int>(l.size());
                int i = 0;
                for (auto p: l) g.off[i++] = p;
                return g;
            };
            switch (nb) {
                case Neighborhood::N8:
                    return {G({{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}})};
                case Neighborhood::N4:
                    return {G({{0, -1}, {1, 0}, {0, 1}, {-1, 0}})};
                case Neighborhood::N2: // front+back: pick the better of the two axes
                    return {G({{0, -1}, {0, 1}}), G({{-1, 0}, {1, 0}})};
                case Neighborhood::N1: // front only: point the single face at the path
                    return {G({{0, -1}}), G({{0, 1}}), G({{-1, 0}}), G({{1, 0}})};
            }
            return {G({{0, -1}, {1, 0}, {0, 1}, {-1, 0}})};
        }

        struct ShelfScore {
            int faces = 0;
            int group = 0;
        };

        ShelfScore shelfBest(const Instance &I, const std::vector<std::uint8_t> &onP, int x, int y) {
            ShelfScore best;
            for (int gi = 0; gi < static_cast<int>(I.cov.size()); ++gi) {
                const CovGroup &g = I.cov[gi];
                int cnt = 0;
                for (int k = 0; k < g.n; ++k) {
                    int nx = x + g.off[k].first, ny = y + g.off[k].second;
                    if (I.inb(nx, ny) && onP[I.id(nx, ny)]) ++cnt;
                }
                if (cnt > best.faces) {
                    best.faces = cnt;
                    best.group = gi;
                }
            }
            return best;
        }

        std::optional<Instance> buildInstance(const std::vector<std::string> &g,
                                              Neighborhood nb = Neighborhood::N8,
                                              std::array<std::pair<int, int>, 4> dirs = ORDER_NESW,
                                              bool two_cell = false) {
            Instance I;
            I.H = static_cast<int>(g.size());
            I.W = 0;
            for (auto &r: g) I.W = std::max(I.W, static_cast<int>(r.size()));
            I.dirs = dirs;
            I.nb = nb;
            I.two_cell = two_cell;
            I.cov = covFor(nb);
            I.isFree.assign(I.W * I.H, 0);
            for (int y = 0; y < I.H; ++y) {
                for (int x = 0; x < static_cast<int>(g[y].size()); ++x) {
                    char ch = g[y][x];
                    int c = I.id(x, y);
                    if (ch == '#' || ch == ' ') continue;
                    I.isFree[c] = 1;
                    if (ch == 'S') I.s = c;
                    if (ch == 'T') I.t = c;
                }
            }
            if (I.s < 0 || I.t < 0) return std::nullopt;
            for (int c = 0; c < I.W * I.H; ++c) {
                if (I.isFree[c]) {
                    I.freeCells.push_back(c);
                    if (c != I.s && c != I.t) I.vars.push_back(c);
                }
            }
            return I;
        }

        template<class FDom, class FRb>
        int match2cell(const Instance &I, const std::vector<std::uint8_t> &shelf,
                       const std::vector<std::uint8_t> &onP, FDom &&onDom, FRb &&onRb) {
            const int N = I.W * I.H;
            std::vector<std::uint8_t> matched(N, 0);

            auto onPath = [&](int x, int y) { return I.inb(x, y) && onP[I.id(x, y)] ? 1 : 0; };

            auto weight = [&](int a, int b) {
                const int ax = I.cx(a), ay = I.cy(a);
                const int lx = I.cx(b) - ax, ly = I.cy(b) - ay; // long-axis step
                const int px = ly, py = lx; // perpendicular step
                const int bx = ax + lx, by = ay + ly;
                const int lpos = onPath(ax + px, ay + py) + onPath(bx + px, by + py); // one long face
                const int lneg = onPath(ax - px, ay - py) + onPath(bx - px, by - py); // opposite long face
                const int ends = onPath(ax - lx, ay - ly) + onPath(bx + lx, by + ly); // two short ends
                const int corn = onPath(ax - lx + px, ay - ly + py) + onPath(ax - lx - px, ay - ly - py) +
                                 onPath(bx + lx + px, by + ly + py) + onPath(bx + lx - px, by + ly - py);
                switch (I.nb) {
                    case Neighborhood::N1: return std::max(lpos, lneg);
                    case Neighborhood::N2: return lpos + lneg;
                    case Neighborhood::N4: return lpos + lneg + ends;
                    default: return lpos + lneg + ends + corn; // N8
                }
            };
            auto adjN8 = [&](int c) {
                const int cx = I.cx(c), cy = I.cy(c);
                int w = 0;
                for (auto [dx, dy]: MOORE8) {
                    const int nx = cx + dx, ny = cy + dy;
                    if (I.inb(nx, ny) && onP[I.id(nx, ny)]) ++w;
                }
                return w;
            };

            int score = 0;

            for (int pass = 0; pass < 2; ++pass) {
                for (int c: I.freeCells) {
                    if (!shelf[c] || matched[c]) continue;
                    if (pass == 0 && adjN8(c) == 0) continue;
                    const int cx = I.cx(c), cy = I.cy(c);
                    int bestN = -1, bestW = -1;
                    for (auto [dx, dy]: ORTHO4) {
                        const int nx = cx + dx, ny = cy + dy;
                        if (!I.inb(nx, ny)) continue;
                        const int n = I.id(nx, ny);
                        if (!shelf[n] || matched[n]) continue;
                        const int w = weight(c, n);
                        if (w > bestW) {
                            bestW = w;
                            bestN = n;
                        }
                    }
                    if (bestN >= 0) {
                        matched[c] = matched[bestN] = 1;
                        score += bestW;
                        onDom(c, bestN, bestW);
                    }
                }
            }
            for (int c: I.freeCells)
                if (shelf[c] && !matched[c]) onRb(c);
            return score;
        }

        int score2cell(const Instance &I, const std::vector<std::uint8_t> &shelf,
                       const std::vector<std::uint8_t> &onP) {
            return match2cell(I, shelf, onP, [](int, int, int) {
                              }, [](int) {
                              });
        }

        struct Domino {
            int a, b, score;
        };

        struct Tiling {
            int score = 0;
            std::vector<Domino> dominoes;
            std::vector<int> roadblocks;
        };

        Tiling tile2cell(const Instance &I, const std::vector<std::uint8_t> &shelf,
                         const std::vector<std::uint8_t> &onP) {
            Tiling t;
            t.score = match2cell(
                I, shelf, onP,
                [&](int a, int b, int w) { t.dominoes.push_back({a, b, w}); },
                [&](int c) { t.roadblocks.push_back(c); });
            return t;
        }

        int coverageOf(const Instance &I, const std::vector<std::uint8_t> &shelf, const std::vector<int> &path) {
            std::vector<std::uint8_t> onP(I.W * I.H, 0);
            for (int c: path) onP[c] = 1;
            if (I.two_cell) return score2cell(I, shelf, onP);
            int score = 0;
            for (int c: I.freeCells) {
                if (!shelf[c]) continue;
                score += shelfBest(I, onP, I.cx(c), I.cy(c)).faces;
            }
            return score;
        }

        std::optional<std::vector<int> > bfsPath(const Instance &I, const std::vector<std::uint8_t> &walk) {
            const int N = I.W * I.H;
            if (!walk[I.s]) return std::nullopt;
            std::vector<int> par(N, -1), dist(N, -1);
            std::queue<int> q;
            dist[I.s] = 0;
            q.push(I.s);
            while (!q.empty()) {
                int u = q.front();
                q.pop();
                if (u == I.t) break;
                int ux = I.cx(u), uy = I.cy(u);
                for (auto [dx, dy]: I.dirs) {
                    int nx = ux + dx, ny = uy + dy;
                    if (!I.inb(nx, ny)) continue;
                    int nid = I.id(nx, ny);
                    if (walk[nid] && dist[nid] < 0) {
                        dist[nid] = dist[u] + 1;
                        par[nid] = u;
                        q.push(nid);
                    }
                }
            }
            if (dist[I.t] < 0) return std::nullopt;
            std::vector<int> path;
            for (int c = I.t; c != -1; c = par[c]) path.push_back(c);
            std::reverse(path.begin(), path.end());
            return path;
        }

        struct EvalResult {
            bool feasible = false;
            int coverage = 0;
            std::vector<int> path;
        };

        EvalResult evaluate(const Instance &I, const std::vector<std::uint8_t> &shelf) {
            std::vector<std::uint8_t> walk(I.W * I.H, 0);
            for (int c: I.freeCells) walk[c] = shelf[c] ? 0 : 1;
            walk[I.s] = 1;
            walk[I.t] = 1;
            auto p = bfsPath(I, walk);
            if (!p) return {false, 0, {}};
            return {true, coverageOf(I, shelf, *p), std::move(*p)};
        }

        struct Solution {
            std::vector<std::uint8_t> shelf;
            EvalResult eval;
        };

        std::vector<std::uint8_t> corridorShelf(const Instance &I, const std::vector<int> &path) {
            std::vector<std::uint8_t> onP(I.W * I.H, 0);
            for (int c: path) onP[c] = 1;
            std::vector<std::uint8_t> shelf(I.W * I.H, 0);
            for (int c: I.freeCells) shelf[c] = onP[c] ? 0 : 1;
            return shelf;
        }

        std::optional<std::vector<int> > randDFS(const Instance &I, int start, int goal,
                                                 const std::vector<std::uint8_t> &allowed, std::mt19937 &rng) {
            const int N = I.W * I.H;
            if (!allowed[start] || !allowed[goal]) return std::nullopt;
            std::vector<std::uint8_t> vis(N, 0);
            std::vector<int> stk;
            std::vector<std::array<int, 4> > ord;
            std::vector<int> cur;
            auto shuffled = [&]() {
                std::array<int, 4> a = {0, 1, 2, 3};
                std::shuffle(a.begin(), a.end(), rng);
                return a;
            };
            stk.push_back(start);
            ord.push_back(shuffled());
            cur.push_back(0);
            vis[start] = 1;
            while (!stk.empty()) {
                int u = stk.back();
                if (u == goal) return stk;
                int ux = I.cx(u), uy = I.cy(u);
                bool adv = false;
                while (cur.back() < 4) {
                    int d = ord.back()[cur.back()];
                    cur.back()++;
                    auto [dx, dy] = I.dirs[d];
                    int nx = ux + dx, ny = uy + dy;
                    if (!I.inb(nx, ny)) continue;
                    int nid = I.id(nx, ny);
                    if (allowed[nid] && !vis[nid]) {
                        vis[nid] = 1;
                        stk.push_back(nid);
                        ord.push_back(shuffled());
                        cur.push_back(0);
                        adv = true;
                        break;
                    }
                }
                if (!adv) {
                    stk.pop_back();
                    ord.pop_back();
                    cur.pop_back();
                }
            }
            return std::nullopt;
        }

        std::optional<std::vector<int> > reroute(const Instance &I, const std::vector<int> &path, std::mt19937 &rng) {
            const int L = static_cast<int>(path.size());
            if (L < 2) return std::nullopt;
            std::uniform_int_distribution<int> di(0, L - 2);
            int i = di(rng);
            std::uniform_int_distribution<int> dj(i + 1, L - 1);
            int j = dj(rng);
            int A = path[i], B = path[j];

            std::vector<std::uint8_t> allowed = I.isFree;
            for (int c: path) allowed[c] = 0;
            for (int k = i + 1; k < j; ++k) allowed[path[k]] = 1;
            allowed[A] = 1;
            allowed[B] = 1;

            auto seg = randDFS(I, A, B, allowed, rng);
            if (!seg) return std::nullopt;

            std::vector<int> np;
            np.reserve(i + seg->size() + (L - 1 - j));
            for (int k = 0; k < i; ++k) np.push_back(path[k]);
            for (int c: *seg) np.push_back(c);
            for (int k = j + 1; k < L; ++k) np.push_back(path[k]);
            return np;
        }

        struct OptResult {
            Solution sol;
            std::vector<int> intended;
        };

        OptResult solvePathSA(const Instance &I, std::vector<int> initPath,
                              std::uint64_t iters, double T0, double Tmin, std::mt19937 &rng,
                              std::atomic<std::uint64_t> *progress = nullptr,
                              std::atomic<int> *bestShared = nullptr) {
            auto realise = [&](const std::vector<int> &p) {
                std::vector<std::uint8_t> shelf = corridorShelf(I, p);
                return Solution{shelf, evaluate(I, shelf)};
            };
            auto publishBest = [&](int cov) {
                if (!bestShared) return;
                int cur = bestShared->load(std::memory_order_relaxed);
                while (cov > cur && !bestShared->compare_exchange_weak(cur, cov, std::memory_order_relaxed)) {
                }
            };
            std::vector<int> curPath = std::move(initPath);
            Solution cur = realise(curPath);
            Solution best = cur;
            std::vector<int> bestPath = curPath;
            publishBest(best.eval.coverage);

            std::uniform_real_distribution<double> unit(0.0, 1.0);
            double T = T0;
            const double alpha = std::pow(Tmin / T0, 1.0 / static_cast<double>(std::max<std::uint64_t>(1, iters)));

            std::uint64_t sinceFlush = 0;
            for (std::uint64_t step = 0; step < iters; ++step) {
                auto np = reroute(I, curPath, rng);
                if (np) {
                    Solution ns = realise(*np);
                    double delta = static_cast<double>(ns.eval.coverage) - static_cast<double>(cur.eval.coverage);
                    if (delta >= 0.0 || unit(rng) < std::exp(delta / T)) {
                        curPath = *np;
                        cur = ns;
                        if (ns.eval.coverage > best.eval.coverage) {
                            best = ns;
                            bestPath = curPath;
                            publishBest(best.eval.coverage);
                        }
                    }
                }
                T *= alpha;
                if (progress && ++sinceFlush >= 1024) {
                    progress->fetch_add(sinceFlush, std::memory_order_relaxed);
                    sinceFlush = 0;
                }
            }
            if (progress && sinceFlush) progress->fetch_add(sinceFlush, std::memory_order_relaxed);
            return {best, bestPath};
        }

        using Cell = std::pair<int, int>;

        struct MapModel {
            std::set<Cell> floor; // walkable floor accumulated up to `level`
            std::set<Cell> high_platform; // fixed obstacles (display platforms)
            Cell entrance{-1, -1};
            Cell exit{-1, -1};
            int max_level = 0;
        };

        std::vector<Cell> regionCells(const nlohmann::json &p) {
            std::vector<Cell> cells;
            if (p.size() < 2) return cells;
            const int x = p[0].get<int>();
            const int y = p[1].get<int>();
#ifdef SHELF_RECT_REGIONS
            const int w = p.size() >= 4 ? p[2].get<int>() : 1;
            const int h = p.size() >= 4 ? p[3].get<int>() : 1;
#else
            const int w = 1, h = 1;
#endif
            cells.reserve(static_cast<std::size_t>(std::max(0, w)) * std::max(0, h));
            for (int dy = 0; dy < h; ++dy)
                for (int dx = 0; dx < w; ++dx)
                    cells.emplace_back(x + dx, y + dy);
            return cells;
        }

        std::optional<MapModel> parseMapJson(const nlohmann::json &j, int level, std::string &err) {

            MapModel m;
            if (j.contains("highPlatformArr")) {
                for (const auto &c: j["highPlatformArr"]) {
                    if (c.contains("x") && c.contains("y"))
                        m.high_platform.emplace(c["x"].get<int>(), c["y"].get<int>());
                }
            }

            if (!j.contains("data")) {
                err = "map has no \"data\" array";
                return std::nullopt;
            }
            for (const auto &e: j["data"]) {
                int unlock = e.contains("nUnlockPraiseLevel") ? e["nUnlockPraiseLevel"].get<int>() : 0;
                m.max_level = std::max(m.max_level, unlock);
                if (unlock > level) continue; // not yet unlocked at this level
                if (!e.contains("CommandList")) continue;
                for (const auto &cmd: e["CommandList"]) {
                    int type = cmd["type"].get<int>();
                    const auto &p = cmd["Params"];
                    switch (type) {
                        case 101: // floor tile
                            for (const Cell &cell: regionCells(p)) m.floor.insert(cell);
                            break;
                        case 201: // high platform region
                            for (const Cell &cell: regionCells(p)) m.high_platform.insert(cell);
                            break;
                        case 301: // entrance -> inner cell
                            m.entrance = (p.size() >= 4)
                                             ? Cell{p[2].get<int>(), p[3].get<int>()}
                                             : Cell{p[0].get<int>(), p[1].get<int>()};
                            break;
                        case 302: // exit -> inner cell
                            m.exit = (p.size() >= 4)
                                         ? Cell{p[2].get<int>(), p[3].get<int>()}
                                         : Cell{p[0].get<int>(), p[1].get<int>()};
                            break;
                        default:
                            break;
                    }
                }
            }
            return m;
        }

        std::optional<MapModel> loadMapJson(std::string_view text, int level, std::string &err) {
            nlohmann::json j;
            try {
                j = nlohmann::json::parse(text.begin(), text.end());
            } catch (const std::exception &e) {
                err = std::string("JSON parse error: ") + e.what();
                return std::nullopt;
            }
            return parseMapJson(j, level, err);
        }

        std::optional<MapModel> loadMapFile(const std::string &path, int level, std::string &err) {
            std::ifstream in(path);
            if (!in) {
                err = "cannot open map file: " + path;
                return std::nullopt;
            }
            nlohmann::json j;
            try {
                in >> j;
            } catch (const std::exception &e) {
                err = std::string("JSON parse error: ") + e.what();
                return std::nullopt;
            }
            return parseMapJson(j, level, err);
        }

        std::string mapFilePath(const std::string &maps_dir, int map_index) {
            const std::string scene_path = maps_dir + "/" + std::to_string(map_index) + "/buildConstData.json";
            std::ifstream scene(scene_path);
            if (scene) return scene_path;
            return maps_dir + "/m" + std::to_string(map_index) + ".json";
        }

        std::string renderSolution(const Instance &I, const Solution &sol, int ox, int oy) {
            std::ostringstream os;
            std::vector<std::uint8_t> onP(I.W * I.H, 0);
            for (int c: sol.eval.path) onP[c] = 1;

            std::vector<char> mark(I.W * I.H, 0);
            if (I.two_cell) {
                Tiling t = tile2cell(I, sol.shelf, onP);
                const bool directional = (I.nb == Neighborhood::N1 || I.nb == Neighborhood::N2);
                auto facingGlyph = [&](const Domino &d) -> char {
                    const int ax = I.cx(d.a), ay = I.cy(d.a);
                    const int lx = I.cx(d.b) - ax, ly = I.cy(d.b) - ay;  // long axis a->b
                    const int px = ly, py = lx;                          // perpendicular
                    const int bx = ax + lx, by = ay + ly;
                    auto on = [&](int x, int y) { return I.inb(x, y) && onP[I.id(x, y)] ? 1 : 0; };
                    const int lpos = on(ax + px, ay + py) + on(bx + px, by + py);
                    const int lneg = on(ax - px, ay - py) + on(bx - px, by - py);
                    if (lpos + lneg == 0) return 'x';                    // scores 0 -> wasted
                    if (I.nb == Neighborhood::N2) return px != 0 ? '-' : '|';
                    const int dx = lpos >= lneg ? px : -px;             // toward the better face
                    const int dy = lpos >= lneg ? py : -py;
                    if (dx > 0) return '>';
                    if (dx < 0) return '<';
                    return dy > 0 ? 'v' : '^';
                };
                for (const Domino &d: t.dominoes) {
                    const char g = directional
                                       ? facingGlyph(d)
                                       : (d.score > 0 ? static_cast<char>('0' + std::min(d.score, 9)) : 'x');
                    mark[d.a] = g;
                    mark[d.b] = g;
                }
                for (int c: t.roadblocks) mark[c] = 'o';
            } else {
                for (int c: I.freeCells) {
                    if (!sol.shelf[c]) continue;
                    ShelfScore sb = shelfBest(I, onP, I.cx(c), I.cy(c));
                    if (sb.faces == 0) {
                        mark[c] = 'x';
                        continue;
                    }
                    switch (I.nb) {
                        case Neighborhood::N1: mark[c] = "^v<>"[sb.group];
                            break; // front faces the path
                        case Neighborhood::N2: mark[c] = "|-"[sb.group];
                            break; // front+back axis
                        default: mark[c] = static_cast<char>('0' + sb.faces);
                            break;
                    }
                }
            }
            for (int y = 0; y < I.H; ++y) {
                os << "  ";
                for (int x = 0; x < I.W; ++x) {
                    int c = I.id(x, y);
                    char ch;
                    if (!I.isFree[c]) ch = '#';
                    else if (c == I.s) ch = 'S';
                    else if (c == I.t) ch = 'T';
                    else if (sol.shelf[c]) ch = mark[c];
                    else if (onP[c]) ch = '*';
                    else ch = '.';
                    os << ch;
                }
                os << '\n';
            }
            const char *glyphs;
            if (I.two_cell) {
                glyphs = (I.nb == Neighborhood::N1)
                             ? "^v<> 2-cell shelf front (long face)  o roadblock"
                             : (I.nb == Neighborhood::N2)
                                   ? "|- 2-cell shelf long-face axis  o roadblock"
                                   : "1-9 2-cell shelf score (both halves)  o roadblock";
            } else {
                glyphs = (I.nb == Neighborhood::N1)
                             ? "^v<> shelf front facing"
                             : (I.nb == Neighborhood::N2)
                                   ? "|- shelf front/back axis"
                                   : "1-8 shelf faces seen";
            }
            os << "  legend: # obstacle  S/T endpoints  * customer path  "
                    << glyphs << "  x wasted shelf  . unused floor\n";
            std::string cov;
            if (I.two_cell) {
                switch (I.nb) {
                    case Neighborhood::N1: cov = "2-cell N1 (best long face)";
                        break;
                    case Neighborhood::N2: cov = "2-cell N2 (both long faces)";
                        break;
                    case Neighborhood::N4: cov = "2-cell N4 (long + short faces)";
                        break;
                    default: cov = "2-cell N8 (full perimeter)";
                        break;
                }
            } else {
                switch (I.nb) {
                    case Neighborhood::N4: cov = "N4 (orthogonal)";
                        break;
                    case Neighborhood::N2: cov = "N2 (front+back, best axis)";
                        break;
                    case Neighborhood::N1: cov = "N1 (front only, best dir)";
                        break;
                    default: cov = "N8 (Moore)";
                        break;
                }
            }
            os << "  coverage = " << cov
                    << "   origin (map coords) = (" << ox << ", " << oy << ")\n";
            return os.str();
        }

        SolveResult solveMapModel(const MapModel &m, int level, const SolveOptions &opt) {
            SolveResult R;
            R.level = level;

            if (m.entrance.first < 0 || m.exit.first < 0) {
                R.error = "map is missing an entrance (301) or exit (302)";
                return R;
            }

            std::set<Cell> walk = m.floor;
            if (opt.exclude_high_platform) {
                for (const auto &c: m.high_platform) walk.erase(c);
            }
            walk.insert(m.entrance);
            walk.insert(m.exit);

            if (walk.size() < 2) {
                R.error = "not enough walkable tiles at level " + std::to_string(level);
                return R;
            }

            int minx = INT32_MAX, miny = INT32_MAX, maxx = INT32_MIN, maxy = INT32_MIN;
            for (const auto &[x, y]: walk) {
                minx = std::min(minx, x);
                miny = std::min(miny, y);
                maxx = std::max(maxx, x);
                maxy = std::max(maxy, y);
            }
            const int ox = minx, oy = miny;
            const int W = maxx - minx + 1, H = maxy - miny + 1;

            std::vector<std::string> grid(H, std::string(W, '#'));
            for (const auto &[x, y]: walk) grid[y - oy][x - ox] = '.';
            grid[m.entrance.second - oy][m.entrance.first - ox] = 'S';
            grid[m.exit.second - oy][m.exit.first - ox] = 'T';

            auto inst = buildInstance(grid, opt.neighborhood, ORDER_NESW, opt.two_cell);
            if (!inst) {
                R.error = "internal: failed to place S/T on grid";
                return R;
            }
            const Instance &I = *inst;

            R.ok = true;
            R.walkable = static_cast<int>(I.freeCells.size());
            R.flippable = static_cast<int>(I.vars.size());
            R.entrance = m.entrance;
            R.exit = m.exit;

            if (!bfsPath(I, I.isFree)) {
                R.feasible = false;
                R.render = renderSolution(I, Solution{corridorShelf(I, {}), {}}, ox, oy);
                return R;
            }

            const std::uint64_t iters =
                    opt.iters_base + opt.iters_per_cell * static_cast<std::uint64_t>(I.freeCells.size());
            const int restarts = std::max(1, opt.restarts);

            std::atomic<std::uint64_t> done{0};
            std::atomic<int> bestAtomic{-1};
            const std::uint64_t total = iters * static_cast<std::uint64_t>(restarts);
            std::atomic<std::uint64_t> *pDone = opt.on_progress ? &done : nullptr;
            std::atomic<int> *pBest = opt.on_progress ? &bestAtomic : nullptr;

            std::vector<OptResult> results(restarts);
            auto runRestart = [&](int r) {
                std::mt19937 rng(opt.seed + static_cast<std::uint32_t>(r) * 0x9E3779B9u);
                std::optional<std::vector<int> > init =
                        (r == 0) ? bfsPath(I, I.isFree) : randDFS(I, I.s, I.t, I.isFree, rng);
                if (!init) {
                    results[r].sol.eval.coverage = -1;
                    if (pDone) pDone->fetch_add(iters, std::memory_order_relaxed); // keep `total` honest
                    return;
                }
                results[r] = solvePathSA(I, *init, iters, opt.t0, opt.t_min, rng, pDone, pBest);
            };

            std::atomic<bool> monitoring{static_cast<bool>(opt.on_progress)};
            std::thread monitor;
            if (opt.on_progress) {
                monitor = std::thread([&] {
                    while (monitoring.load(std::memory_order_relaxed)) {
                        SolveProgress p;
                        p.fraction = total
                                             ? std::min(1.0,
                                                        static_cast<double>(done.load(std::memory_order_relaxed)) /
                                                                static_cast<double>(total))
                                             : 0.0;
                        p.best = bestAtomic.load(std::memory_order_relaxed);
                        opt.on_progress(p);
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                });
            }

            unsigned hw = opt.threads ? opt.threads : std::thread::hardware_concurrency();
            unsigned nworkers = std::max(1u, std::min<unsigned>(hw ? hw : 1u, static_cast<unsigned>(restarts)));
            if (nworkers == 1) {
                for (int r = 0; r < restarts; ++r) runRestart(r);
            } else {
                std::atomic<int> next{0};
                std::vector<std::thread> pool;
                pool.reserve(nworkers);
                for (unsigned w = 0; w < nworkers; ++w) {
                    pool.emplace_back([&] {
                        for (int r = next.fetch_add(1); r < restarts; r = next.fetch_add(1)) runRestart(r);
                    });
                }
                for (auto &th: pool) th.join();
            }

            if (monitor.joinable()) {
                monitoring.store(false, std::memory_order_relaxed);
                monitor.join();
            }
            if (opt.on_progress) opt.on_progress(SolveProgress{1.0, bestAtomic.load(std::memory_order_relaxed)});

            OptResult best;
            best.sol.eval.coverage = -1;
            for (auto &res: results) {
                if (res.sol.eval.coverage > best.sol.eval.coverage) best = std::move(res);
            }
            if (best.sol.eval.coverage < 0) {
                R.feasible = false;
                return R;
            }

            R.feasible = best.sol.eval.feasible;
            R.coverage = best.sol.eval.coverage;
            R.path_length = static_cast<int>(best.sol.eval.path.size());
            for (int c: best.sol.eval.path) R.path.emplace_back(I.cx(c) + ox, I.cy(c) + oy);
            std::vector<std::uint8_t> onP(I.W * I.H, 0);
            for (int pc: best.sol.eval.path) onP[pc] = 1;
            if (I.two_cell) {
                Tiling t = tile2cell(I, best.sol.shelf, onP);
                auto mc = [&](int c) { return std::pair<int, int>{I.cx(c) + ox, I.cy(c) + oy}; };
                for (const Domino &d: t.dominoes) {
                    std::array<std::pair<int, int>, 2> cells{mc(d.a), mc(d.b)};
                    R.dominoes.push_back(cells);
                    if (d.score > 0) {
                        R.counted_shelves.push_back(cells[0]);
                        R.counted_shelves.push_back(cells[1]);
                    }
                }
                for (int c: t.roadblocks) R.roadblocks.push_back(mc(c));
            } else {
                for (int c: I.freeCells) {
                    if (!best.sol.shelf[c]) continue;
                    int x = I.cx(c), y = I.cy(c);
                    if (shelfBest(I, onP, x, y).faces > 0) R.counted_shelves.emplace_back(x + ox, y + oy);
                }
            }
            R.render = renderSolution(I, best.sol, ox, oy);
            return R;
        }
    } // namespace

    SolveResult solveMapJson(std::string_view map_json, int level, const SolveOptions &opt) {
        SolveResult R;
        R.level = level;

        std::string err;
        auto mm = loadMapJson(map_json, level, err);
        if (!mm) {
            R.error = err;
            return R;
        }
        return solveMapModel(*mm, level, opt);
    }

    SolveResult solveMapFile(const std::string &map_path, int level, const SolveOptions &opt) {
        SolveResult R;
        R.level = level;

        std::string err;
        auto mm = loadMapFile(map_path, level, err);
        if (!mm) {
            R.error = err;
            return R;
        }
        return solveMapModel(*mm, level, opt);
    }

    SolveResult solveMap(int map_index, int level, const SolveOptions &opt) {
        if (opt.maps_dir == "Scene") {
            const std::string_view embedded = embeddedMapJson(map_index);
            if (!embedded.empty()) return solveMapJson(embedded, level, opt);
        }

        std::string path = mapFilePath(opt.maps_dir, map_index);
        return solveMapFile(path, level, opt);
    }
}
