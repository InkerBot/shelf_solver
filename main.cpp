#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "shelf_solver.hpp"

namespace {
    using Cell = std::pair<int, int>;

    bool endsWith(const std::string &s, const std::string &suffix) {
        return s.size() >= suffix.size() &&
               s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
    }

    std::string num(double v) {
        std::ostringstream o;
        o << std::fixed << std::setprecision(2) << v;
        return o.str();
    }

    struct CashierModel {
        Cell entrance{-1, -1}; // config 301 inner: customer enters here (S)
        Cell real_exit{-1, -1}; // config 302 inner: the exit door (made solid)
        Cell cashier_access{-1, -1}; // config 303: the path's goal (solver T)
        std::vector<Cell> counter; // the real 2-cell cashier counter (solid)
    };

    Cell innerCell(const nlohmann::json &p) {
        if (p.size() >= 4) return {p[2].get<int>(), p[3].get<int>()};
        return {p[0].get<int>(), p[1].get<int>()};
    }

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

    void emitFloorCells(const nlohmann::json &cmd, const std::set<Cell> &obstacles,
                        nlohmann::json &kept) {
        for (const Cell &cell: regionCells(cmd.at("Params"))) {
            if (obstacles.count(cell)) continue;
            nlohmann::json fc = cmd;
            fc["Params"] = nlohmann::json::array({cell.first, cell.second});
            kept.push_back(std::move(fc));
        }
    }

    std::optional<CashierModel> buildCashierMap(const std::string &src, const std::string &dst) {
        std::ifstream in(src);
        if (!in) return std::nullopt;
        nlohmann::json j;
        try { in >> j; } catch (...) { return std::nullopt; }
        if (!j.contains("data")) return std::nullopt;

        CashierModel m;
        for (const auto &e: j["data"]) {
            if (!e.contains("CommandList")) continue;
            for (const auto &cmd: e["CommandList"]) {
                const int t = cmd.value("type", 0);
                const auto &p = cmd["Params"];
                if (t == 301) m.entrance = innerCell(p);
                else if (t == 302) m.real_exit = innerCell(p);
                else if (t == 303 && p.size() >= 2)
                    m.cashier_access = {p[0].get<int>(), p[1].get<int>()};
            }
        }
        if (m.entrance.first < 0 || m.real_exit.first < 0 || m.cashier_access.first < 0)
            return std::nullopt;

        m.counter = {
            {m.cashier_access.first + 1, m.cashier_access.second},
            {m.real_exit.first + 1, m.real_exit.second}
        };
        std::set<Cell> obstacles(m.counter.begin(), m.counter.end());
        obstacles.insert(m.real_exit);

        for (auto &e: j["data"]) {
            if (!e.contains("CommandList")) continue;
            nlohmann::json kept = nlohmann::json::array();
            for (auto &cmd: e["CommandList"]) {
                const int t = cmd.value("type", 0);
                auto &p = cmd["Params"];
                if (t == 101) {
                    // expand floor region, dropping cells under obstacles
                    emitFloorCells(cmd, obstacles, kept);
                    continue;
                }
                if (t == 302) {
                    if (p.size() >= 4) {
                        p[2] = m.cashier_access.first;
                        p[3] = m.cashier_access.second;
                    } else {
                        p[0] = m.cashier_access.first;
                        p[1] = m.cashier_access.second;
                    }
                }
                kept.push_back(cmd);
            }
            e["CommandList"] = std::move(kept);
        }

        std::ofstream out(dst);
        if (!out) return std::nullopt;
        out << j.dump();
        return out ? std::optional<CashierModel>(m) : std::nullopt;
    }

    struct CellGrid {
        int ox = 0, oy = 0;
        std::vector<std::string> rows;
        int width() const { return rows.empty() ? 0 : static_cast<int>(rows[0].size()); }
        int height() const { return static_cast<int>(rows.size()); }
    };

    struct Overlays {
        std::vector<Cell> counter; // cashier counter cells
        Cell exit_door{-1, -1}; // real exit door
        Cell checkout{-1, -1}; // cashier access tile (path end) -> exit connector
        std::vector<std::array<Cell, 2> > dominoes; // 2-cell shelves -> half-joining bar
    };

    bool parseRender(const std::string &render, CellGrid &out) {
        if (render.empty()) return false;

        std::istringstream in(render);
        std::string line;
        bool have_origin = false;
        while (std::getline(in, line)) {
            const std::string key = "origin (map coords) = (";
            std::size_t p = line.find(key);
            if (p != std::string::npos) {
                if (std::sscanf(line.c_str() + p + key.size(), "%d, %d", &out.ox, &out.oy) == 2)
                    have_origin = true;
                continue;
            }
            if (line.find("legend") != std::string::npos) continue;
            if (line.find("coverage =") != std::string::npos) continue;
            if (line.size() < 2) continue;
            out.rows.push_back(line.substr(2)); // strip the 2-space margin
        }
        return have_origin && !out.rows.empty();
    }

    bool isFacing(char ch) {
        return ch == '^' || ch == 'v' || ch == '<' || ch == '>' || ch == '|' || ch == '-';
    }

    struct Style {
        const char *fill;
        const char *label;
    };

    Style styleFor(char ch) {
        static const char *kFaces[10] = {
            "", "#cfe9c4", "#aed89c", "#8ac674", "#67b34e",
            "#48a033", "#2f8a20", "#1d7414", "#0f5d0a", "#0a4a06"
        };
        if (ch >= '1' && ch <= '9') return {kFaces[ch - '0'], ""};
        if (isFacing(ch)) return {"#48a033", ""}; // counted shelf (n1/n2); arrow drawn on top
        switch (ch) {
            case '.': return {"#e9eef3", ""}; // unused floor
            case '*': return {"#4a90d9", ""}; // customer path
            case 'x': // wasted shelf (0 faces seen)
            case 'o': return {"#f0a030", ""}; // roadblock -- drawn as a wasted-shelf block
            case 'S': return {"#e0457b", "S"}; // entrance
            case 'T': return {"#16a085", "T"}; // goal (cashier access)
            default: return {nullptr, ""}; // '#' / obstacle / unknown
        }
    }

    using Proj = std::function<std::pair<double, double>(double, double)>;

    std::string drawPanel(const CellGrid &g, const Overlays &ov,
                          const Proj &proj, double &w, double &h) {
        const double pad = 8.0;

        double minx = 1e18, miny = 1e18, maxx = -1e18, maxy = -1e18;
        auto bumpCell = [&](double mx, double my) {
            for (int dy = 0; dy <= 1; ++dy)
                for (int dx = 0; dx <= 1; ++dx) {
                    auto [px, py] = proj(mx + dx, my + dy);
                    minx = std::min(minx, px);
                    maxx = std::max(maxx, px);
                    miny = std::min(miny, py);
                    maxy = std::max(maxy, py);
                }
        };
        bool any = false;
        for (int r = 0; r < g.height(); ++r)
            for (int c = 0; c < g.width(); ++c)
                if (styleFor(g.rows[r][c]).fill) {
                    any = true;
                    bumpCell(g.ox + c, g.oy + r);
                }
        for (const auto &c: ov.counter) {
            any = true;
            bumpCell(c.first, c.second);
        }
        if (ov.exit_door.first >= 0) {
            any = true;
            bumpCell(ov.exit_door.first, ov.exit_door.second);
        }
        if (!any) {
            w = h = 0;
            return {};
        }

        const double offx = -minx + pad, offy = -miny + pad;
        w = (maxx - minx) + 2 * pad;
        h = (maxy - miny) + 2 * pad;

        auto at = [&](double mx, double my) {
            auto [px, py] = proj(mx, my);
            return std::pair<double, double>{px + offx, py + offy};
        };
        auto poly = [&](std::ostringstream &s, double mx, double my, const char *fill,
                        const char *stroke, double sw) {
            const std::pair<double, double> c[4] = {
                at(mx, my), at(mx + 1, my), at(mx + 1, my + 1), at(mx, my + 1)
            };
            s << "  <polygon points=\"";
            for (int k = 0; k < 4; ++k) s << num(c[k].first) << ',' << num(c[k].second) << ' ';
            s << "\" fill=\"" << fill << "\" stroke=\"" << stroke
                    << "\" stroke-width=\"" << num(sw) << "\"/>\n";
        };
        auto label = [&](std::ostringstream &s, double mx, double my, const char *txt,
                         const char *color) {
            auto [lx, ly] = at(mx + 0.5, my + 0.5);
            s << "  <text x=\"" << num(lx) << "\" y=\"" << num(ly + 4)
                    << "\" font-size=\"11\" font-weight=\"bold\" fill=\"" << color
                    << "\" text-anchor=\"middle\">" << txt << "</text>\n";
        };

        auto facing = [&](std::ostringstream &s, double mx, double my, char ch) {
            const double cx = mx + 0.5, cy = my + 0.5, r = 0.42;
            double ax = cx, ay = cy, bx = cx, by = cy; // a = tail, b = arrowhead
            bool dbl = false;
            switch (ch) {
                case '^': ay = cy + r;
                    by = cy - r;
                    break;
                case 'v': ay = cy - r;
                    by = cy + r;
                    break;
                case '<': ax = cx + r;
                    bx = cx - r;
                    break;
                case '>': ax = cx - r;
                    bx = cx + r;
                    break;
                case '|': ay = cy + r;
                    by = cy - r;
                    dbl = true;
                    break;
                case '-': ax = cx - r;
                    bx = cx + r;
                    dbl = true;
                    break;
                default: return;
            }
            auto [px1, py1] = at(ax, ay);
            auto [px2, py2] = at(bx, by);
            s << "  <line x1=\"" << num(px1) << "\" y1=\"" << num(py1) << "\" x2=\""
                    << num(px2) << "\" y2=\"" << num(py2)
                    << "\" stroke=\"#16400f\" stroke-width=\"1.7\" marker-end=\"url(#arrow)\"";
            if (dbl) s << " marker-start=\"url(#arrow)\"";
            s << "/>\n";
        };

        std::ostringstream s;

        for (int r = 0; r < g.height(); ++r) {
            for (int c = 0; c < g.width(); ++c) {
                const char ch = g.rows[r][c];
                const Style st = styleFor(ch);
                if (!st.fill) continue;
                const double mx = g.ox + c, my = g.oy + r;
                poly(s, mx, my, st.fill, "#b7c2cd", 0.6);
                if (st.label[0]) label(s, mx, my, st.label, "#ffffff");
                if (isFacing(ch)) facing(s, mx, my, ch);
            }
        }

        for (const auto &d: ov.dominoes) {
            const int gr = d[0].second - g.oy, gc = d[0].first - g.ox;
            const char ch = (gr >= 0 && gr < g.height() && gc >= 0 && gc < g.width())
                                ? g.rows[gr][gc]
                                : '\0';
            // bar on scoring shelves (digit for n4/n8, facing glyph for n1/n2); skip 'x'
            if (!((ch >= '1' && ch <= '9') || isFacing(ch))) continue;
            auto [ax, ay] = at(d[0].first + 0.5, d[0].second + 0.5);
            auto [bx, by] = at(d[1].first + 0.5, d[1].second + 0.5);
            s << "  <line x1=\"" << num(ax) << "\" y1=\"" << num(ay) << "\" x2=\""
                    << num(bx) << "\" y2=\"" << num(by)
                    << "\" stroke=\"#14532d\" stroke-width=\"2.2\" stroke-linecap=\"round\"/>\n";
        }

        if (ov.checkout.first >= 0 && ov.exit_door.first >= 0) {
            auto [ax, ay] = at(ov.checkout.first + 0.5, ov.checkout.second + 0.5);
            auto [bx, by] = at(ov.exit_door.first + 0.5, ov.exit_door.second + 0.5);
            s << "  <line x1=\"" << num(ax) << "\" y1=\"" << num(ay) << "\" x2=\""
                    << num(bx) << "\" y2=\"" << num(by)
                    << "\" stroke=\"#888\" stroke-width=\"1.5\" stroke-dasharray=\"4 3\"/>\n";
        }

        for (std::size_t i = 0; i < ov.counter.size(); ++i)
            poly(s, ov.counter[i].first, ov.counter[i].second, "#f1c40f", "#b8860b", 1.5);
        if (!ov.counter.empty())
            label(s, ov.counter[0].first, ov.counter[0].second, "C", "#7a5c00");
        if (ov.exit_door.first >= 0) {
            poly(s, ov.exit_door.first, ov.exit_door.second, "#8e44ad", "#5e2d75", 1.0);
            label(s, ov.exit_door.first, ov.exit_door.second, "E", "#ffffff");
        }

        {
            const double ax = pad + 10, ay = pad + 10;
            auto [ox0, oy0] = proj(0, 0);
            auto unit = [&](double dx, double dy) {
                auto [px, py] = proj(dx, dy);
                double vx = px - ox0, vy = py - oy0;
                double len = std::sqrt(vx * vx + vy * vy);
                if (len < 1e-9) len = 1;
                return std::pair<double, double>{vx / len * 24.0, vy / len * 24.0};
            };
            auto arrow = [&](double dx, double dy, const char *col, const char *txt) {
                s << "  <line x1=\"" << num(ax) << "\" y1=\"" << num(ay) << "\" x2=\""
                        << num(ax + dx) << "\" y2=\"" << num(ay + dy) << "\" stroke=\"" << col
                        << "\" stroke-width=\"1.5\" marker-end=\"url(#arrow)\"/>\n";
                s << "  <text x=\"" << num(ax + dx * 1.25) << "\" y=\"" << num(ay + dy * 1.25 + 4)
                        << "\" font-size=\"10\" fill=\"" << col << "\" text-anchor=\"middle\">"
                        << txt << "</text>\n";
            };
            auto [xdx, xdy] = unit(1, 0);
            auto [ydx, ydy] = unit(0, 1);
            arrow(xdx, xdy, "#c0392b", "+x");
            arrow(ydx, ydy, "#2471a3", "+y");
        }

        return s.str();
    }

    bool writeSolutionSvg(const std::string &out_path, const shelf::SolveResult &res,
                          int map_index, int level, const char *nb, bool two_cell,
                          const std::optional<CashierModel> &model) {
        CellGrid g;
        if (!parseRender(res.render, g)) return false;

        Overlays ov;
        if (model) {
            ov.counter = model->counter;
            ov.exit_door = model->real_exit;
            ov.checkout = res.exit; // solver T == the cashier access tile
        }
        ov.dominoes = res.dominoes; // empty unless 2-cell mode

        const double T = 22.0;
        Proj rawProj = [T](double mx, double my) {
            return std::pair<double, double>{mx * T, my * T};
        };

        const double S = 15.0;
        Proj isoProj = [S](double mx, double my) {
            return std::pair<double, double>{(mx - my) * S, -(mx + my) * S};
        };

        double rawW = 0, rawH = 0, isoW = 0, isoH = 0;
        const std::string rawFrag = drawPanel(g, ov, rawProj, rawW, rawH);
        const std::string isoFrag = drawPanel(g, ov, isoProj, isoW, isoH);

        const double pad = 24, gap = 60, titleH = 32, subH = 26, legendH = 110;
        const double contentTop = titleH + subH;
        const double contentH = std::max(rawH, isoH);
        const double rawX = pad;
        const double isoX = pad + rawW + gap;
        const double totalW = std::max(isoX + isoW + pad, 560.0);
        const double totalH = contentTop + contentH + legendH;

        std::ostringstream svg;
        svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << num(totalW)
                << "\" height=\"" << num(totalH) << "\" viewBox=\"0 0 " << num(totalW)
                << ' ' << num(totalH) << "\" font-family=\"sans-serif\">\n";
        svg << "  <defs><marker id=\"arrow\" viewBox=\"0 0 10 10\" refX=\"8\" refY=\"5\""
                " markerWidth=\"6\" markerHeight=\"6\" orient=\"auto-start-reverse\">"
                "<path d=\"M0,0 L10,5 L0,10 z\" fill=\"#555\"/></marker></defs>\n";
        svg << "  <rect width=\"" << num(totalW) << "\" height=\"" << num(totalH)
                << "\" fill=\"#ffffff\"/>\n";

        svg << "  <text x=\"" << num(totalW / 2) << "\" y=\"28\" font-size=\"18\""
                " font-weight=\"bold\" text-anchor=\"middle\">map " << map_index
                << "  ·  level " << level << "  ·  coverage " << (two_cell ? "2cell/" : "") << nb;
        if (res.feasible)
            svg << "  ·  score " << res.coverage << "  ·  path " << res.path_length;
        else
            svg << "  ·  INFEASIBLE";
        svg << "</text>\n";

        svg << "  <text x=\"" << num(rawX + rawW / 2) << "\" y=\"" << num(titleH + 18)
                << "\" font-size=\"13\" font-weight=\"bold\" text-anchor=\"middle\">"
                "Solver grid</text>\n";
        svg << "  <text x=\"" << num(isoX + isoW / 2) << "\" y=\"" << num(titleH + 18)
                << "\" font-size=\"13\" font-weight=\"bold\" text-anchor=\"middle\">"
                "Game view (isometric)</text>\n";
        svg << "  <g transform=\"translate(" << num(rawX) << ',' << num(contentTop)
                << ")\">\n" << rawFrag << "  </g>\n";
        svg << "  <g transform=\"translate(" << num(isoX) << ',' << num(contentTop)
                << ")\">\n" << isoFrag << "  </g>\n";

        const double ly = contentTop + contentH + 22;
        struct Item {
            const char *fill;
            const char *stroke;
            const char *text;
        };
        std::vector<Item> items = {
            {"#4a90d9", "#b7c2cd", "customer path"},
            {"#48a033", "#b7c2cd", two_cell ? "2-cell shelf" : "shelf"},
            {"#f0a030", "#b7c2cd", "wasted shelf"}, {"#e9eef3", "#b7c2cd", "unused floor"},
            {"#e0457b", "#b7c2cd", "S = entrance"}, {"#16a085", "#b7c2cd", "T = checkout"},
            {"#f1c40f", "#b8860b", "C = cashier"}, {"#8e44ad", "#5e2d75", "E = exit"},
        };
        double lx = pad, lrow = ly;
        int col = 0;
        for (const auto &it: items) {
            svg << "  <rect x=\"" << num(lx) << "\" y=\"" << num(lrow)
                    << "\" width=\"14\" height=\"14\" fill=\"" << it.fill << "\" stroke=\""
                    << it.stroke << "\"/>\n";
            svg << "  <text x=\"" << num(lx + 20) << "\" y=\"" << num(lrow + 11)
                    << "\" font-size=\"12\">" << it.text << "</text>\n";
            if (++col % 4 == 0) {
                lx = pad;
                lrow += 22;
            } else { lx += 200; }
        }

        svg << "</svg>\n";

        std::ofstream out(out_path);
        if (!out) return false;
        out << svg.str();
        return static_cast<bool>(out);
    }
}

int main(int argc, char **argv) {
    int map_index = 0;
    int level = 0;
    shelf::SolveOptions opt;
    std::string svg_path = "solution.svg";

    if (argc >= 3) {
        map_index = std::atoi(argv[1]);
        level = std::atoi(argv[2]);
        for (int i = 3; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "n1") opt.neighborhood = shelf::Neighborhood::N1;
            else if (a == "n2") opt.neighborhood = shelf::Neighborhood::N2;
            else if (a == "n4") opt.neighborhood = shelf::Neighborhood::N4;
            else if (a == "n8") opt.neighborhood = shelf::Neighborhood::N8;
            else if (a == "2cell" || a == "2") opt.two_cell = true;
            else if (endsWith(a, ".svg")) svg_path = a;
            else opt.maps_dir = a;
        }
    } else {
        std::cout << "Map index: ";
        if (!(std::cin >> map_index)) return 1;
        std::cout << "Current level: ";
        if (!(std::cin >> level)) return 1;
    }

    const char *nb = "n8";
    switch (opt.neighborhood) {
        case shelf::Neighborhood::N1: nb = "n1";
            break;
        case shelf::Neighborhood::N2: nb = "n2";
            break;
        case shelf::Neighborhood::N4: nb = "n4";
            break;
        case shelf::Neighborhood::N8: nb = "n8";
            break;
    }

    const std::string cov_label = (opt.two_cell ? std::string("2cell/") : std::string()) + nb;
    std::cout << "Solving map " << map_index << " at level " << level
            << " (coverage: " << cov_label << ", maps dir: " << opt.maps_dir << ")...\n";

    const std::string src_path = opt.maps_dir + "/" + std::to_string(map_index) + "/buildConstData.json";
    namespace fs = std::filesystem;
    const fs::path tmp_path =
            fs::temp_directory_path() / ("shelf_solver_m" + std::to_string(map_index) + ".tmp.json");
    const std::optional<CashierModel> model = buildCashierMap(src_path, tmp_path.string());

    const auto t0 = std::chrono::steady_clock::now();

    opt.on_progress = [t0](const shelf::SolveProgress &p) {
        const int width = 30;
        const int filled = static_cast<int>(p.fraction * width + 0.5);
        std::string bar(static_cast<std::size_t>(filled), '#');
        bar.resize(width, '.');
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::cerr << "\r  solving [" << bar << "] " << std::setw(3)
                << static_cast<int>(p.fraction * 100 + 0.5) << "%   best="
                << std::setw(4) << p.best << "   " << std::fixed << std::setprecision(1)
                << secs << "s   " << std::flush;
    };

    shelf::SolveResult res = model
                                 ? shelf::solveMapFile(tmp_path.string(), level, opt)
                                 : shelf::solveMap(map_index, level, opt);
    const std::chrono::duration<double> dt = std::chrono::steady_clock::now() - t0;
    std::cerr << "\n"; // finish the progress line

    std::error_code ec;
    fs::remove(tmp_path, ec);

    if (!res.ok) {
        std::cerr << "ERROR: " << res.error << "\n";
        return 1;
    }

    std::cout << "\nwalkable=" << res.walkable << "  flippable=" << res.flippable
            << "  entrance=(" << res.entrance.first << "," << res.entrance.second << ")";
    if (model)
        std::cout << "  cashier=(" << res.exit.first << "," << res.exit.second << ")"
                << "  exit_door=(" << model->real_exit.first << "," << model->real_exit.second << ")";
    else
        std::cout << "  exit=(" << res.exit.first << "," << res.exit.second << ")";
    std::cout << "\n";

    if (!res.feasible) {
        std::cout << "INFEASIBLE: the cashier is unreachable at this level.\n";
        if (!res.render.empty()) std::cout << res.render;
    } else {
        std::cout << "feasible=yes  score=" << res.coverage
                << "  path_len=" << res.path_length;
        if (opt.two_cell)
            std::cout << "  shelves(2-cell)=" << res.dominoes.size()
                    << "  roadblocks=" << res.roadblocks.size();
        std::cout << "  (" << dt.count() << " s)\n\n";
        std::cout << res.render;
    }

    if (writeSolutionSvg(svg_path, res, map_index, level, nb, opt.two_cell, model)) {
        std::cout << "\nWrote SVG visualization to " << svg_path
                << "  (right panel matches the in-game orientation)\n";
    } else {
        std::cerr << "\nWARNING: could not write SVG to " << svg_path << "\n";
    }

    return 0;
}
