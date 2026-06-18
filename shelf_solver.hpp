#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace shelf {
    struct SolveProgress {
        double fraction = 0.0; // SA iterations completed / total, clamped to [0, 1]
        int best = -1; // best coverage found so far across all restarts
    };

    enum class Neighborhood { N8, N4, N2, N1 };

    struct SolveOptions {
        Neighborhood neighborhood = Neighborhood::N8; // shelf adjacency model
        bool two_cell = false;
        std::uint64_t iters_base = 200000; // SA iterations, constant part
        std::uint64_t iters_per_cell = 25000; // ... plus this many per walkable cell
        int restarts = 8; // independent SA restarts (best wins)
        double t0 = 3.0; // SA start temperature
        double t_min = 1e-2; // SA end temperature
        std::uint32_t seed = 2024; // RNG seed (deterministic output)
        unsigned threads = 0; // worker threads (0 => hardware concurrency)
        bool exclude_high_platform = true;
        std::string maps_dir = "Scene";
        std::function<void(const SolveProgress &)> on_progress;
    };

    struct SolveResult {
        bool ok = false; // false => see `error`
        std::string error;

        bool feasible = false; // true => a customer path exists
        int coverage = 0; // total shelf-face contacts along the realised path
        int path_length = 0;

        int level = 0; // level that was solved
        int walkable = 0; // number of walkable floor tiles
        int flippable = 0; // tiles the solver may turn into shelves

        std::pair<int, int> entrance{-1, -1}; // S, in map coords
        std::pair<int, int> exit{-1, -1}; // T, in map coords

        std::vector<std::pair<int, int> > path; // customer path, s..t order
        std::vector<std::pair<int, int> > counted_shelves; // shelves the customer sees

        std::vector<std::array<std::pair<int, int>, 2> > dominoes;
        std::vector<std::pair<int, int> > roadblocks;

        std::string render; // human-readable ASCII map of the solution
    };

    SolveResult solveMap(int map_index, int level, const SolveOptions &opt = {});

    SolveResult solveMapFile(const std::string &map_path, int level,
                             const SolveOptions &opt = {});

    SolveResult solveMapJson(std::string_view map_json, int level,
                             const SolveOptions &opt = {});
}
