#define _POSIX_C_SOURCE 200809L
#include <iostream>
#include <fstream>
#include <ranges>
#include <sstream>
#include <vector>
#include <algorithm>
#include <map>
#include <filesystem>
#include <string>
#include <thread>
#include <optional>
#include <cmath>
#include <ctime>
#include <random>
#include <unistd.h>
#include <cstring>
#include <cassert>

static std::vector<int>
parse_cpu_list(const std::string& cpu_list_str)
{
    std::stringstream ss(cpu_list_str);
    std::string part;

    std::vector<int> cpus;
    while (std::getline(ss, part, ',')) {
        size_t pos = part.find('-');
        if (pos != std::string::npos) {
            int start = std::stoi(part.substr(0, pos));
            int end = std::stoi(part.substr(pos+1));
            for (int i = start; i <= end; i++)
                cpus.push_back(i);
        } else {
            cpus.push_back(std::stoi(part));
        }
    }

    return cpus;
}

static std::map<int, std::vector<int>>
get_numa_nodes()
{
    namespace fs = std::filesystem;

    std::map<int, std::vector<int>> numa_nodes;
    fs::path node_path = "/sys/devices/system/node/";

    // List all NUMA nodes
    for (const auto& node : fs::directory_iterator(node_path)) {
        std::string name = node.path().filename().string();
        if (name.starts_with("node")) {
            int node_id = std::stoi(name.substr(4));  // Extract NUMA node number
            fs::path cpu_path = node_path / name / "cpulist";
            std::ifstream file;
            file.open(cpu_path);
            if (!file.is_open())
                continue;
            std::string contents{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
            auto cpus = parse_cpu_list(contents);
            numa_nodes[node_id] = cpus;
            file.close();
        }
    }

    return numa_nodes;
}

struct Stats {
    int user;
    int nice;
    int system;
    int idle;
};

static std::map<unsigned, Stats>
get_stats()
{
    std::map<unsigned, Stats> stats;

    std::ifstream file;
    file.open("/proc/stat");
    if (!file.is_open())
        throw std::runtime_error("cannot open file");
    std::string contents{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
    file.close();

    std::stringstream ss(contents);
    std::vector<std::string> lines;
    std::string line;
    while(std::getline(ss, line, '\n')) {
        lines.push_back(line);
    }

    unsigned total_cores = std::thread::hardware_concurrency();
    for (unsigned core = 0; core < total_cores; core++) {
        auto cpu_str = "cpu" + std::to_string(core);
        std::string core_stats_line;
        for (const auto& line : lines) {
            if (line.starts_with(cpu_str)) {
                core_stats_line = line;
                break;
            }
        }
        if (core_stats_line.empty())
            continue;
        Stats core_stats{};
        std::stringstream ss(core_stats_line);
        std::string stat;
        for (int i = 0; i < 5; i++) {
            std::getline(ss, stat, ' ');
            switch (i) {
                case 1: core_stats.user   = std::stoi(stat); break;
                case 2: core_stats.nice   = std::stoi(stat); break;
                case 3: core_stats.system = std::stoi(stat); break;
                case 4: core_stats.idle   = std::stoi(stat); break;
                default: assert(false);
            }
        }
        stats[core] = core_stats;
    }

    return stats;
}

static void
sleep_double(double interval)
{
    double seconds = NAN;
    double fractional = modf(interval, &seconds);
    struct timespec req = {
        .tv_sec = (time_t) seconds,
        .tv_nsec = (long) (fractional*1e9)
    };
    struct timespec rem{};
    while (nanosleep(&req, &rem)) {
        req = rem;
    }
}

static std::map<unsigned, int>
get_cores_load(double interval)
{
    std::map<unsigned, int> cores_load;

    auto stats1 = get_stats();
    sleep_double(interval);
    auto stats2 = get_stats();

    for (const auto& [core, core_stats1] : stats1) {
        if (stats2.find(core) == stats2.end())
            continue;  // Skip cores missing in stats2 (e.g., hot plug event)
        auto core_stats2 = stats2[core];
        int total = (core_stats2.user - core_stats1.user) +
            (core_stats2.nice - core_stats1.nice) +
            (core_stats2.system - core_stats1.system) +
            (core_stats2.idle - core_stats1.idle);
        int total_idle = core_stats2.idle - core_stats1.idle;
        int load = (total == 0) ? 0 : (100 * (total - total_idle)) / total;
        cores_load[core] = load;
    }

    return cores_load;
}

static std::optional<std::pair<int, std::vector<int>>>
find_low_load_cluster(double max_load, unsigned cluster_size, double interval)
{
    auto numa_nodes = get_numa_nodes();
    auto cores_load = get_cores_load(interval);

    for (const auto& [node, cpus] : numa_nodes) {
        std::vector<unsigned> start_cpus;
        for (auto cpu : cpus) {
            if (cpu % 2 == 0)
                start_cpus.push_back(cpu);
        }

        for (auto start_cpu : start_cpus) {
            std::vector<int> cluster;
            for (int i = 0; i < int(cluster_size); i++) {
                if (std::find(cpus.begin(), cpus.end(), start_cpu + i) != cpus.end())
                    cluster.push_back(int(start_cpu) + i);
            }

            if (cluster.size() != cluster_size)
                continue;  // Skip incomplete clusters

            double avg_load = 0;
            for (auto cpu : cluster)
                avg_load += cores_load[cpu];
            avg_load /= cluster_size;
            if (avg_load < max_load)
                return std::pair<int, std::vector<int>>{node, cluster};
        }
    }

    return std::nullopt;
}

std::vector<std::string>
get_cpubind_cmd(unsigned cores, double max_load, std::string& error)
{
    std::vector<std::string> command = {"numactl", "-l", "-C"};
    if (system("which numactl > /dev/null") != 0)
      {
        error = "numactl not found";
        return {};
      }
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.4, 2.0);
    double interval = dist(gen); // Randomize to avoid binding conflicts
    auto cluster = find_low_load_cluster(max_load, cores, interval);
    if (!cluster.has_value())
      {
        error = "No suitable cluster found (max_load = " + std::to_string(max_load) + "%)";
        return {};
      }
    auto [numa_node, cpus] = cluster.value();
    std::string cpu_str;
    for (auto cpu : cpus)
        cpu_str += std::to_string(cpu) + ",";
    if (!cpu_str.empty())
      cpu_str.pop_back();
    command.push_back(cpu_str);
    return command;
}

// NOLINTBEGIN(*-avoid-c-arrays, cppcoreguidelines-pro-bounds-pointer-arithmetic)
void
attempt_numactl(int argc, char * argv[], unsigned cores, double max_load)
{
    std::vector<char *> command_args;
    for (int i = 0; i < argc; i++)
      if (strcmp(argv[i], "--numa") != 0)
        command_args.push_back(argv[i]);
    command_args.push_back(nullptr);

    std::string error;
    auto cpubind_command = get_cpubind_cmd(cores, max_load, error);

    if (cpubind_command.empty()) {
        std::cerr << error << ": running without CPU binding.\n";
        return;
    }
    for (auto & it : std::ranges::reverse_view(cpubind_command))
      command_args.insert(command_args.begin(), 1, strdup(it.c_str()));

    for (auto *arg : command_args)
      if (arg != nullptr)
        std::cout << arg << " ";
    std::cout << "\n";
    execvp(command_args[0], command_args.data());
}
// NOLINTEND(*-avoid-c-arrays, cppcoreguidelines-pro-bounds-pointer-arithmetic)
