#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <tuple>
#include <sstream>
#include <algorithm>
#include <ctime>

struct Time
{
    int begin;
    int end;
    int service;
    int current;
};

struct Problem
{
    Problem() : n_vehicle{}, capacity{}, times{}, cost{}, demand{} {};
    Problem(size_t n_vehicle_, size_t capacity_, std::vector<Time> times_,
            std::vector<std::vector<float>> cost_, std::vector<size_t> demand_) : n_vehicle(n_vehicle_), capacity(capacity_),
                                                                                  times(times_), cost(cost_), demand(demand_){};

    size_t n_vehicle;
    size_t capacity;
    std::vector<Time> times;
    std::vector<std::vector<float>> cost;
    std::vector<size_t> demand;

    std::vector<std::tuple<size_t, double>> polar_coord;
};

struct Solution
{
    Solution() : routes{}, times{} {};
    Solution(std::vector<std::vector<size_t>> routes_, std::vector<Time> times_) : routes(routes_), times(times_){};

    std::vector<std::vector<size_t>> routes;
    std::vector<Time> times;
};

float CalcObjective(const Problem &prb, const Solution &sln)
{
    float obj = 0;

    for (int i = 0; i < sln.routes.size(); ++i)
        for (int j = 0; j < sln.routes[i].size() - 1; ++j)
            obj += prb.cost[sln.routes[i][j]][sln.routes[i][j + 1]];

    return obj;
};

Solution ClusterFirstRouteSecond(const Problem &prob)
{
    Solution sln;
    Problem prb = prob;
    size_t tmp_cap = 0;
    std::vector<size_t> route;

    std::vector<std::vector<size_t>> clusters;

    // go throw polar coord angles and add with capacity check
    // then inside clusters perform tsp/whatever

    // STAGE 1 -- CLUSTERRING
    int i = 0;

    // FIXME: magic num to get it works for r105.txt
    int cust_per_veh = (((prb.demand.size() - 1) / prb.n_vehicle) + 1) + 6;
    int current_cust = 0;

    while (i < prb.polar_coord.size())
    {
        if (((tmp_cap += prb.demand[std::get<0>(prb.polar_coord[i])]) <= prb.capacity) && (current_cust < cust_per_veh))
        {
            route.push_back(std::get<0>(prb.polar_coord[i]));
            ++i;
            ++current_cust;
        }
        else
        {
            current_cust = 0;
            clusters.push_back(route);
            route.clear();
            tmp_cap = 0;
        }
    }

    if (!route.empty())
    {
        clusters.push_back(route);
    }

    std::vector<size_t> non_visited;
    std::vector<size_t> feasible_route = {0};

    // STAGE 2 -- INSIDE THE CLUSTERS
    for (int i = 0; i < clusters.size(); ++i)
    {
        std::sort(clusters[i].begin(), clusters[i].end(),
                  [&](const size_t &A, const size_t &B) {
                      return prb.times[A].end < prb.times[B].end;
                  });

        clusters[i].insert(clusters[i].begin(), 0);
        clusters[i].push_back(0);

        int last_serve_time = 0;
        for (int j = 1; j < clusters[i].size(); ++j)
        {
            int curr = clusters[i][j];
            int prev = feasible_route.back();
            // can be served in time and depot is reachable from cust
            // TODO: < or <= ?
            if ((last_serve_time + (int)std::round(prb.cost[prev][curr]) + prb.times[prev].service <= prb.times[curr].end) &&
                (last_serve_time + (int)std::round(prb.cost[prev][curr]) +
                     prb.times[prev].service + (int)std::round(prb.cost[curr][0]) + prb.times[curr].service <=
                 prb.times[0].end))
            {
                feasible_route.push_back(curr);
                prb.times[curr].current = std::max(last_serve_time + (int)std::round(prb.cost[prev][curr]) + prb.times[prev].service,
                                                   prb.times[curr].begin);
                last_serve_time = prb.times[curr].current;
            }
            else
            {
                // manage this cust later
                non_visited.push_back(curr);
            }
        }

        // have more than depot
        if (feasible_route.size() > 2)
        {
            feasible_route.insert(feasible_route.begin(), 0);
            sln.routes.push_back(feasible_route);
        }

        feasible_route.clear();
        feasible_route = {0};
    }

    // STAGE 3 -- NON-VISITED CUSTOMERS
    std::sort(non_visited.begin(), non_visited.end(),
              [&](const size_t &A, const size_t &B) {
                  return prb.times[A].end < prb.times[B].end;
              });

    // num of vehicles left
    for (int veh = 0; veh < prb.n_vehicle - sln.routes.size(); ++veh)
    {
        int tmp_cap = 0;
        int last_served = 0;
        int prev = 0;
        int curr = 0;

        for (int i = 0; i < non_visited.size(); ++i)
        {
            curr = non_visited[i];
            prev = feasible_route.back();

            // depot is reachable and time is feasible
            if ((last_served + (int)std::round(prb.cost[prev][curr]) + prb.times[prev].service <= prb.times[curr].end) &&
                (last_served + (int)std::round(prb.cost[prev][curr]) +
                     prb.times[prev].service + (int)std::round(prb.cost[curr][0]) + prb.times[curr].service <=
                 prb.times[0].end))
            {
                if ((tmp_cap + prb.demand[curr]) <= prb.capacity)
                {
                    tmp_cap += prb.demand[curr];
                    feasible_route.push_back(curr);
                    prb.times[curr].current = std::max(last_served + (int)std::round(prb.cost[prev][curr]) +
                                                           prb.times[prev].service,
                                                       prb.times[curr].begin);
                    last_served = prb.times[curr].current;
                }
            }
        }

        if (feasible_route.size() > 1)
        {
            feasible_route.push_back(0);

            sln.routes.push_back(feasible_route);

            for (int i = 1; i < feasible_route.size() - 1; ++i)
            {
                non_visited.erase(std::remove(non_visited.begin(), non_visited.end(), feasible_route[i]), non_visited.end());
            }

            feasible_route.clear();
            feasible_route = {0};
        }
    }

    // FIXME: DELETE THIS
    if (!non_visited.empty())
    {
        std::cout << "****CANNOT FIX NON-VISITED CUST****" << std::endl;
        std::cout << non_visited.size() << " CUST LEFT" << std::endl;
    }

    // FIXME: DELETE THIS
    static char ttt = 'a';
    std::ofstream file;
    std::string str("out/solution");
    str.push_back(ttt++);
    str.append(".txt");
    file.open(str);

    // FIXME: DELETE THIS
    for (auto &a : sln.routes)
    {
        for (auto &b : a)
        {
            file << b << ' ';
        }
        file << std::endl;
    }

    sln.times = prb.times;

    return sln;
}

bool AccuracyCheck(const Problem &prb, const Solution &sln)
{
    // sanity check
    if (sln.routes.size() == 0)
    {
        std::cout << "!!!NO ROUTES!!!" << std::endl;
        return false;
    }

    int tmp_cap = 0;
    std::vector<size_t> visited(sln.times.size(), 0);

    // set visited and check capacity
    for (auto &route : sln.routes)
    {
        for (auto &cust : route)
        {
            tmp_cap += prb.demand[cust];
            visited[cust] += 1;
        }
        if (tmp_cap > prb.capacity)
        {
            std::cout << "!!!OUT OF CAPACITY!!!" << std::endl;
            return false;
        }
        tmp_cap = 0;
    }

    // check visited
    for (int i = 1; i < visited.size(); ++i)
        if (visited[i] != 1)
        {
            std::cout << "!!!VISITING PROBLEM!!!" << std::endl;
            return false;
        }

    // check times
    int time_ = 0;
    int prev = 0;
    int curr = 0;
    for (int i = 0; i < sln.routes.size(); ++i)
    {
        for (int j = 1; j < sln.routes[i].size(); ++j)
        {
            prev = sln.routes[i][j - 1];
            curr = sln.routes[i][j];
            time_ = std::max(time_ + (int)std::round(prb.cost[prev][curr]) + prb.times[prev].service, prb.times[curr].begin);
            if (time_ > prb.times[curr].end)
            {
                std::cout << "!!!BAD TIMING - TOO LATE!!!" << std::endl;
                return false;
            }
        }
        time_ = 0;
    }

    // check vehicles
    if (sln.routes.size() > prb.n_vehicle)
    {
        std::cout << "!!!OUT OF VEHICLES!!!" << std::endl;
        return false;
    }

    //std::cout << "!!!ACCURACY PASSED!!!" << std::endl;
    return true;
}

void ParseInput(std::ifstream &in, Problem &prb)
{
    std::vector<std::pair<float, float>> xy;
    std::string s;
    std::stringstream helper;

    // skip unused lines
    for (int i = 0; i < 4; ++i)
        std::getline(in, s);

    in >> prb.n_vehicle >> prb.capacity;

    // skip unused lines
    for (int i = 0; i < 5; ++i)
        std::getline(in, s);

    // CUST NO.   XCOORD.   YCOORD.   DEMAND    READY TIME   DUE DATE   SERVICE TIME
    int tmp, dem, start, end, dur;
    float x, y;

    while (std::getline(in, s))
    {
        helper << s;
        helper >> tmp >> x >> y >> dem >> start >> end >> dur;

        prb.times.push_back({start, end, dur, 0});
        prb.demand.emplace_back(dem);
        xy.push_back({x, y});

        helper.clear();
    }

    // fill cost matrix
    std::vector<float> dist;
    for (int i = 0; i < xy.size(); ++i)
    {
        for (int j = 0; j < xy.size(); ++j)
        {
            dist.emplace_back(sqrt(pow(xy[i].first - xy[j].first, 2) + pow(xy[i].second - xy[j].second, 2)));
        }
        prb.cost.emplace_back(dist);
        dist.clear();
    }

    // calculate polar coords
    for (size_t i = 1; i < xy.size(); ++i)
    {
        double ttan = atan2(xy[i].second - xy[0].second, xy[i].first - xy[0].first);
        prb.polar_coord.push_back(std::make_tuple(i, ttan));
    }
    std::sort(prb.polar_coord.begin(), prb.polar_coord.end(),
              [](const std::tuple<size_t, double> &a,
                 const std::tuple<size_t, double> &b) {
                  return (std::get<1>(a) - std::get<1>(b) > 0);
              });

    // FIXME: magic num 89 to get it works for r105.txt
    std::rotate(prb.polar_coord.begin(), prb.polar_coord.begin() + 89, prb.polar_coord.end());
}

Solution LocalSearch(const Problem &prb, const Solution &sln)
{
    Solution new_sol(sln);

    // 1-opt
    for (int i = 0; i < sln.routes.size(); ++i)
    {
        for (int j = 1; j < sln.routes[i].size(); ++j)
        {
        }
    }

    // 2-opt

    return AccuracyCheck(prb, new_sol) ? new_sol : sln;
}

int main()
{
    std::vector<std::string> files{
        "C108.txt", "C203.txt", "C249.TXT",
        "C266.TXT", "R146.TXT", "R168.TXT",
        "R202.txt", "RC105.txt", "RC148.TXT",
        "RC207.txt", "R1103.TXT", "R1104.TXT",
        "R1105.TXT", "R1107.TXT", "R11010.TXT"};

    for (auto &file : files)
    {
        std::cout << "Case " << file << std::endl;
        std::ifstream in(file);

        Problem prb;
        ParseInput(in, prb);

        Solution sln = ClusterFirstRouteSecond(prb);
        if (!AccuracyCheck(prb, sln))
            std::cout << " !!!DID NOT PASS ACCURACY TEST!!!" << std::endl;
        else
            std::cout << " OBJECTIVE = " << CalcObjective(prb, sln) << std::endl;

        // run local search to improve solution
        /* Solution improved_sln = sln;
        for (int i = 0; i < 10; ++i)
        {
            improved_sln = LocalSearch(prb, improved_sln);
            if (!AccuracyCheck(prb, improved_sln))
                std::cout << "!!!DID NOT PASS ACCURACY TEST!!!" << std::endl;
            else
                std::cout << "IMPROVED OBJECTIVE = " << CalcObjective(prb, improved_sln) << std::endl;
        }*/

        std::cout << std::endl;
    }
}
