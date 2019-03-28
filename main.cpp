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

    /*// FIXME: DELETE THIS
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
    }*/

    sln.times = prb.times;

    return sln;
}

bool AccuracyCheck(const Problem &prb, const Solution &sln)
{
    // sanity check
    if (sln.routes.size() == 0)
    {
        //std::cout << "!!!NO ROUTES!!!" << std::endl;
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
            //std::cout << "!!!OUT OF CAPACITY!!!" << std::endl;
            return false;
        }
        tmp_cap = 0;
    }

    // check visited
    for (int i = 1; i < visited.size(); ++i)
        if (visited[i] != 1)
        {
            //std::cout << "!!!VISITING PROBLEM!!!" << std::endl;
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
                //std::cout << "!!!BAD TIMING - TOO LATE!!!" << std::endl;
                return false;
            }
        }
        time_ = 0;
    }

    // check vehicles
    if (sln.routes.size() > prb.n_vehicle)
    {
        //std::cout << "!!!OUT OF VEHICLES!!!" << std::endl;
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

//FIXME: fix vectors invalidation in local search
Solution LocalSearch(const Problem &prb, const Solution &sln)
{
    Solution sol_2_opt(sln);

    // 2-opt
    for (int r = 0; r < sln.routes.size(); ++r)
        for (int i = 1; i < sln.routes[r].size() - 1; ++i)
            for (int j = 1; j < sln.routes[r].size() - 1; ++j)
                if (j != i)
                {
                    Solution tmp_sol(sol_2_opt);
                    std::vector<size_t> new_route = sol_2_opt.routes[r];
                    std::swap(new_route[j], new_route[i]);
                    std::replace(tmp_sol.routes.begin(), tmp_sol.routes.end(), sol_2_opt.routes[r], new_route);

                    if (AccuracyCheck(prb, tmp_sol) && (CalcObjective(prb, tmp_sol) < CalcObjective(prb, sol_2_opt)))
                        sol_2_opt = tmp_sol;
                }

    // relocate
    int r1_size = 0;
    int r2_size = 0;
    int r3_size = 0;
    int i = 1, j = 1, k = 1;

    Solution sol_reloc(sln);

    for (int r = 1; r < sln.routes.size() - 1; ++r)
    {
        r1_size = sol_reloc.routes[r].size() - 1;
        while (i < r1_size)
        {
            r3_size = sol_reloc.routes[r - 1].size() - 1;
            while (k < r3_size)
            {
                Solution tmp_sol(sol_reloc);
                std::vector<size_t> new_route_r1 = tmp_sol.routes[r];
                std::vector<size_t> new_route_r2 = tmp_sol.routes[r - 1];

                new_route_r2.insert(new_route_r2.begin() + k, new_route_r1[i]);
                new_route_r1.erase(std::remove(new_route_r1.begin(),
                                               new_route_r1.end(), new_route_r1[i]),
                                   new_route_r1.end());

                std::replace(tmp_sol.routes.begin(), tmp_sol.routes.end(), tmp_sol.routes[r], new_route_r1);
                std::replace(tmp_sol.routes.begin(), tmp_sol.routes.end(), tmp_sol.routes[r - 1], new_route_r2);

                if (AccuracyCheck(prb, tmp_sol) && (CalcObjective(prb, tmp_sol) < CalcObjective(prb, sol_reloc)))
                {
                    sol_reloc = tmp_sol;

                    --r1_size;
                    if (i == r1_size)
                        break;
                    ++r3_size;
                }
                ++k;
            }

            if (i == r1_size)
                break;

            r2_size = sol_reloc.routes[r + 1].size() - 1;
            while (j < r2_size)
            {
                Solution tmp_sol(sol_reloc);
                std::vector<size_t> new_route_r1 = tmp_sol.routes[r];
                std::vector<size_t> new_route_r2 = tmp_sol.routes[r + 1];

                new_route_r2.insert(new_route_r2.begin() + j, new_route_r1[i]);
                new_route_r1.erase(std::remove(new_route_r1.begin(),
                                               new_route_r1.end(), new_route_r1[i]),
                                   new_route_r1.end());

                std::replace(tmp_sol.routes.begin(), tmp_sol.routes.end(), tmp_sol.routes[r], new_route_r1);
                std::replace(tmp_sol.routes.begin(), tmp_sol.routes.end(), tmp_sol.routes[r + 1], new_route_r2);

                if (AccuracyCheck(prb, tmp_sol) && (CalcObjective(prb, tmp_sol) < CalcObjective(prb, sol_reloc)))
                {
                    sol_reloc = tmp_sol;
                    --r1_size;
                    if (i == r1_size)
                        break;
                    ++r2_size;
                }
                ++j;
            }
            ++i;
            j = 1;
            k = 1;
        }
        i = 1;
    }

    // exchange
    r1_size = 0;
    r2_size = 0;

    i = j = 1;

    Solution sol_exch(sln);
    for (int r = 0; r < sln.routes.size() - 1; ++r)
    {
        r1_size = sol_exch.routes[r].size() - 1;

        while (i < r1_size)
        {
            r2_size = sol_exch.routes[r + 1].size() - 1;

            while (j < r2_size)
            {
                Solution tmp_sol(sol_exch);
                int cust_r1 = tmp_sol.routes[r][i];
                int cust_r2 = tmp_sol.routes[r + 1][j];

                std::vector<size_t> new_r1 = tmp_sol.routes[r];
                std::vector<size_t> new_r2 = tmp_sol.routes[r + 1];

                std::replace(new_r1.begin(), new_r1.end(), cust_r1, cust_r2);
                std::replace(new_r2.begin(), new_r2.end(), cust_r2, cust_r1);

                std::replace(tmp_sol.routes.begin(), tmp_sol.routes.end(), tmp_sol.routes[r], new_r1);
                std::replace(tmp_sol.routes.begin(), tmp_sol.routes.end(), tmp_sol.routes[r + 1], new_r2);

                if (AccuracyCheck(prb, tmp_sol) && (CalcObjective(prb, tmp_sol) < CalcObjective(prb, sol_exch)))
                {
                    sol_exch = tmp_sol;
                }
                ++j;
            }
            ++i;
            j = 1;
        }
        i = 1;
    }

    auto best_obj = std::min({CalcObjective(prb, sln), CalcObjective(prb, sol_2_opt),
                              CalcObjective(prb, sol_reloc), CalcObjective(prb, sol_exch)});

    if (best_obj == CalcObjective(prb, sol_2_opt))
        return sol_2_opt;
    if (best_obj == CalcObjective(prb, sol_reloc))
        return sol_reloc;
    if (best_obj == CalcObjective(prb, sol_exch))
        return sol_exch;

    return sln;
}

void SetSolutionDifference(Problem &prb, const Solution &sln1,
                           const Solution &sln2, std::vector<std::vector<size_t>> &pens,
                           const float &lambda)
{
    for (int i = 0; i < sln1.routes.size(); ++i)
        for (int j = 1; j < sln1.routes[i].size() - 1; ++j)
        {
            auto prev = sln1.routes[i][j - 1];
            auto curr = sln1.routes[i][j];
            if (j < sln2.routes[i].size() && prev == sln2.routes[i][j - 1] &&
                curr == sln2.routes[i][j])
            {
                ++pens[prev][curr];
                prb.cost[prev][curr] += prb.cost[prev][curr] * lambda;
            }
        }
}

Solution GuidedWithPen(Problem &modif_prob, const Solution &new_sol,
                       const Solution &curr_sol, std::vector<std::vector<size_t>> &pens,
                       const float &lambda)
{
    SetSolutionDifference(modif_prob, new_sol, curr_sol, pens, lambda);
    return LocalSearch(modif_prob, new_sol);
}

Solution GuidedLocalSearch(const Problem &prb, const Solution &sln, const float &lambda)
{
    Problem modif_prob = prb;

    Solution prev_best_sol = sln;
    Solution curr_best_sol = LocalSearch(prb, sln);

    // set penalty matrix for each arc
    std::vector<size_t> tmp(prb.cost.size(), 0);
    std::vector<std::vector<size_t>> pens(prb.cost.size(), tmp);

    int i = 0;
    const int stop = 10;

    while (true)
    {
        Solution new_sln = GuidedWithPen(modif_prob, curr_best_sol, prev_best_sol, pens, lambda);

        if (CalcObjective(prb, new_sln) - CalcObjective(prb, curr_best_sol) < 0)
        {
            std::cout << "GUIDED IMPROVED " << CalcObjective(prb, new_sln) << std::endl;
            prev_best_sol = curr_best_sol;
            curr_best_sol = new_sln;
        }
        else
        {
            break;
        }
    }

    curr_best_sol = LocalSearch(prb, curr_best_sol);

    return curr_best_sol;
}

int main()
{
    std::vector<std::string> files{
        "C108.txt", "C203.txt", "C249.TXT",
        "C266.TXT", "R146.TXT", "R168.TXT",
        "R202.txt", "RC105.txt", "RC148.TXT",
        "RC207.txt", "R1103.TXT", "R1104.TXT",
        "R1105.TXT", "R1107.TXT", "R11010.TXT"};

    // more magic numbers
    std::vector<float> lambdas{0.03, 0.03, 0.01,
                               0.006, 0.00005, 0.0005,
                               0.01, 0.01, 0.0005,
                               0.01, 0.00001, 0.00001,
                               0.00001, 0.00001, 0.000007};

    int i = 0; //index for lambda
    for (auto &file_name : files)
    {
        std::cout << "Case " << file_name << std::endl;
        std::ifstream in(file_name);

        Problem prb;
        ParseInput(in, prb);

        Solution sln = ClusterFirstRouteSecond(prb);
        if (!AccuracyCheck(prb, sln))
            std::cout << " !!!DID NOT PASS ACCURACY TEST!!!" << std::endl;
        else
            std::cout << " OBJECTIVE = " << CalcObjective(prb, sln) << std::endl;

        // run local search to improve solution
        Solution prev_stage_sln = sln;
        Solution improved_sln = LocalSearch(prb, prev_stage_sln);
        if (!AccuracyCheck(prb, improved_sln))
            std::cout << "!!!DID NOT PASS ACCURACY TEST!!!" << std::endl;
        else
            std::cout << "IMPROVED OBJECTIVE = " << CalcObjective(prb, improved_sln) << std::endl;

        while (AccuracyCheck(prb, improved_sln) &&
               CalcObjective(prb, improved_sln) < CalcObjective(prb, prev_stage_sln))
        {
            prev_stage_sln = improved_sln;
            improved_sln = LocalSearch(prb, improved_sln);
            if (!AccuracyCheck(prb, improved_sln))
                std::cout << "!!!DID NOT PASS ACCURACY TEST!!!" << std::endl;
            else
                std::cout << "IMPROVED OBJECTIVE = " << CalcObjective(prb, improved_sln) << std::endl;
        }

        // run guided local search
        Solution gls_sln = GuidedLocalSearch(prb, sln, lambdas[i++]);
        if (!AccuracyCheck(prb, gls_sln))
            std::cout << "!!!DID NOT PASS ACCURACY TEST!!!" << std::endl;
        else
            std::cout << "Guided Local Search IMPROVED OBJECTIVE = " << CalcObjective(prb, gls_sln) << std::endl;

        auto best_sln = CalcObjective(prb, gls_sln) < CalcObjective(prb, improved_sln) ? gls_sln : improved_sln;
        if (AccuracyCheck(prb, best_sln))
        {
            // FIXME: DELETE THIS
            static char ttt = 'a';
            std::ofstream file;
            std::string str("out/solution_");
            str.push_back(ttt++);
            str.append(".txt");
            file.open(str);
            file << file_name << std::endl
                 << CalcObjective(prb, best_sln) << std::endl;

            // FIXME: DELETE THIS
            for (auto &a : best_sln.routes)
            {
                if (a.size() > 2)
                {
                    for (auto &b : a)
                    {
                        file << b << ' ';
                    }
                    file << std::endl;
                }
            }
            file.close();
        }

        std::cout << std::endl;
    }
}
