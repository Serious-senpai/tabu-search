#pragma once

#include <vector>

#include "helpers.cpp"

const unsigned HELD_KARP_LIMIT = 17;
const unsigned GA_POPULATION_SIZE = 100;
const unsigned GA_GENERATIONS_COUNT = 150;

const double GA_MUTATION_RATE = 0.4;

std::pair<double, unsigned> __held_karp_solve(
    const unsigned bitmask,
    const unsigned city,
    const std::vector<std::vector<double>> &distances,
    std::vector<std::vector<std::pair<double, unsigned>>> &dp,
    const unsigned level = 0)
{
    if (dp[bitmask][city].first != -1.0)
    {
        return dp[bitmask][city];
    }

    if (bitmask & (1 << city))
    {
        return dp[bitmask][city] = __held_karp_solve(bitmask - (1 << city), city, distances, dp);
    }

    if (bitmask & 1)
    {
        return dp[bitmask][city] = __held_karp_solve(bitmask - 1, city, distances, dp);
    }

    unsigned n = distances.size();
    std::pair<double, unsigned> result = {-1.0, n};
    for (unsigned i = 1; i < n; i++)
    {
        if (bitmask & (1 << i))
        {
            auto before = __held_karp_solve(bitmask - (1 << i), i, distances, dp, level + 1);
            double d = before.first + distances[i][city];
            if (d < result.first || result.first == -1.0)
            {
                result = {d, i};
            }
        }
    }

    return dp[bitmask][city] = result;
}

std::pair<double, std::vector<unsigned>> __held_karp(const std::vector<std::vector<double>> &distances, const unsigned first)
{
    // https://en.wikipedia.org/wiki/Held-Karp_algorithm
    unsigned n = distances.size();
    std::vector<std::vector<std::pair<double, unsigned>>> dp(1 << n, std::vector<std::pair<double, unsigned>>(n, {-1.0, n}));
    for (unsigned end = 1; end < n; end++)
    {
        dp[0][end] = {distances[0][end], 0};
    }

    unsigned path_end = -1, bitmask = (1 << n) - 2;
    std::pair<double, unsigned> distance_end = {1.0e+9, -1};
    for (unsigned end = 1; end < n; end++)
    {
        auto r = __held_karp_solve(bitmask, end, distances, dp);
        r.first += distances[0][end];
        if (r < distance_end)
        {
            distance_end = r;
            path_end = end;
        }
    }

    bitmask -= (1 << path_end);
    std::vector<unsigned> path = {0, path_end};
    while (bitmask > 0)
    {
        auto r = __held_karp_solve(bitmask, path_end, distances, dp);
        path_end = r.second;
        bitmask -= 1 << path_end;
        path.push_back(path_end);
    }

    rotate_to_first(path, first);
    return {distance_end.first, path};
}

std::pair<std::vector<unsigned>, std::vector<unsigned>> crossover(const std::vector<unsigned> &first, const std::vector<unsigned> &second)
{
    unsigned n = first.size();
    if (n != second.size())
    {
        throw std::invalid_argument(format("Crossover of paths with different lengths: %d and %d", n, second.size()));
    }

    std::vector<unsigned> first_child(n, -1), second_child(n, -1);
    unsigned crossover_point = random_int(1, n - 1);

    std::vector<bool> in_first_child(n);
    for (unsigned i = 0; i < n; i++)
    {
        if (i < crossover_point)
        {
            first_child[i] = first[i];
            in_first_child[first[i]] = true;
        }
        else
        {
            second_child[i] = first[i];
        }
    }

    unsigned first_offset = crossover_point, second_offset = 0;
    for (unsigned i = 0; i < n; i++)
    {
        if (!in_first_child[second[i]])
        {
            first_child[first_offset] = second[i];
            first_offset++;
        }
        else
        {
            second_child[second_offset] = second[i];
            second_offset++;
        }
    }

    return {first_child, second_child};
}

void mutate(std::vector<unsigned> &individual)
{
    unsigned n = individual.size(),
             first = random_int(0, n - 1),
             second = random_int(0, n - 1);
    while (first == second)
    {
        first = random_int(0, n - 1);
        second = random_int(0, n - 1);
    }

    std::swap(individual[first], individual[second]);
}

double evaluate(const std::vector<unsigned> &individual, const std::vector<std::vector<double>> &distances)
{
    double result = 0.0;
    unsigned n = individual.size();
    for (unsigned i = 0; i < n; i++)
    {
        unsigned current = individual[i], next = individual[(i + 1) % n];
        result += distances[current][next];
    }

    return result;
}

std::pair<double, std::vector<unsigned>> tsp_solver(
    const std::vector<std::pair<double, double>> &cities,
    const unsigned first = 0,
    const std::optional<std::vector<unsigned>> &heuristic_hint = std::optional<std::vector<unsigned>>())
{
    unsigned n = cities.size();
    if (n == 0)
    {
        throw std::invalid_argument("Empty TSP map");
    }

    if (n == 1)
    {
        std::vector<unsigned> path = {0};
        rotate_to_first(path, first);
        return {0.0, path};
    }

    std::vector<std::vector<double>> distances(n, std::vector<double>(n, 0.0));
    for (unsigned i = 0; i < n; i++)
    {
        for (unsigned j = i + 1; j < n; j++)
        {
            double dx = cities[i].first - cities[j].first,
                   dy = cities[i].second - cities[j].second;
            distances[i][j] = distances[j][i] = sqrt_impl(dx * dx + dy * dy);
        }
    }

    if (n == 2)
    {
        std::vector<unsigned> path = {0, 1};
        rotate_to_first(path, first);
        return {2 * distances[0][1], path};
    }

    if (n == 3)
    {
        std::vector<unsigned> path = {0, 1, 2};
        rotate_to_first(path, first);
        return {distances[0][1] + distances[1][2] + distances[2][0], path};
    }

    if (n <= HELD_KARP_LIMIT)
    {
        return __held_karp(distances, first);
    }
    else
    {
        std::vector<std::vector<unsigned>> population;
        try
        {
            population.push_back(heuristic_hint.value());
        }
        catch (std::bad_optional_access &e)
        {
            // pass
        }

        while (population.size() < GA_POPULATION_SIZE)
        {
            std::vector<unsigned> individual;
            for (unsigned j = 0; j < n; j++)
            {
                individual.push_back(j);
            }

            std::shuffle(individual.begin(), individual.end(), rng);
            population.push_back(individual);
        }

        double result_cost = evaluate(population[0], distances);
        std::vector<unsigned> result = population[0];
        for (unsigned i = 1; i < GA_POPULATION_SIZE; i++)
        {
            auto cost = evaluate(population[i], distances);
            if (cost < result_cost)
            {
                result_cost = cost;
                result = population[i];
            }
        }

        for (unsigned generation = 0; generation < GA_GENERATIONS_COUNT; generation++)
        {
            while (population.size() < 2 * GA_POPULATION_SIZE)
            {
                // Select 2 random parents
                unsigned first = random_int(0, population.size() - 1),
                         second = random_int(0, population.size() - 1);
                while (first == second)
                {
                    first = random_int(0, population.size() - 1);
                    second = random_int(0, population.size() - 1);
                }

                auto results = crossover(population[first], population[second]);
                if (random_double(0.0, 1.0) < GA_MUTATION_RATE)
                {
                    mutate(results.first);
                }
                if (random_double(0.0, 1.0) < GA_MUTATION_RATE)
                {
                    mutate(results.second);
                }

                population.push_back(results.first);
                population.push_back(results.second);
            }

            std::sort(
                population.begin(), population.end(),
                [&distances](std::vector<unsigned> &f, std::vector<unsigned> &s)
                { return evaluate(f, distances) < evaluate(s, distances); });

            while (population.size() > GA_POPULATION_SIZE)
            {
                population.pop_back();
            }

            auto cost = evaluate(population[0], distances);
            if (cost < result_cost)
            {
                result_cost = cost;
                result = population[0];
            }
        }

        rotate_to_first(result, first);
        return {result_cost, result};
    }
}