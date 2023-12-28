#pragma once

#include <algorithm>
#include <set>
#include <vector>
#ifdef DEBUG
#include <iostream>
#endif

#include "../../../utils/helpers.cpp"

typedef std::pair<std::vector<std::vector<unsigned>>, std::vector<std::vector<std::vector<unsigned>>>> solution;

namespace neighborhoods
{
    std::set<solution> technician_technician_swap(const solution &original, const unsigned first_length, const unsigned second_length)
    {
        auto &[truck_paths, _] = original;
        std::set<solution> result;

        for (unsigned first_truck = 0; first_truck < truck_paths.size(); first_truck++)
        {
            auto path_iter = truck_paths.begin() + first_truck;
            auto length = path_iter->size();
            if (length >= 2 + first_length + second_length)
            {
                for (unsigned first_index = 1; first_index < length - first_length; first_index++)
                {
                    for (unsigned second_index = first_index + first_length; second_index < length - second_length; second_index++)
                    {
                        solution copied = original;
                        auto iter = copied.first[first_truck].begin() + first_index;

                        // Copy second segment
                        iter = std::copy(path_iter->begin() + second_index, path_iter->begin() + second_index + second_length, iter);

                        // Copy middle segment
                        iter = std::copy(path_iter->begin() + first_index + first_length, path_iter->begin() + second_index, iter);

                        // Copy first segment
                        iter = std::copy(path_iter->begin() + first_index, path_iter->begin() + first_index + first_length, iter);

#ifdef DEBUG
                        std::cout << "From truck path " << *path_iter << " to " << copied.first[first_truck] << std::endl;
#endif

                        result.insert(copied);
                    }
                }
            }

            for (unsigned second_truck = first_truck + 1; second_truck < truck_paths.size(); second_truck++)
            {
                auto first_path_iter = truck_paths.begin() + first_truck,
                     second_path_iter = truck_paths.begin() + second_truck;

                for (unsigned first_index = 1; first_index < first_path_iter->size() - first_length; first_index++)
                {
                    for (unsigned second_index = 1; second_index < second_path_iter->size() - second_length; second_index++)
                    {
                        solution copied = original;
                        copied.first[first_truck].resize(first_path_iter->size() - first_length + second_length);
                        copied.first[second_truck].resize(second_path_iter->size() - second_length + first_length);

                        auto first_iter = copied.first[first_truck].begin() + first_index;
                        first_iter = std::copy(
                            second_path_iter->begin() + second_index,
                            second_path_iter->begin() + second_index + second_length,
                            first_iter);
                        first_iter = std::copy(
                            first_path_iter->begin() + first_index + first_length,
                            first_path_iter->cend(),
                            first_iter);

                        auto second_iter = copied.first[second_truck].begin() + second_index;
                        second_iter = std::copy(
                            first_path_iter->begin() + first_index,
                            first_path_iter->begin() + first_index + first_length,
                            second_iter);
                        second_iter = std::copy(
                            second_path_iter->begin() + second_index + second_length,
                            second_path_iter->cend(),
                            second_iter);

#ifdef DEBUG
                        std::cout << "From truck paths " << *first_path_iter << " and " << *second_path_iter << " to " << copied.first[first_truck] << " and " << copied.first[second_truck] << std::endl;
#endif

                        result.insert(copied);
                    }
                }
            }
        }

        return result;
    }

    std::set<solution> technician_drone_swap(const solution &original, const unsigned first_length, const unsigned second_length)
    {
        return std::set<solution>();
    }

    std::set<solution> drone_drone_swap(const solution &original, const unsigned first_length, const unsigned second_length)
    {
        auto &[_, drone_paths] = original;
        std::set<solution> result;

        for (unsigned first_drone = 0; first_drone < drone_paths.size(); first_drone++)
        {
            for (unsigned first_path = 0; first_path < drone_paths[first_drone].size(); first_path++)
            {
                auto path_iter = drone_paths[first_drone].begin() + first_path;
                auto length = path_iter->size();
                if (length >= 2 + first_length + second_length)
                {
                    for (unsigned first_index = 1; first_index < length - first_length; first_index++)
                    {
                        for (unsigned second_index = first_index + first_length; second_index < length - second_length; second_index++)
                        {
                            solution copied = original;
                            auto iter = copied.second[first_drone][first_path].begin() + first_index;

                            // Copy second segment
                            iter = std::copy(path_iter->begin() + second_index, path_iter->begin() + second_index + second_length, iter);

                            // Copy middle segment
                            iter = std::copy(path_iter->begin() + first_index + first_length, path_iter->begin() + second_index, iter);

                            // Copy first segment
                            iter = std::copy(path_iter->begin() + first_index, path_iter->begin() + first_index + first_length, iter);

#ifdef DEBUG
                            std::cout << "From drone path " << *path_iter << " to " << copied.second[first_drone][first_path] << std::endl;
#endif

                            result.insert(copied);
                        }
                    }
                }

                for (unsigned second_drone = 0; second_drone < drone_paths.size(); second_drone++)
                {
                    for (unsigned second_path = 0; second_path < drone_paths[second_drone].size(); second_path++)
                    {
                        auto first_path_iter = drone_paths[first_drone].begin() + first_path,
                             second_path_iter = drone_paths[second_drone].begin() + second_path;

                        for (unsigned first_index = 1; first_index < first_path_iter->size() - first_length; first_index++)
                        {
                            for (unsigned second_index = 1; second_index < second_path_iter->size() - second_length; second_index++)
                            {
                                solution copied = original;
                                copied.second[first_drone][first_path].resize(first_path_iter->size() - first_length + second_length);
                                copied.second[second_drone][second_path].resize(second_path_iter->size() - second_length + first_length);

                                auto first_iter = copied.second[first_drone][first_path].begin() + first_index;
                                first_iter = std::copy(
                                    second_path_iter->begin() + second_index,
                                    second_path_iter->begin() + second_index + second_length,
                                    first_iter);
                                first_iter = std::copy(
                                    first_path_iter->begin() + first_index + first_length,
                                    first_path_iter->cend(),
                                    first_iter);

                                auto second_iter = copied.second[second_drone][second_path].begin() + second_index;
                                second_iter = std::copy(
                                    first_path_iter->begin() + first_index,
                                    first_path_iter->begin() + first_index + first_length,
                                    second_iter);
                                second_iter = std::copy(
                                    second_path_iter->begin() + second_index + second_length,
                                    second_path_iter->cend(),
                                    second_iter);

#ifdef DEBUG
                                std::cout << "From drone paths " << *first_path_iter << " and " << *second_path_iter << " to " << copied.second[first_drone][first_path] << " and " << copied.second[second_drone][second_path] << std::endl;
#endif

                                result.insert(copied);
                            }
                        }
                    }
                }
            }
        }

        return result;
    }

    std::set<solution> swap(const solution &original, const unsigned first_length, const unsigned second_length)
    {
        std::set<solution> result;
        for (auto &solution : technician_technician_swap(original, first_length, second_length))
        {
            result.insert(solution);
        }
        for (auto &solution : technician_drone_swap(original, first_length, second_length))
        {
            result.insert(solution);
        }
        for (auto &solution : drone_drone_swap(original, first_length, second_length))
        {
            result.insert(solution);
        }

        return result;
    }
}