#pragma once

#include <algorithm>
#include <chrono>
#include <random>
#include <stdexcept>
#include <string>

template <typename... Args>
std::string format(const std::string &format, Args... args)
{
    // https://stackoverflow.com/a/26221725
    int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) + 1; // Extra space for '\0'
    if (size_s <= 0)
    {
        throw std::runtime_error("Error during formatting.");
    }
    auto size = static_cast<size_t>(size_s);
    std::unique_ptr<char[]> buf(new char[size]);
    std::snprintf(buf.get(), size, format.c_str(), args...);
    return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());

double random_double(const double l, const double r)
{
    std::uniform_real_distribution<double> unif(l, r);
    return unif(rng);
}

unsigned random_int(const unsigned l, const unsigned r)
{
    std::uniform_int_distribution<unsigned> unif(l, r);
    return unif(rng);
}

void rotate_to_first(std::vector<unsigned> &path, const unsigned first)
{
    auto first_iter = std::find(path.begin(), path.end(), first);
    if (first_iter == path.end())
    {
        throw std::invalid_argument(format("First city %d not found in path", first));
    }

    std::rotate(path.begin(), first_iter, path.end());
}

double sqrt_impl(const double value)
{
    if (value < 0)
    {
        throw std::out_of_range(format("Attempted to calculate square root of %lf", value));
    }

    if (value == 0.0)
    {
        return 0.0;
    }

    double low = 0.0, high = value;
    while (high - low > 1.0e-9)
    {
        double mid = (low + high) / 2;
        if (mid * mid < value)
        {
            low = mid;
        }
        else
        {
            high = mid;
        }
    }

    return high;
}

template <typename T>
const T &min(const T &_x, const T &_y, const T &_z)
{
    return std::min(_x, std::min(_y, _z));
}

double round(const double value, const unsigned precision)
{
    double factor = std::pow(10, precision);
    return std::round(value * factor) / factor;
}

double sqr(const double value)
{
    return value * value;
}

#ifdef DEBUG
#include <iostream>

template <typename T>
void list_elements(std::ostream &stream, const std::vector<T> &_v)
{
    int n = _v.size();
    for (int i = 0; i < n; i++)
    {
        stream << _v[i];
        if (i < n - 1)
        {
            stream << ", ";
        }
    }
}

template <typename T>
std::ostream &operator<<(std::ostream &stream, const std::vector<T> &_v)
{
    stream << "[";
    list_elements(stream, _v);
    stream << "]";

    return stream;
}

template <typename T, typename Compare, typename Alloc>
const std::ostream &operator<<(std::ostream &stream, const std::set<T, Compare, Alloc> &_s)
{
    std::vector<T> _v(_s.begin(), _s.end());

    stream << "{";
    list_elements(stream, _v);
    stream << "}";

    return stream;
}

template <typename T, typename Compare, typename Alloc>
std::ostream &operator<<(std::ostream &stream, const std::multiset<T, Compare, Alloc> &_s)
{
    std::vector<T> _v(_s.begin(), _s.end());

    stream << "{";
    list_elements(stream, _v);
    stream << "}";

    return stream;
}

template <typename T1, typename T2, typename Compare, typename Alloc>
std::ostream &operator<<(std::ostream &stream, const std::map<T1, T2, Compare, Alloc> &_m)
{
    std::vector<std::pair<T1, T2>> _v(_m.begin(), _m.end());

    stream << "{";
    int n = _v.size();
    for (int i = 0; i < n; i++)
    {
        stream << _v[i].first << ": " << _v[i].second;
        if (i < n - 1)
        {
            stream << ", ";
        }
    }
    stream << "}";

    return stream;
}

template <typename T1, typename T2>
std::ostream &operator<<(std::ostream &stream, const std::pair<T1, T2> &_p)
{
    stream << "<" << _p.first << ", " << _p.second << ">";
    return stream;
}
#endif