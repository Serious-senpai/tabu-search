#pragma once

#include <string>
#include <vector>
#ifdef DEBUG
#include <iostream>
#endif

#include "../../utils/helpers.cpp"

namespace config
{
    struct TruckConfig
    {
        const double maximum_velocity;
        const double m_t; // weird attribute
        const std::vector<double> coefficients;

        TruckConfig(
            const double maximum_velocity,
            const double m_t,
            const std::vector<double> &coefficients)
            : maximum_velocity(maximum_velocity), m_t(m_t), coefficients(coefficients) {}

        static TruckConfig *instance;
        static void import(
            const double maximum_velocity,
            const double m_t,
            const std::vector<double> &coefficients);
    };

    TruckConfig *TruckConfig::instance;
    void TruckConfig::import(
        const double maximum_velocity,
        const double m_t,
        const std::vector<double> &coefficients)
    {
#ifdef DEBUG
        std::cout << "Importing TruckConfig" << std::endl;
#endif

        TruckConfig::instance = new TruckConfig(maximum_velocity, m_t, coefficients);
    }

    struct BaseDroneConfig
    {
        const double takeoff_speed;
        const double cruise_speed;
        const double landing_speed;
        const double altitude;
        const double capacity;
        const double battery;
        const std::string speed_type;
        const std::string range;

        BaseDroneConfig(
            const double takeoff_speed,
            const double cruise_speed,
            const double landing_speed,
            const double altitude,
            const double capacity,
            const double battery,
            const std::string &speed_type,
            const std::string &range)
            : takeoff_speed(takeoff_speed),
              cruise_speed(cruise_speed),
              landing_speed(landing_speed),
              altitude(altitude),
              capacity(capacity),
              battery(battery),
              speed_type(speed_type),
              range(range) {}

        virtual double takeoff_power(const double weight) = 0;
        virtual double landing_power(const double weight) = 0;
        virtual double cruise_power(const double weight) = 0;
    };

    struct DroneLinearConfig : BaseDroneConfig
    {
        const double beta;
        const double gamma;

        DroneLinearConfig(
            const double takeoff_speed,
            const double cruise_speed,
            const double landing_speed,
            const double altitude,
            const double capacity,
            const double battery,
            const std::string &speed_type,
            const std::string &range,
            const double beta,
            const double gamma)
            : BaseDroneConfig(
                  takeoff_speed,
                  cruise_speed,
                  landing_speed,
                  altitude,
                  capacity,
                  battery,
                  speed_type,
                  range),
              beta(beta),
              gamma(gamma) {}

        double takeoff_power(const double weight)
        {
            return _power(weight);
        }

        double landing_power(const double weight)
        {
            return _power(weight);
        }

        double cruise_power(const double weight)
        {
            return _power(weight);
        }

        static DroneLinearConfig *instance;
        static void import(
            const double takeoff_speed,
            const double cruise_speed,
            const double landing_speed,
            const double altitude,
            const double capacity,
            const double battery,
            const std::string &speed_type,
            const std::string &range,
            const double beta,
            const double gamma);

    private:
        double _power(const double weight)
        {
            return beta * weight + gamma;
        }
    };

    DroneLinearConfig *DroneLinearConfig::instance;
    void DroneLinearConfig::import(
        const double takeoff_speed,
        const double cruise_speed,
        const double landing_speed,
        const double altitude,
        const double capacity,
        const double battery,
        const std::string &speed_type,
        const std::string &range,
        const double beta,
        const double gamma)
    {
#ifdef DEBUG
        std::cout << "Importing DroneLinearConfig" << std::endl;
#endif

        DroneLinearConfig::instance = new DroneLinearConfig(
            takeoff_speed,
            cruise_speed,
            landing_speed,
            altitude,
            capacity,
            battery,
            speed_type,
            range,
            beta,
            gamma);
    }

    struct DroneNonlinearConfig : BaseDroneConfig
    {
        const double k1;
        const double k2;
        const double c1;
        const double c2;
        const double c4;
        const double c5;

        DroneNonlinearConfig(
            const double takeoff_speed,
            const double cruise_speed,
            const double landing_speed,
            const double altitude,
            const double capacity,
            const double battery,
            const std::string &speed_type,
            const std::string &range,
            const double k1,
            const double k2,
            const double c1,
            const double c2,
            const double c4,
            const double c5)
            : BaseDroneConfig(
                  takeoff_speed,
                  cruise_speed,
                  landing_speed,
                  altitude,
                  capacity,
                  battery,
                  speed_type,
                  range),
              k1(k1), k2(k2), c1(c1), c2(c2), c4(c4), c5(c5) {}

        double takeoff_power(const double weight)
        {
            return _vertical_power(takeoff_speed, weight);
        }

        double landing_power(const double weight)
        {
            return _vertical_power(landing_speed, weight);
        }

        double cruise_power(const double weight)
        {
            double w = 1.5 + weight, g = 9.8;
            return (c1 + c2) * pow(sqr(w * g - c5 * (cruise_speed * sqr(cos(M_PI / 18)))) + (c4 * sqr(cruise_speed)), 0.75) + c4 * pow(cruise_speed, 3);
        }

        static DroneNonlinearConfig *instance;
        static void import(
            const double takeoff_speed,
            const double cruise_speed,
            const double landing_speed,
            const double altitude,
            const double capacity,
            const double battery,
            const std::string &speed_type,
            const std::string &range,
            const double k1,
            const double k2,
            const double c1,
            const double c2,
            const double c4,
            const double c5);

    private:
        double _vertical_power(const double speed, const double weight)
        {
            double w = 1.5 + weight, g = 9.8;
            return k1 * w * g * (speed / 2 + sqrt(sqr(speed / 2) + w * g / sqr(k2))) + c2 * (pow(w * g, 1.5));
        }
    };

    DroneNonlinearConfig *DroneNonlinearConfig::instance;
    void DroneNonlinearConfig::import(
        const double takeoff_speed,
        const double cruise_speed,
        const double landing_speed,
        const double altitude,
        const double capacity,
        const double battery,
        const std::string &speed_type,
        const std::string &range,
        const double k1,
        const double k2,
        const double c1,
        const double c2,
        const double c4,
        const double c5)
    {
#ifdef DEBUG
        std::cout << "Importing DroneNonlinearConfig" << std::endl;
#endif

        DroneNonlinearConfig::instance = new DroneNonlinearConfig(
            takeoff_speed,
            cruise_speed,
            landing_speed,
            altitude,
            capacity,
            battery,
            speed_type,
            range,
            k1, k2, c1, c2, c4, c5);
    }

    struct DroneEnduranceConfig
    {
        const std::string speed_type;
        const std::string range;
        const double capacity;
        const double fixed_time;
        const double fixed_distance;
        const double drone_speed;

        DroneEnduranceConfig(
            const std::string &speed_type,
            const std::string &range,
            const double capacity,
            const double fixed_time,
            const double fixed_distance,
            const double drone_speed)
            : speed_type(speed_type),
              range(range),
              capacity(capacity),
              fixed_time(fixed_time),
              fixed_distance(fixed_distance),
              drone_speed(drone_speed) {}

        static DroneEnduranceConfig *instance;
        static void import(
            const std::string &speed_type,
            const std::string &range,
            const double capacity,
            const double fixed_time,
            const double fixed_distance,
            const double drone_speed);
    };

    DroneEnduranceConfig *DroneEnduranceConfig::instance;
    void DroneEnduranceConfig::import(
        const std::string &speed_type,
        const std::string &range,
        const double capacity,
        const double fixed_time,
        const double fixed_distance,
        const double drone_speed)
    {
#ifdef DEBUG
        std::cout << "Importing DroneEnduranceConfig" << std::endl;
#endif

        DroneEnduranceConfig::instance = new DroneEnduranceConfig(
            speed_type,
            range,
            capacity,
            fixed_time,
            fixed_distance,
            drone_speed);
    }

    struct Customer
    {
        const double x;
        const double y;
        const double demand;
        const bool dronable;
        const double drone_service_time;
        const double technician_service_time;

        Customer(
            const double x,
            const double y,
            const double demand,
            const bool dronable,
            const double drone_service_time,
            const double technician_service_time)
            : x(x), y(y), demand(demand), dronable(dronable),
              drone_service_time(drone_service_time), technician_service_time(technician_service_time) {}

        static std::vector<Customer> customers;
        static std::vector<std::vector<double>> distances;
        static void import(
            const std::vector<double> &x,
            const std::vector<double> &y,
            const std::vector<double> &demands,
            const std::vector<bool> &dronable,
            const std::vector<double> &drone_service_time,
            const std::vector<double> &technician_service_time);
    };

    std::vector<Customer> Customer::customers;
    std::vector<std::vector<double>> Customer::distances;
    void Customer::import(
        const std::vector<double> &x,
        const std::vector<double> &y,
        const std::vector<double> &demands,
        const std::vector<bool> &dronable,
        const std::vector<double> &drone_service_time,
        const std::vector<double> &technician_service_time)
    {
        unsigned n = x.size();

#ifdef DEBUG
        std::cout << "Importing " << n << " customers" << std::endl;
#endif

        if (y.size() != n || demands.size() != n || dronable.size() != n)
        {
            throw std::invalid_argument("All arrays must have the same size");
        }

        customers.clear();
        for (unsigned i = 0; i < n; i++)
        {
            customers.emplace_back(x[i], y[i], demands[i], dronable[i], drone_service_time[i], technician_service_time[i]);
        }

        distances.clear();
        distances.resize(n, std::vector<double>(n, 0.0));
        for (unsigned i = 0; i < n; i++)
        {
            for (unsigned j = i + 1; j < n; j++)
            {
                distances[i][j] = distances[j][i] = sqrt_impl(sqr(x[i] - x[j]) + sqr(y[i] - y[j]));
            }
        }
    }

}

const unsigned LINEAR = 0;
const unsigned NONLINEAR = 1;
const unsigned ENDURANCE = 2;

std::vector<double> calculate_drone_arrival_timestamps(
    const std::vector<unsigned> &path,
    const unsigned config_type,
    const double offset)
{
    if (config_type != LINEAR && config_type != NONLINEAR && config_type != ENDURANCE)
    {
        throw std::invalid_argument(format("Invalid config_type = %d", config_type));
    }

    unsigned n = path.size();
    std::vector<double> result = {offset};
    if (config_type == ENDURANCE)
    {
        auto config = config::DroneEnduranceConfig::instance;
        for (unsigned i = 1; i < n; i++)
        {
            result.push_back(result.back() + config::Customer::distances[path[i - 1]][path[i]] / config->drone_speed);
        }
    }
    else
    {
        auto config = config_type == LINEAR ? (config::BaseDroneConfig *)config::DroneLinearConfig::instance
                                            : (config::BaseDroneConfig *)config::DroneNonlinearConfig::instance;

        double vertical_time = config->altitude * (1 / config->takeoff_speed + 1 / config->landing_speed);
        for (unsigned i = 1; i < n; i++)
        {
            double distance = config::Customer::distances[path[i - 1]][path[i]],
                   shift = path[i - 1] == path[i] ? 0.0
                                                  : config::Customer::customers[path[i - 1]].drone_service_time + vertical_time + distance / config->cruise_speed;

            result.push_back(result.back() + shift);
        }
    }

    return result;
}

std::vector<double> calculate_technician_arrival_timestamps(
    const std::vector<unsigned> &path)
{
    unsigned n = path.size();
    std::vector<double> result = {0.0};

    unsigned coefficients_index = 0;
    double current_within_timespan = 0.0;
    for (unsigned i = 1; i < n; i++)
    {
        double timestamp = result.back() + config::Customer::customers[path[i - 1]].technician_service_time;
        current_within_timespan += config::Customer::customers[path[i - 1]].technician_service_time;
        while (current_within_timespan >= 3600.0)
        {
            current_within_timespan -= 3600.0;
            coefficients_index++;
        }

        double distance = config::Customer::distances[path[i - 1]][path[i]];
        while (distance > 0.0)
        {
            double velocity = config::TruckConfig::instance->maximum_velocity * config::TruckConfig::instance->coefficients[coefficients_index % config::TruckConfig::instance->coefficients.size()],
                   time_shift = std::min(distance / velocity, 3600.0 - current_within_timespan);

            timestamp += time_shift;
            distance -= velocity * time_shift;
            current_within_timespan += time_shift;
            if (current_within_timespan >= 3600.0)
            {
                current_within_timespan -= 3600.0;
                coefficients_index++;
            }
        }

        result.push_back(timestamp);
    }

    return result;
}

double calculate_drone_total_waiting_time(
    const std::vector<unsigned> &path,
    const std::vector<double> &arrival_timestamps)
{
    unsigned n = path.size();
    if (arrival_timestamps.size() != n)
    {
        throw std::invalid_argument(format("arrival_timestamps.size() = %d != %d = path.size()", arrival_timestamps.size(), n));
    }

    double result = 0.0;
    for (unsigned i = 1; i < n - 1; i++)
    {
        result += arrival_timestamps.back() - arrival_timestamps[i] - config::Customer::customers[path[i]].drone_service_time;
    }

    return result;
}

double calculate_technician_total_waiting_time(
    const std::vector<unsigned> &path,
    const std::vector<double> &arrival_timestamps)
{
    unsigned n = path.size();
    if (arrival_timestamps.size() != n)
    {
        throw std::invalid_argument(format("arrival_timestamps.size() = %d != %d = path.size()", arrival_timestamps.size(), n));
    }

    double result = 0.0;
    for (unsigned i = 1; i < n - 1; i++)
    {
        result += arrival_timestamps.back() - arrival_timestamps[i] - config::Customer::customers[path[i]].technician_service_time;
    }

    return result;
}