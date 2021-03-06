//
// Copyright 2019 Mateusz Polnik
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <ostream>

#include <glog/logging.h>

#include "metadata.h"

std::ostream &operator<<(std::ostream &out, quake::Metadata::Property property) {
    switch (property) {
        case quake::Metadata::Property::None:
            out << "";
            break;
        case quake::Metadata::Property::ScenarioGenerator:
            out << "scenario_generator";
            break;
        case quake::Metadata::Property::ScenariosNumber:
            out << "scenarios_number";
            break;
        case quake::Metadata::Property::SolutionMethod:
            out << "solution_method";
            break;
        case quake::Metadata::Property::TargetTrafficIndex:
            out << "target_traffic_index";
            break;
        case quake::Metadata::Property::ObservationPeriod:
            out << "observation_period";
            break;
        case quake::Metadata::Property::SwitchDuration:
            out << "switch_duration";
            break;
        case quake::Metadata::Property::SolutionType:
            out << "solution_type";
            break;
        case quake::Metadata::Property::Gap:
            out << "gap";
            break;
        case quake::Metadata::Property::GapLimit:
            out << "gap_limit";
            break;
        case quake::Metadata::Property::TimeLimit:
            out << "time_limit";
            break;
        case quake::Metadata::Property::IntervalStep:
            out << "interval_step";
            break;
        case quake::Metadata::Property::Epsilon:
            out << "epsilon";
            break;
        default:
            LOG(FATAL) << "Conversion to string for Property " << static_cast<std::size_t>(property) << " is not defined";
    }

    return out;
}

std::ostream &operator<<(std::ostream &out, quake::Metadata::ErrorModel model) {
    switch (model) {
        case quake::Metadata::ErrorModel::None:
            out << "";
            break;
        case quake::Metadata::ErrorModel::Coregionalization:
            out << "coregionalization";
            break;
        case quake::Metadata::ErrorModel::PastErrorReplication:
            out << "past_error_replication";
            break;
        case quake::Metadata::ErrorModel::CorrelatedErrorSimulation:
            out << "correlated_error_simulation";
            break;
        case quake::Metadata::ErrorModel::IndependentErrorSimulation:
            out << "independent_error_simulation";
            break;
        default:
            LOG(FATAL) << "Conversion to string for ScenarioGenerator " << static_cast<std::size_t>(model) << " is not defined";
    }

    return out;
}

std::ostream &operator<<(std::ostream &out, quake::Metadata::SolutionMethod method) {
    switch (method) {
        case quake::Metadata::SolutionMethod::None:
            out << "";
            break;
        case quake::Metadata::SolutionMethod::Deterministic:
            out << "deterministic";
            break;
        case quake::Metadata::SolutionMethod::SampleAverageApproximation:
            out << "sample_average_approximation";
            break;
        case quake::Metadata::SolutionMethod::ConditionalValueAtRisk:
            out << "conditional_value_at_risk";
            break;
        case quake::Metadata::SolutionMethod::UncorrelatedCrossMoment:
            out << "uncorrelated_cross_moment";
            break;
        default:
            LOG(FATAL) << "Conversion to string for SolutionMethod " << static_cast<std::size_t>(method) << " is not defined";
    }

    return out;
}

std::ostream &operator<<(std::ostream &out, quake::Metadata::SolutionType solution_type) {
    switch (solution_type) {
        case quake::Metadata::SolutionType::None:
            out << "";
            break;
        case quake::Metadata::SolutionType::Reference:
            out << "reference";
            break;
        case quake::Metadata::SolutionType::Test:
            out << "test";
            break;
        default:
            LOG(FATAL) << "Conversion to string for SolutionType " << static_cast<std::size_t>(solution_type) << " is not defined";
    }

    return out;
}

void quake::from_json(const nlohmann::json &json, quake::Metadata::Property &property) {
    const auto property_name = json.get<std::string>();

    if (property_name.empty()) {
        property = quake::Metadata::Property::None;
        return;
    }

    if (property_name == "scenario_generator") {
        property = quake::Metadata::Property::ScenarioGenerator;
        return;
    }

    if (property_name == "scenarios_number") {
        property = quake::Metadata::Property::ScenariosNumber;
        return;
    }

    if (property_name == "solution_method") {
        property = quake::Metadata::Property::SolutionMethod;
        return;
    }

    if (property_name == "target_traffic_index") {
        property = quake::Metadata::Property::TargetTrafficIndex;
        return;
    }

    if (property_name == "observation_period") {
        property = quake::Metadata::Property::ObservationPeriod;
        return;
    }

    if (property_name == "switch_duration") {
        property = quake::Metadata::Property::SwitchDuration;
        return;
    }

    if (property_name == "solution_type") {
        property = quake::Metadata::Property::SolutionType;
        return;
    }

    if (property_name == "gap") {
        property = quake::Metadata::Property::Gap;
        return;
    }

    if (property_name == "gap_limit") {
        property = quake::Metadata::Property::GapLimit;
        return;
    }

    if (property_name == "time_limit") {
        property = quake::Metadata::Property::TimeLimit;
        return;
    }

    if (property_name == "interval_step") {
        property = quake::Metadata::Property::IntervalStep;
        return;
    }

    if (property_name == "epsilon") {
        property = quake::Metadata::Property::Epsilon;
        return;
    }

    LOG(FATAL) << "Conversion from string for Property " << property_name << " is not defined";
}

quake::Metadata::Metadata() = default;

quake::Metadata::Metadata(std::unordered_map<quake::Metadata::Property, nlohmann::json> properties)
        : properties_{std::move(properties)} {}


template<>
boost::optional<nlohmann::json> quake::Metadata::GetProperty(const quake::Metadata::Property property) const {
    const auto find_it = properties_.find(property);
    if (find_it == std::cend(properties_)) {
        return boost::none;
    }

    return boost::make_optional(find_it->second);
}

void quake::to_json(nlohmann::json &json, const quake::Metadata &metadata) {
    json = metadata.properties_;
}

void quake::from_json(const nlohmann::json &json, quake::Metadata &metadata) {
    auto properties = json.get<std::unordered_map<Metadata::Property, nlohmann::json> >();
    metadata = quake::Metadata(std::move(properties));
}

quake::Metadata quake::Metadata::Trim(const boost::posix_time::time_period &time_period) const {
    quake::Metadata trimmed_metadata{*this};

    const auto observation_period_opt = trimmed_metadata.GetProperty<boost::posix_time::time_period>(Property::ObservationPeriod);
    if (observation_period_opt) {
        CHECK_LE(observation_period_opt->begin(), time_period.begin());
        CHECK_LE(time_period.end(), observation_period_opt->end());

        trimmed_metadata.SetProperty(Property::ObservationPeriod, time_period);
    }

    return trimmed_metadata;
}

template<typename ValueType>
std::string to_string(ValueType value) {
    std::stringstream stream;
    stream << value;
    return stream.str();
}

void quake::to_json(nlohmann::json &json, quake::Metadata::Property property) {
    json = to_string(property);
}

void quake::to_json(nlohmann::json &json, quake::Metadata::SolutionMethod solution_method) {
    json = to_string(solution_method);
}

void quake::to_json(nlohmann::json &json, quake::Metadata::SolutionType solution_type) {
    json = to_string(solution_type);
}
