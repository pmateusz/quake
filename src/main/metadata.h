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

#ifndef QUAKE_METADATA_H
#define QUAKE_METADATA_H

#include <unordered_map>

#include <nlohmann/json.hpp>

#include <boost/config.hpp>
#include <boost/optional.hpp>
#include <boost/date_time/posix_time/time_period.hpp>

#include "util/json.h"

namespace quake {

    class Metadata {
    public:
        enum class Property {
            None,
            ScenarioGenerator,
            ScenariosNumber,
            SolutionMethod,
            TargetTrafficIndex,
            ObservationPeriod,
            SwitchDuration,
            SolutionType,
            GapLimit,
            Gap,
            TimeLimit,
            IntervalStep,
            Epsilon
        };

        enum class SolutionType {
            None,
            Reference,
            Test
        };

        enum class ErrorModel {
            None,
            Coregionalization,
            PastErrorReplication,
            CorrelatedErrorSimulation,
            IndependentErrorSimulation
        };

        enum class SolutionMethod {
            None,
            Deterministic,
            SampleAverageApproximation,
            ConditionalValueAtRisk,
            UncorrelatedCrossMoment
        };

        Metadata();

        explicit Metadata(std::unordered_map<Metadata::Property, nlohmann::json> properties);

        template<typename ValueType>
        void SetProperty(Metadata::Property property, ValueType value);

        template<typename ValueType>
        boost::optional<ValueType> GetProperty(Metadata::Property property) const;

    private:
        friend void to_json(nlohmann::json &json, const Metadata &metadata);

        friend void from_json(const nlohmann::json &json, Metadata &metadata);

        std::unordered_map<Metadata::Property, nlohmann::json> properties_;
    };

    void to_json(nlohmann::json &json, const Metadata &metadata);

    void from_json(const nlohmann::json &json, Metadata &metadata);

    void to_json(nlohmann::json &json, Metadata::Property property);

    void from_json(const nlohmann::json &json, Metadata::Property &property);

    void to_json(nlohmann::json &json, Metadata::SolutionMethod solution_method);

    void to_json(nlohmann::json &json, Metadata::SolutionType solution_type);
}

std::ostream &operator<<(std::ostream &out, quake::Metadata::Property property);

std::ostream &operator<<(std::ostream &out, quake::Metadata::ErrorModel model);

std::ostream &operator<<(std::ostream &out, quake::Metadata::SolutionMethod method);

std::ostream &operator<<(std::ostream &out, quake::Metadata::SolutionType method);

template<typename ValueType>
void quake::Metadata::SetProperty(quake::Metadata::Property property, ValueType value) {
    properties_.emplace(property, value);
}

template<typename ValueType>
boost::optional<ValueType> quake::Metadata::GetProperty(const quake::Metadata::Property property) const {
    const auto find_it = properties_.find(property);
    if (find_it == std::cend(properties_)) {
        return boost::none;
    }

    ValueType value = util::from_json<ValueType>(find_it->second);
    return boost::make_optional(std::move(value));
}

template<>
boost::optional<nlohmann::json> quake::Metadata::GetProperty(quake::Metadata::Property property) const;

#endif //QUAKE_METADATA_H
