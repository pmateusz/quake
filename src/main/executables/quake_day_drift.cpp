#include <cstdlib>
#include <functional>
#include <algorithm>
#include <list>

#include <glog/logging.h>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <pykep/core_functions/par2ic.h>
#include <pykep/third_party/libsgp4/DateTime.h>
#include <pykep/third_party/libsgp4/Eci.h>
#include <pykep/third_party/libsgp4/Vector.h>
#include <pykep/third_party/libsgp4/Util.h>
#include <pykep/third_party/libsgp4/Observer.h>
#include <pykep/third_party/libsgp4/CoordTopocentric.h>

#include "util/hash.h"
#include "kepler_elements.h"
#include "ground_station.h"

Eci CreateEci(const DateTime &data_time,
              const boost::numeric::ublas::vector<double> &cartesian_position,
              const boost::numeric::ublas::vector<double> &cartesian_velocity) {
    return {data_time,
            Vector{cartesian_position[0], cartesian_position[1], cartesian_position[2]},
            Vector{cartesian_velocity[0], cartesian_velocity[1], cartesian_velocity[2]}};
}

DateTime CreateDateTime(const boost::posix_time::ptime &date_time) {
    return {date_time.date().year(),
            date_time.date().month(),
            date_time.date().day(),
            static_cast<int>(date_time.time_of_day().hours()),
            static_cast<int>(date_time.time_of_day().minutes()),
            static_cast<int>(date_time.time_of_day().seconds())};
}

void PropagateOrbitalElements(const quake::KeplerElements &initial_position,
                              const boost::posix_time::time_duration &elapsed_time,
                              boost::numeric::ublas::vector<double> &output_elements) {
    const auto current_ascending_node_longitude = Util::WrapTwoPI(
            initial_position.AscendingNodeLongitude() + initial_position.AscendingNodeLongitudeVariation() * elapsed_time.total_seconds());
    const auto current_theta = Util::WrapTwoPI(
            initial_position.TrueAnomaly() + initial_position.CircularOrbitVelocity() * elapsed_time.total_seconds());

    output_elements(0) = initial_position.SemimajorAxis();
    output_elements(1) = initial_position.Eccentricity();
    output_elements(2) = initial_position.Inclination();
    output_elements(3) = current_ascending_node_longitude;
    output_elements(4) = initial_position.PeriapsisArgument();
    output_elements(5) = current_theta;
}

class CommunicationBandBuilder {
public:
    CommunicationBandBuilder()
            : last_contact_{boost::posix_time::not_a_date_time} {}

    CommunicationBandBuilder(const CommunicationBandBuilder &other) {
        last_contact_ = other.last_contact_;
        communication_begin_ = other.communication_begin_;
        communication_end_ = other.communication_end_;
    }

    CommunicationBandBuilder(CommunicationBandBuilder &&other) {
        last_contact_ = other.last_contact_;
        communication_begin_ = std::move(other.communication_begin_);
        communication_end_ = std::move(other.communication_end_);
    }

    CommunicationBandBuilder &operator=(const CommunicationBandBuilder &other) {
        last_contact_ = other.last_contact_;
        communication_begin_ = other.communication_begin_;
        communication_end_ = other.communication_end_;
        return *this;
    }

    CommunicationBandBuilder &operator=(CommunicationBandBuilder &&other) {
        last_contact_ = other.last_contact_;
        communication_begin_ = std::move(other.communication_begin_);
        communication_end_ = std::move(other.communication_end_);
        return *this;
    }

    bool ConvergedAt(const boost::posix_time::ptime &epoch) const {
        const auto date_duration = epoch.date() - last_contact_.date();
        return date_duration.days() > 1;
    }

    boost::posix_time::time_duration GetBeginDrift() const {
        std::vector<boost::posix_time::time_duration> normalized_time_durations;

        for (const auto &entry : communication_begin_) {
            normalized_time_durations.emplace_back(Normalize(entry.second).time_of_day());
        }

        const auto minmax_it_pair = std::minmax_element(std::cbegin(normalized_time_durations), std::cend(normalized_time_durations));
        return (*minmax_it_pair.second - *minmax_it_pair.first);
    }

    boost::posix_time::time_duration GetMinNormalizedBegin() const {
        std::vector<boost::posix_time::time_duration> time_durations;
        for (const auto &entry : communication_begin_) {
            time_durations.emplace_back(Normalize(entry.second).time_of_day());
        }

        return *std::min_element(std::cbegin(time_durations), std::cend(time_durations));
    }

    bool TryUpdate(const boost::posix_time::ptime &date_time) {
        if (last_contact_ != boost::posix_time::not_a_date_time) {
            const auto date_duration = last_contact_.date() - date_time.date();
            if (date_duration.days() > 1 || date_duration.days() < -1) {
                return false;
            }

            const auto time_duration = Normalize(last_contact_).time_of_day() - Normalize(date_time).time_of_day();
            if (time_duration >= boost::posix_time::minutes(60) || time_duration <= boost::posix_time::minutes(-60)) {
                return false;
            }
        }

        return TryInsert(date_time);
    }

private:
    friend std::ostream &operator<<(std::ostream &output_stream, const CommunicationBandBuilder &band_builder);

    boost::posix_time::ptime Normalize(const boost::posix_time::ptime &date_time) const {
        return date_time - boost::posix_time::hours(12);
    }

    bool TryInsert(const boost::posix_time::ptime &date_time) {
        const auto current_night_date_time = Normalize(date_time);
        auto find_it = communication_begin_.find(current_night_date_time.date());
        if (find_it != std::end(communication_begin_)) {
            const auto &communication_end_time = communication_end_.at(current_night_date_time.date());
            if (communication_end_time - date_time >= boost::posix_time::seconds(10)) {
                return false;
            }
            last_contact_ = date_time;
            communication_end_.at(current_night_date_time.date()) = date_time;
        } else {
            last_contact_ = date_time;
            communication_begin_.emplace(current_night_date_time.date(), date_time);
            communication_end_.emplace(current_night_date_time.date(), date_time);
        }
        return true;
    }

    boost::posix_time::ptime last_contact_;

    std::unordered_map<boost::gregorian::date, boost::posix_time::ptime> communication_begin_;
    std::unordered_map<boost::gregorian::date, boost::posix_time::ptime> communication_end_;
};

std::ostream &operator<<(std::ostream &output_stream, const CommunicationBandBuilder &band_builder) {
    std::stringstream msg;

    msg << "LastContact: " << band_builder.last_contact_ << std::endl;
    std::vector<boost::gregorian::date> dates;
    for (const auto &entry : band_builder.communication_begin_) {
        dates.emplace_back(entry.first);
    }

    std::sort(std::begin(dates), std::end(dates));
    msg << "CommunicationPeriods:" << std::endl;
    for (const auto &date : dates) {
        msg << " - " << band_builder.communication_begin_.at(date) << " : ";

        const auto find_it = band_builder.communication_end_.find(date);
        if (find_it != std::cend(band_builder.communication_end_)) {
            msg << find_it->second;
        } else {
            msg << "-";
        }
        msg << std::endl;
    }

    output_stream << msg.str();
    return output_stream;
}

std::list<CommunicationBandBuilder> ComputeCommunicationBands(double orbit_inclination,
                                                              double orbit_altitude,
                                                              boost::posix_time::ptime epoch_start,
                                                              boost::posix_time::ptime epoch_end) {
    boost::numeric::ublas::vector<double> orbital_elements(6, 0.0);
    boost::numeric::ublas::vector<double> cartesian_position(3, 0.0);
    boost::numeric::ublas::vector<double> cartesian_velocity(3, 0.0);

    const quake::KeplerElements initial_satellite_position{quake::util::EARTH_EQUATORIAL_RADIUS_KM + orbit_altitude,
                                                           0,
                                                           orbit_inclination * M_PI / 180.0,
                                                           110.5 * M_PI / 180.0,
                                                           0.0,
                                                           0 * M_PI / 180.0};

    Observer ground_station_observer{quake::GroundStation::London.coordinates()};

    std::list<CommunicationBandBuilder> band_builders;
    for (boost::posix_time::ptime current_time = epoch_start; current_time < epoch_end; current_time += boost::posix_time::seconds(1)) {
        const auto elapsed_time = current_time - epoch_start;
        const auto current_date_time = epoch_start + elapsed_time;
        if (current_date_time.time_of_day() <= boost::posix_time::hours(18) && current_date_time.time_of_day() >= boost::posix_time::hours(6)) {
            continue;
        }

        cartesian_position.clear();
        cartesian_velocity.clear();

        PropagateOrbitalElements(initial_satellite_position, elapsed_time, orbital_elements);
        kep_toolbox::par2ic(orbital_elements, kMU, cartesian_position, cartesian_velocity);

        const auto current_astronomic_date_time = CreateDateTime(current_date_time);
        Eci current_satellite_eci = CreateEci(current_astronomic_date_time, cartesian_position, cartesian_velocity);
        const auto elevation = Util::RadiansToDegrees(ground_station_observer.GetLookAngle(current_satellite_eci).elevation);
        if (elevation > 15.0) {
            auto inserted = false;
            for (auto &band_builder : band_builders) {
                if (band_builder.TryUpdate(current_date_time)) {
                    inserted = true;
                    break;
                }
            }

            if (!inserted) {
                CommunicationBandBuilder band_builder;
                inserted = band_builder.TryUpdate(current_date_time);
                band_builders.emplace_front(std::move(band_builder));
            }

            CHECK(inserted);
        }
    }
    return band_builders;
}

boost::posix_time::time_duration GetMaxCommunicationStartDrift(double orbit_inclination,
                                                               double orbit_altitude,
                                                               boost::posix_time::ptime epoch_start,
                                                               boost::posix_time::ptime epoch_end) {

    const auto band_builders = ComputeCommunicationBands(orbit_inclination, orbit_altitude, epoch_start, epoch_end);
    std::vector<boost::posix_time::time_duration> min_begins;
    for (const auto &band_builder : band_builders) {
        if (!band_builder.ConvergedAt(epoch_end)) {
            continue;
        }
        min_begins.emplace_back(band_builder.GetMinNormalizedBegin());
    }
    const auto min_max_begin_element = std::minmax_element(std::cbegin(min_begins), std::cend(min_begins));
    return *min_max_begin_element.second - *min_max_begin_element.first;
}

boost::posix_time::time_duration GetMaxDriftWithinBand(double orbit_inclination,
                                                       double orbit_altitude,
                                                       boost::posix_time::ptime epoch_start,
                                                       boost::posix_time::ptime epoch_end) {

    const auto band_builders = ComputeCommunicationBands(orbit_inclination, orbit_altitude, epoch_start, epoch_end);
    std::vector<boost::posix_time::time_duration> duration_drifts;
    for (const auto &band_builder : band_builders) {
        duration_drifts.emplace_back(band_builder.GetBeginDrift());
    }
    return *std::max_element(std::cbegin(duration_drifts), std::cend(duration_drifts));
}

template<typename ResultType>
class CachedFunction {
public:
    explicit CachedFunction(std::function<ResultType(double)> function)
            : function_{std::move(function)} {}

    ResultType operator()(double input) {
        const auto find_it = cached_values_.find(input);
        if (find_it != std::cend(cached_values_)) {
            return find_it->second;
        }

        const auto output = function_(input);

        cached_values_.emplace(input, output);

        return output;
    }

private:
    std::function<ResultType(double)> function_;
    std::unordered_map<double, ResultType> cached_values_;
};

template<typename ResultType>
void HillClimbing(double initial_point, double initial_step, std::function<ResultType(double)> function) {
    auto current_input = initial_point;
    auto current_step = initial_step;
    auto current_output = function(current_input);

    CachedFunction<ResultType> cached_function(std::move(function));

    while (current_step > 10e-18) {
        const auto candidate_left = current_input - current_step;
        const auto candidate_left_output = cached_function(candidate_left);

        const auto candidate_right = current_input + current_step;
        const auto candidate_right_output = cached_function(candidate_right);

        if (candidate_left_output < current_output && candidate_right_output < current_output) {
            std::stringstream msg;

            msg << "Both directions bring improvement. We go ";
            if (candidate_left_output < candidate_right_output) {
                msg << "left, as it leads to better improvement.";
            } else if (candidate_left_output > candidate_right_output) {
                msg << "right, as it leads to better improvement.";
            } else {
                msg << "right, as tie breaker.";
            }

            LOG(WARNING) << msg.str();
        }

        if (candidate_left_output >= current_output && candidate_right_output >= current_output) {
            current_step /= 2.0;
            LOG(INFO) << "No improvement with current step. Lowering step to: " << current_step;
        } /*else if (candidate_left_output < current_output && candidate_right_output < current_output) {
            LOG(INFO) << "Not sure where to go: "
                      << candidate_left_output << " (left) "
                      << current_output << " (current) "
                      << candidate_right_output << " (right)";
            break;
        } */else if (candidate_left_output < candidate_right_output) {
            current_input = candidate_left;
            current_output = candidate_left_output;

            LOG(INFO) << "Current Best: " << std::fixed << std::setprecision(15) << current_output
                      << " (" << std::fixed << std::setprecision(15) << current_input << ")";
        } else if (candidate_left_output > candidate_right_output) {
            current_input = candidate_right;
            current_output = candidate_right_output;

            LOG(INFO) << "Current Best: " << std::fixed << std::setprecision(15) << current_output
                      << " (" << std::fixed << std::setprecision(15) << current_input << ")";
        } else if (candidate_left_output == candidate_right_output) {
            // we go right as tie breaker
            current_input = candidate_right;
            current_output = candidate_right_output;

            LOG(INFO) << "Current Best: " << std::fixed << std::setprecision(15) << current_output
                      << " (" << std::fixed << std::setprecision(15) << current_input << ")";
        } else {
            CHECK_EQ(candidate_left_output, current_output);
            CHECK_EQ(current_output, candidate_right_output);
            LOG(INFO) << "No further improvement possible";
            break;
        }
    }
}

int main(int argc, char *argv[]) {
    const boost::posix_time::ptime begin_epoch{boost::gregorian::date{2013, 1, 1}};
    const boost::posix_time::ptime end_epoch{boost::gregorian::date{2019, 1, 1}};
    // inclination 97.631739501953120, altitude 566.900146484375000

//    HillClimbing<boost::posix_time::time_duration>(566.9014, 0.00001,
//                                                   [&begin_epoch, &end_epoch](double orbit_altitude) -> boost::posix_time::time_duration {
//                                                       return GetMaxDriftWithinBand(97.631739501953120, orbit_altitude, begin_epoch, end_epoch);
//                                                   });

    HillClimbing<boost::posix_time::time_duration>(97.631746, 0.000008,
                                                   [&begin_epoch, &end_epoch](double orbit_inclination) -> boost::posix_time::time_duration {
                                                       return GetMaxCommunicationStartDrift(orbit_inclination, 560.0, begin_epoch, end_epoch);
                                                   });


    return EXIT_SUCCESS;
}