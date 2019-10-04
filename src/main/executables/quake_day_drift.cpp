#include <cstdlib>

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
//    const auto seconds_remaining = Util::Mod(elapsed_time.total_seconds(), kSECONDS_PER_DAY);
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


int main(int argc, char *argv[]) {
    const boost::numeric::ublas::vector<double> reference_orbital_elements(6, 0.0);
    const boost::numeric::ublas::vector<double> reference_cartesian_position(3, 0.0);
    const boost::numeric::ublas::vector<double> reference_cartesian_velocity(3, 0.0);

    boost::numeric::ublas::vector<double> orbital_elements = reference_orbital_elements;
    boost::numeric::ublas::vector<double> cartesian_position = reference_cartesian_position;
    boost::numeric::ublas::vector<double> cartesian_velocity = reference_cartesian_velocity;

    const quake::KeplerElements initial_satellite_position{
            quake::util::EARTH_EQUATORIAL_RADIUS_KM + 566.85,
            0,
            97.4 * M_PI / 180.0,
            110.5 * M_PI / 180.0,
            0.0,
            0 * M_PI / 180.0};


    boost::posix_time::ptime initial_epoch{boost::gregorian::date{2013, 1, 1}};
    boost::posix_time::ptime end_epoch{boost::gregorian::date{2013, 1, 31}};

    Observer ground_station_observer{quake::GroundStation::London.coordinates()};
    std::map<boost::gregorian::date, std::vector<double>> elevations_by_date;
    for (boost::posix_time::ptime current_time = initial_epoch; current_time < end_epoch; current_time += boost::posix_time::seconds(1)) {
        const auto elapsed_time = current_time - initial_epoch;
        const auto current_date_time = initial_epoch + elapsed_time;
        const auto current_astronomic_date_time = CreateDateTime(current_date_time);

        PropagateOrbitalElements(initial_satellite_position, elapsed_time, orbital_elements);
        cartesian_position.clear();
        cartesian_velocity.clear();
        kep_toolbox::par2ic(orbital_elements, kMU, cartesian_position, cartesian_velocity);

        Eci current_satellite_eci = CreateEci(current_astronomic_date_time, cartesian_position, cartesian_velocity);
        const auto elevation = ground_station_observer.GetLookAngle(current_satellite_eci).elevation;

        auto find_it = elevations_by_date.find(current_time.date());
        if (find_it != std::end(elevations_by_date)) {
            find_it->second.emplace_back(elevation);
        } else {
            elevations_by_date.emplace(current_time.date(), std::vector<double>{elevation});
        }
    }

    std::vector<boost::gregorian::date> dates;
    for (const auto &entry : elevations_by_date) {
        dates.emplace_back(entry.first);
    }
    std::sort(std::begin(dates), std::end(dates));

    for (const auto &date : dates) {
        LOG(INFO) << date << " " << *std::max_element(std::begin(elevations_by_date.at(date)), std::end(elevations_by_date.at(date)));
    }

    return EXIT_SUCCESS;
}