//
// Copyright 2018 Mateusz Polnik, Marilena Di Carlo
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


#include <glog/logging.h>

#include <utility>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <pykep/core_functions/par2ic.h>
#include <pykep/third_party/libsgp4/CoordTopocentric.h>
#include <pykep/third_party/libsgp4/DateTime.h>
#include <pykep/third_party/libsgp4/Eci.h>
#include <pykep/third_party/libsgp4/Observer.h>
#include <pykep/third_party/libsgp4/SolarPosition.h>
#include <pykep/third_party/libsgp4/Vector.h>
#include <pykep/third_party/libsgp4/Util.h>

#include "ground_station.h"
#include "kepler_elements.h"
#include "elevation.h"

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

std::vector<double> quake::GetElevation(const quake::GroundStation &ground_station,
                                        const quake::KeplerElements &initial_satellite_position,
                                        const boost::posix_time::ptime &initial_epoch,
                                        const boost::posix_time::time_period &observation_period,
                                        const boost::posix_time::time_duration &time_step) {
    Observer ground_station_observer{ground_station.coordinates()};

    const boost::numeric::ublas::vector<double> reference_orbital_elements(6, 0.0);
    const boost::numeric::ublas::vector<double> reference_cartesian_position(3, 0.0);
    const boost::numeric::ublas::vector<double> reference_cartesian_velocity(3, 0.0);

    boost::numeric::ublas::vector<double> orbital_elements = reference_orbital_elements;
    boost::numeric::ublas::vector<double> cartesian_position = reference_cartesian_position;
    boost::numeric::ublas::vector<double> cartesian_velocity = reference_cartesian_velocity;

    CHECK_GE(observation_period.begin(), initial_epoch);
    const auto initial_elapsed_time = observation_period.begin() - initial_epoch;
    const auto observation_time_to_use = observation_period.length() + initial_elapsed_time;

    std::vector<double> elevations;
    for (auto elapsed_time = boost::posix_time::seconds(0);
         elapsed_time < observation_time_to_use;
         elapsed_time += time_step) {
        const auto current_date_time = initial_epoch + elapsed_time;
        const auto current_astronomic_date_time = CreateDateTime(current_date_time);

        PropagateOrbitalElements(initial_satellite_position, elapsed_time, orbital_elements);
        cartesian_position.clear();
        cartesian_velocity.clear();
        kep_toolbox::par2ic(orbital_elements, kMU, cartesian_position, cartesian_velocity);

        if (elapsed_time >= initial_elapsed_time) {
            Eci satellite_eci = CreateEci(current_astronomic_date_time, cartesian_position, cartesian_velocity);
            elevations.push_back(ground_station_observer.GetLookAngle(satellite_eci).elevation);
        }
    }

//    const auto observation_time_to_use = observation_time + initial_elapsed_time;
//    std::vector<double> elevations;
//    for (auto elapsed_time = initial_elapsed_time;
//         elapsed_time < observation_time_to_use;
//         elapsed_time += time_step) {
//        const auto current_date_time = start_time + elapsed_time;
//        const auto current_astronomic_date_time = CreateDateTime(current_date_time);
//
//        PropagateOrbitalElements(initial_satellite_position, elapsed_time, orbital_elements);
//        cartesian_position.clear();
//        cartesian_velocity.clear();
//        kep_toolbox::par2ic(orbital_elements, kMU, cartesian_position, cartesian_velocity);
//
//        Eci satellite_eci = CreateEci(current_astronomic_date_time, cartesian_position, cartesian_velocity);
//        elevations.push_back(ground_station_observer.GetLookAngle(satellite_eci).elevation);
//    }

    return elevations;
}

std::vector<double> quake::GetMatlabElevation(const quake::GroundStation &ground_station,
                                              const quake::KeplerElements &initial_satellite_position,
                                              const boost::posix_time::ptime &start_time,
                                              const boost::posix_time::time_duration &observation_time,
                                              const boost::posix_time::time_duration &time_step) {
    const auto lat_sin = sin(ground_station.coordinates().latitude);
    const auto lat_cos = cos(ground_station.coordinates().latitude);

    const boost::posix_time::ptime reference_time{boost::gregorian::date{start_time.date().year(), 1, 1},
                                                  boost::posix_time::time_duration{}};

    boost::numeric::ublas::vector<double> ground_station_cartesian_position(3);
    boost::numeric::ublas::vector<double> current_orbital_elements(6, 0.0);
    boost::numeric::ublas::vector<double> current_cartesian_position(3, 0.0);
    boost::numeric::ublas::vector<double> current_cartesian_velocity(3, 0.0);
    boost::numeric::ublas::matrix<double> rotation(3, 3);

    CHECK_GE(start_time, reference_time);
    const auto initial_elapsed_time = start_time - reference_time;
    const auto observation_time_to_use = observation_time + initial_elapsed_time;

    std::vector<double> elevations;
    for (auto elapsed_time = boost::posix_time::seconds(0);
         elapsed_time < observation_time_to_use;
         elapsed_time += time_step) {
        const auto current_date_time = reference_time + elapsed_time;
        const auto current_astronomic_date_time = CreateDateTime(current_date_time);

        PropagateOrbitalElements(initial_satellite_position, elapsed_time, current_orbital_elements);

        if (elapsed_time >= initial_elapsed_time) {
            current_cartesian_position.clear();
            current_cartesian_velocity.clear();
            kep_toolbox::par2ic(current_orbital_elements,
                                kMU,
                                current_cartesian_position,
                                current_cartesian_velocity);

            Eci ground_station_eci{current_astronomic_date_time, ground_station.coordinates()};
            Eci satellite_eci = CreateEci(current_astronomic_date_time,
                                          current_cartesian_position,
                                          current_cartesian_velocity);

            ground_station_cartesian_position(0) = ground_station_eci.Position().x;
            ground_station_cartesian_position(1) = ground_station_eci.Position().y;
            ground_station_cartesian_position(2) = ground_station_eci.Position().z;
            const auto cartesian_difference = current_cartesian_position - ground_station_cartesian_position;

            const auto lst = current_astronomic_date_time.ToLocalMeanSiderealTime(
                    ground_station.coordinates().longitude);
            const auto lst_sin = sin(lst);
            const auto lst_cos = cos(lst);
            rotation(0, 0) = lat_sin * lst_cos;
            rotation(0, 1) = lat_sin * lst_sin;
            rotation(0, 2) = -lat_cos;

            rotation(1, 0) = -lst_sin;
            rotation(1, 1) = lst_cos;
            rotation(1, 2) = 0;

            rotation(2, 0) = lat_cos * lst_cos;
            rotation(2, 1) = lat_cos * lst_sin;
            rotation(2, 2) = lat_sin;

            auto rotated_cartesian_difference = prod(rotation, cartesian_difference);
            auto elevation = asin(rotated_cartesian_difference(2) / norm_2(rotated_cartesian_difference));
            elevations.push_back(elevation);
        }
    }

    return elevations;
}

quake::SatelliteTracker::SatelliteTracker(quake::KeplerElements initial_satellite_position,
                                          boost::posix_time::ptime initial_epoch,
                                          boost::posix_time::time_period observation_period,
                                          boost::posix_time::time_duration time_step)
        : initial_satellite_position_{initial_satellite_position},
          initial_epoch_{initial_epoch},
          observation_period_{observation_period},
          time_step_{std::move(time_step)} {}

boost::numeric::ublas::vector<double> ToBoostVector(const Vector &vector) {
    boost::numeric::ublas::vector<double> boost_vector(3, 0.0);
    boost_vector(0) = vector.x;
    boost_vector(1) = vector.y;
    boost_vector(2) = vector.z;
    return boost_vector;
}

class PenumbraTracker {
public:
    PenumbraTracker()
            : current_time_{boost::posix_time::not_a_date_time},
              umbra_started_{boost::posix_time::not_a_date_time},
              penumbra_started_{boost::posix_time::not_a_date_time} {}

    void Sunlit(boost::posix_time::ptime date_time) {
        TryCloseUmbra();
        TryClosePenumbra();

        current_time_ = date_time;
    }

    void Umbra(boost::posix_time::ptime date_time) {
        if (umbra_started_ == boost::posix_time::not_a_date_time) {
            umbra_started_ = date_time;
        }

        if (penumbra_started_ == boost::posix_time::not_a_date_time) {
            penumbra_started_ = date_time;
        }

        current_time_ = date_time;
    }

    void Penumbra(boost::posix_time::ptime date_time) {
        TryCloseUmbra();

        if (penumbra_started_ == boost::posix_time::not_a_date_time) {
            penumbra_started_ = date_time;
        }

        current_time_ = date_time;
    }

    void Close() {
        TryCloseUmbra();
        TryClosePenumbra();
    }

    inline const std::vector<boost::posix_time::time_period> &Umbras() const { return umbras_; }

    inline const std::vector<boost::posix_time::time_period> &Penumbras() const { return penumbras_; }

private:
    void TryCloseUmbra() {
        if (umbra_started_ != boost::posix_time::not_a_date_time) {
            CHECK_GE(current_time_, umbra_started_);

            umbras_.emplace_back(boost::posix_time::time_period{umbra_started_, current_time_});
            umbra_started_ = boost::posix_time::not_a_date_time;
        }
    }

    void TryClosePenumbra() {
        if (penumbra_started_ != boost::posix_time::not_a_date_time) {
            CHECK_GE(current_time_, penumbra_started_);

            penumbras_.emplace_back(boost::posix_time::time_period{penumbra_started_, current_time_});
            penumbra_started_ = boost::posix_time::not_a_date_time;
        }
    }

    boost::posix_time::ptime current_time_;
    boost::posix_time::ptime umbra_started_;
    boost::posix_time::ptime penumbra_started_;

    std::vector<boost::posix_time::time_period> umbras_;
    std::vector<boost::posix_time::time_period> penumbras_;
};

class IndianConicShadowModel {
public:
    void Update(const Eci &sun_position, const Eci &satellite_position, const boost::posix_time::ptime &date_time) {
        using namespace quake;
        using namespace boost::numeric;

        const auto sat_position = ToBoostVector(satellite_position.Position());
        const auto solar_position = ToBoostVector(sun_position.Position());
        const auto solar_position_magnitude = ublas::norm_2(solar_position);
        const auto solar_unit_vector = solar_position / solar_position_magnitude;

        const auto solar_unit_sat_position_dot_product = ublas::inner_prod(solar_unit_vector, sat_position);
        if (solar_unit_sat_position_dot_product <= 0) {
            // the satellite may not be in the sunlit state

            // the projection vector of the satellite position onto solar unit vector
            const auto p = solar_unit_vector * solar_unit_sat_position_dot_product;
            const auto p_magnitude = ublas::norm_2(p);

            // the distance between the umbral cone vertex and the center of the Earth
            const auto X_u = (util::EARTH_EQUATORIAL_RADIUS_KM * solar_position_magnitude)
                             / (util::SOLAR_RADIUS_KM - util::EARTH_EQUATORIAL_RADIUS_KM);

            // the umbral cone angle
            const auto alpha = std::asin((util::SOLAR_RADIUS_KM - util::EARTH_EQUATORIAL_RADIUS_KM) / solar_position_magnitude);

            // the distance between the umbral cone axis and the umbral cone terminator point at the projected satellite position
            const auto d_u = (X_u - p_magnitude) * std::tan(alpha);

            // the distance between the penumbral cone vertex and the center of the Earth
            const auto X_p = (util::EARTH_EQUATORIAL_RADIUS_KM * solar_position_magnitude)
                             / (util::SOLAR_RADIUS_KM + util::EARTH_EQUATORIAL_RADIUS_KM);

            // the penumbral cone angle
            const auto beta = std::asin((util::SOLAR_RADIUS_KM + util::EARTH_EQUATORIAL_RADIUS_KM) / solar_position_magnitude);

            // the distance between the penumbral cone axis and the penumbral cone terminator point at the projected satellite position
            const auto d_p = (X_p + p_magnitude) * std::tan(beta);

            // the distance between the center of umbral or penumbral cone and the satellite at the projected satellite point
            const auto q = sat_position - p;
            const auto q_magnitude = ublas::norm_2(q);
            if (q_magnitude >= d_p) {
                // satellite is in the sunlit
                tracker_.Sunlit(date_time);
            } else if (q_magnitude > d_u && q_magnitude < d_p) {
                // satellite is in the penumbra
                tracker_.Penumbra(date_time);
            } else {
                CHECK_LT(q_magnitude, d_u);
                // satellite is in the umbra
                tracker_.Umbra(date_time);
            }
        } else {
            tracker_.Sunlit(date_time);
        }
    }

    void Close() {
        tracker_.Close();
    }

    std::vector<boost::posix_time::time_period> UmbraWindows() { return tracker_.Umbras(); }

    std::vector<boost::posix_time::time_period> PenumbraWindows() { return tracker_.Penumbras(); }

private:
    PenumbraTracker tracker_;
};

class ChineseConicShadowModel {
public:
    void Update(const Eci &sun_position, const Eci &satellite_position, const boost::posix_time::ptime &date_time) {
        using namespace quake;
        using namespace boost::numeric;

        const auto solar_r = ToBoostVector(sun_position.Position());
        const auto solar_r_magnitude = ublas::norm_2(solar_r);

        const auto satellite_r = ToBoostVector(satellite_position.Position());
        const auto satellite_r_magnitude = ublas::norm_2(satellite_r);

        // penumbral cone geometry
        const auto x_p = (util::EARTH_EQUATORIAL_RADIUS_KM * util::ASTRONOMIC_UNIT_KM) / (util::SOLAR_RADIUS_KM + util::EARTH_EQUATORIAL_RADIUS_KM);
        const auto alpha_p =
                M_PI - std::acos(util::EARTH_EQUATORIAL_RADIUS_KM / x_p) - std::acos(util::EARTH_EQUATORIAL_RADIUS_KM / satellite_r_magnitude);

        // umbral cone geometry
        const auto x_u = (util::EARTH_EQUATORIAL_RADIUS_KM * util::ASTRONOMIC_UNIT_KM) / (util::SOLAR_RADIUS_KM - util::EARTH_EQUATORIAL_RADIUS_KM);
        const auto alpha_u =
                std::acos(util::EARTH_EQUATORIAL_RADIUS_KM / x_u) - std::acos(util::EARTH_EQUATORIAL_RADIUS_KM / satellite_r_magnitude);

        // satellite angle
        const auto dot_product = ublas::inner_prod(satellite_r, solar_r);
        const auto alpha_s =
                M_PI - std::acos(dot_product / (satellite_r_magnitude * solar_r_magnitude));

        if (alpha_s < alpha_u) {
            tracker_.Umbra(date_time);
        } else if (alpha_u <= alpha_s && alpha_s < alpha_p) {
            tracker_.Penumbra(date_time);
        } else if (alpha_s >= alpha_p) {
            tracker_.Sunlit(date_time);
        } else {
            LOG(FATAL) << alpha_u << alpha_p << alpha_s;
        }
    }

    void Close() {
        tracker_.Close();
    }

    std::vector<boost::posix_time::time_period> UmbraWindows() { return tracker_.Umbras(); }

    std::vector<boost::posix_time::time_period> PenumbraWindows() { return tracker_.Penumbras(); }

private:
    PenumbraTracker tracker_;
};

void quake::SatelliteTracker::CalculatePositions(std::vector<Eci> &output_positions,
                                                 std::vector<boost::posix_time::time_period> &output_umbras,
                                                 std::vector<boost::posix_time::time_period> &output_penumbras) {
    using namespace boost::numeric;

    ublas::vector<double> orbital_elements(6, 0.0);
    ublas::vector<double> cartesian_position(3, 0.0);
    ublas::vector<double> cartesian_velocity(3, 0.0);

    CHECK_GE(observation_period_.begin(), initial_epoch_);
    const auto initial_elapsed_time = observation_period_.begin() - initial_epoch_;
    const auto observation_time_to_use = observation_period_.length() + initial_elapsed_time;

    SolarPosition solar_position_finder;
    ChineseConicShadowModel shadow_model;
    std::vector<Eci> positions;
    positions.reserve(observation_period_.length().total_seconds() / time_step_.total_seconds());
    for (auto elapsed_time = boost::posix_time::seconds(0); elapsed_time < observation_time_to_use; elapsed_time += time_step_) {
        const auto current_date_time = initial_epoch_ + elapsed_time;
        const auto current_astronomic_date_time = CreateDateTime(current_date_time);

        PropagateOrbitalElements(initial_satellite_position_, elapsed_time, orbital_elements);
        cartesian_position.clear();
        cartesian_velocity.clear();
        kep_toolbox::par2ic(orbital_elements, kMU, cartesian_position, cartesian_velocity);

        if (elapsed_time < initial_elapsed_time) {
            continue;
        }

        Eci sat_eci = CreateEci(current_astronomic_date_time, cartesian_position, cartesian_velocity);
        positions.emplace_back(sat_eci);

        Eci solar_eci = solar_position_finder.FindPosition(current_astronomic_date_time);
        shadow_model.Update(solar_eci, sat_eci, current_date_time);
    }
    shadow_model.Close();

    output_positions = positions;
    output_umbras = shadow_model.UmbraWindows();
    output_penumbras = shadow_model.PenumbraWindows();

    CHECK_LE(output_umbras.size(), output_penumbras.size());
}
