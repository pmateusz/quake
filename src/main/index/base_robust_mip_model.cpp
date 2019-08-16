#include "base_robust_mip_model.h"

quake::BaseRobustMipModel::BaseRobustMipModel(const quake::ExtendedProblem *problem,
                                              boost::posix_time::time_duration interval_step,
                                              std::vector<Forecast> forecasts)
        : BaseIntervalMipModel(problem, std::move(interval_step), std::move(forecasts)) {
    CHECK(!Forecasts().empty());

    // create time periods for cloud cover measurements
    {
        std::vector<boost::posix_time::time_period> cloud_cover_periods;
        const auto &forecast = Forecasts().front();
        const auto &forecast_index = forecast.Index();
        if (!forecast_index.empty()) {
            auto index_it = forecast_index.cbegin();
            const auto index_it_end = forecast_index.cend();
            const auto &update_frequency = index_it->second.UpdateFrequency();
            const auto &period = index_it->second.Period();

            ++index_it;
            for (; index_it != index_it_end; ++index_it) {
                CHECK_EQ(period, index_it->second.Period());
                CHECK_EQ(update_frequency, index_it->second.UpdateFrequency());
            }

            for (auto slot_start = period.begin(); slot_start < period.end(); slot_start += update_frequency) {
                cloud_cover_periods.emplace_back(slot_start, update_frequency);
            }
        }
        cloud_cover_periods_ = std::move(cloud_cover_periods);
    }

    // check var model
    {
        for (const auto &station : ObservableStations()) {
            const auto &var_model = problem_->VarModel(station);
            CHECK_EQ(var_model.LowerBound.size(), cloud_cover_periods_.size());
            CHECK_EQ(var_model.UpperBound.size(), cloud_cover_periods_.size());
        }
    }

    // create time periods for observable cloud cover measurements
    {
        std::vector<std::vector<boost::posix_time::time_period> > observable_cloud_cover_periods;
        observable_cloud_cover_periods.resize(Stations().size());
        for (const auto &station : ObservableStations()) {
            std::vector<boost::posix_time::time_period> observable_cloud_cover_periods_row;
            for (const auto &cloud_cover_slot : cloud_cover_periods_) {
                auto is_contained_in_transfer_window = false;
                for (const auto &transfer_window : problem_->TransferWindows(station)) {
                    if (transfer_window.is_after(cloud_cover_slot.end())) {
                        break;
                    }

                    if (transfer_window.is_before(cloud_cover_slot.begin())) {
                        continue;
                    }

                    if (transfer_window.intersects(cloud_cover_slot)) {
                        LOG_IF(WARNING, !cloud_cover_slot.contains(transfer_window))
                                        << "Cloud cover measurement observation " << cloud_cover_slot
                                        << " does not fully cover " << transfer_window;

                        is_contained_in_transfer_window = true;
                        break;
                    }

                    LOG(FATAL) << "Should not reach this line";
                }

                if (is_contained_in_transfer_window) {
                    observable_cloud_cover_periods_row.emplace_back(cloud_cover_slot);
                }
            }

            observable_cloud_cover_periods.at(Index(station)) = std::move(observable_cloud_cover_periods_row);
        }
        observable_cloud_cover_periods_ = std::move(observable_cloud_cover_periods);
    }
}

std::size_t quake::BaseRobustMipModel::CloudCoverIndex(const boost::posix_time::time_period &period) const {
    const auto size = cloud_cover_periods_.size();
    for (std::size_t index = 0; index < size; ++index) {
        const auto &other_period = cloud_cover_periods_[index];
        if (other_period.contains(period) || other_period == period) {
            return index;
        }
    }

    LOG(FATAL) << "Cloud cover period not found for period " << period;
    return 0;
}

const std::vector<boost::posix_time::time_period> &quake::BaseRobustMipModel::CloudCover(const quake::GroundStation &station) const {
    return cloud_cover_periods_;
}


const std::vector<boost::posix_time::time_period> &quake::BaseRobustMipModel::ObservableCloudCover(const quake::GroundStation &station) const {
    return observable_cloud_cover_periods_.at(Index(station));
}

double quake::BaseRobustMipModel::CloudCoverLowerBound(const quake::GroundStation &station, const boost::posix_time::time_period &period) const {
    return GetCloudCoverValue(problem_->VarModel(station).LowerBound, period);
}

double quake::BaseRobustMipModel::CloudCoverUpperBound(const quake::GroundStation &station, const boost::posix_time::time_period &period) const {
    return GetCloudCoverValue(problem_->VarModel(station).UpperBound, period);
}

double normalize_cloud_cover_value(double value) {
    CHECK_GE(value, 0.0);
    CHECK_LE(value, 100.0);
    return value / 100.0;
}

double quake::BaseRobustMipModel::CloudCoverMean(const quake::GroundStation &station, const boost::posix_time::time_period &period) const {
    const auto &forecast = Forecasts().front();
    const auto value = forecast.GetCloudCover(station, period.begin());
    return normalize_cloud_cover_value(value);
}

double cloud_cover_std_to_variance(double std) {
    static const double MIN_VARIANCE_LOWER_BOUND = 0.0001;
    CHECK_GE(std, 0.0);
    return std::max(std::pow(std, 2.0) / 10000.0, MIN_VARIANCE_LOWER_BOUND);
}

double quake::BaseRobustMipModel::CloudCoverVariance(const quake::GroundStation &station, const boost::posix_time::time_period &period) const {
    const auto index = CloudCoverIndex(period);
    const auto standard_deviation = problem_->VarModel(station).StandardDeviation.at(index);

    if (index == 0) {
        CHECK_GE(standard_deviation, 0.0);
    } else {
        CHECK_GT(standard_deviation, 0.0);
    }

    return cloud_cover_std_to_variance(standard_deviation);
}

double quake::BaseRobustMipModel::CloudCoverVarianceLowerBound() const {
    auto variance_lower_bound = std::numeric_limits<double>::max();
    for (const auto &station : Stations()) {
        if (station == GroundStation::None) { continue; }

        for (const auto standard_deviation : problem_->VarModel(station).StandardDeviation) {
            variance_lower_bound = std::min(cloud_cover_std_to_variance(standard_deviation), variance_lower_bound);
        }
    }

    return variance_lower_bound;
}


double quake::BaseRobustMipModel::GetCloudCoverValue(const std::vector<double> &container, const boost::posix_time::time_period &period) const {
    const auto dual_index = CloudCoverIndex(period);
    const auto value = container.at(dual_index);
    return normalize_cloud_cover_value(value);
}

GRBLinExpr quake::BaseRobustMipModel::GetMaxKeysTransferredExpr(const GroundStation &station) {
    GRBLinExpr transferred_keys = 0;
    for (const auto &interval : StationIntervals(station)) {
        transferred_keys += interval.Var() * problem_->KeyRate(station, interval.Period());
    }
    return transferred_keys;
}

GRBLinExpr quake::BaseRobustMipModel::GetMaxKeysTransferredExpr(const GroundStation &station, const boost::posix_time::time_period &period) {
    GRBLinExpr transferred_keys = 0;
    for (const auto &interval : GetIntervals(station, period)) {
        transferred_keys += interval.Var() * problem_->KeyRate(station, interval.Period());
    }
    return transferred_keys;
}