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
            for (const auto &slot : cloud_cover_periods_) {
                LOG(INFO) << "Slot: " << slot;

                auto is_contained_in_transfer_window = false;
                for (const auto &transfer_window : problem_->TransferWindows(station)) {
                    LOG(INFO) << "Transfer window: " << transfer_window;

                    if (transfer_window.is_after(slot.end())) {
                        break;
                    }

                    if (transfer_window.is_before(slot.begin())) {
                        continue;
                    }

                    if (transfer_window.intersects(slot)) {
                        LOG_IF(WARNING, !transfer_window.contains(slot))
                                        << "Cloud cover measurement observation " << slot
                                        << " is not fully contained in " << transfer_window;

                        is_contained_in_transfer_window = true;
                        break;
                    }

                    LOG(FATAL) << "Should not reach this line";
                }

                if (is_contained_in_transfer_window) {
                    observable_cloud_cover_periods_row.emplace_back(slot);
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

void quake::BaseRobustMipModel::CreateCloudCoverDuals(std::vector<std::vector<GRBVar>> &container) {
    CHECK(container.empty());

    container.resize(Stations().size());
    for (const auto &station : ObservableStations()) {
        const auto station_index = Index(station);

        auto &container_row = container.at(station_index);
        container_row.reserve(cloud_cover_periods_.size());
        for (const auto &period : cloud_cover_periods_) {
            container_row.emplace_back(mip_model_.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS));
        }
    }
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

double quake::BaseRobustMipModel::GetCloudCoverValue(const std::vector<double> &container, const boost::posix_time::time_period &period) const {
    const auto dual_index = CloudCoverIndex(period);
    const auto value = container.at(dual_index) / 100.0;
    CHECK_GE(value, 0.0);
    CHECK_LE(value, 1.0);
    return value;
}