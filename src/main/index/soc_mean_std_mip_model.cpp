#include "soc_mean_std_mip_model.h"

quake::SocMeanStdMipModel::SocMeanStdMipModel(const quake::ExtendedProblem *problem, boost::posix_time::time_duration interval_step)
        : BaseRobustMipModel(problem,
                             std::move(interval_step),
                             std::vector<Forecast>{problem->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast)}) {

}

void quake::SocMeanStdMipModel::Build(const boost::optional<Solution> &solution) {
    BaseRobustMipModel::Build(solution);
}


