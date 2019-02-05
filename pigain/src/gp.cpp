#include <pigain/gp.h>

namespace pig
{
void sqExpKernel(std::vector<Eigen::Vector3d> x1, std::vector<Eigen::Vector3d> x2,
                 std::tuple<double, double, double> hyper_params)
{
  // TODO: Implement
}

std::pair<double, double> gp(std::vector<double> y, std::vector<Eigen::Vector3d> x,
                             std::vector<Eigen::Vector3d> x_star,
                             std::tuple<double, double, double> hyper_params)
{
  // TODO: Implement
}
}  // namespace pig