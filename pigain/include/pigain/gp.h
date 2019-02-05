#ifndef PIGAIN_GP_H
#define PIGAIN_GP_H

#include <eigen3/Eigen/Dense>
#include <vector>

namespace pig
{
void sqExpKernel(std::vector<Eigen::Vector3d> x1, std::vector<Eigen::Vector3d> x2,
                 std::tuple<double, double, double> hyper_params);

std::pair<double, double> gp(std::vector<double> y, std::vector<Eigen::Vector3d> x,
                             std::vector<Eigen::Vector3d> x_star,
                             std::tuple<double, double, double> hyper_params);
}  // namespace pig

#endif  // PIGAIN_GP_H