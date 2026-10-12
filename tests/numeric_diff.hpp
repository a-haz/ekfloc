#pragma once

#include <Eigen/Core>

#include <cstddef>

namespace test_util {

/// Central difference of eval. diff(a, b) must equal a - b in the output tangent space.
template <typename Eval, typename Difference>
Eigen::MatrixXd central_jacobian(Eval eval, Difference diff, const Eigen::VectorXd& x, double eps = 1e-7) {
    const Eigen::VectorXd reference = eval(x);
    const Eigen::Index rows = diff(reference, reference).size();
    Eigen::MatrixXd jacobian(rows, x.size());
    for (Eigen::Index column = 0; column < x.size(); ++column) {
        Eigen::VectorXd upper = x;
        Eigen::VectorXd lower = x;
        upper(column) += eps;
        lower(column) -= eps;
        jacobian.col(column) = diff(eval(upper), eval(lower)) / (2.0 * eps);
    }
    return jacobian;
}

}  // namespace test_util
