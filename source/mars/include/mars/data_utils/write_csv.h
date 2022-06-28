// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef WRITE_CSV_H
#define WRITE_CSV_H

#include <Eigen/Dense>
#include <iostream>
#include <sstream>

namespace mars
{
class WriteCsv
{
public:
  inline static std::string vec_to_csv(const Eigen::VectorXd& a)
  {
    std::stringstream os;
    // TODO(chb) option to pre or post add comma
    for (int k = 0; k < a.size(); k++)
    {
      os << ", " << a(k);
    }
    return os.str();
  }

  ///
  /// \brief get_csv_state_with_cov_header_string generate string with state and upper triangular covariance
  /// \return
  ///
  inline static std::string cov_mat_to_csv(const Eigen::MatrixXd& cov)
  {
    if (cov.rows() != cov.cols())
    {
      std::cout << "Cov Mat to CSV: Cov matrix non-squared. No line written." << std::endl;
      return std::string("");
    }

    std::stringstream os;
    const int num_cov_state = static_cast<int>(cov.rows());

    for (int k = 0; k < num_cov_state; ++k)
    {
      int row_count = k * num_cov_state + k;
      int col_count = num_cov_state - k;  // Size relative to row_count
      os << vec_to_csv(Eigen::Map<const Eigen::VectorXd>(cov.data() + row_count, col_count));
    }

    return os.str();
  }

  static std::string get_cov_header_string(const int& num_states)
  {
    std::stringstream os;

    const std::string var("p_");
    for (int row = 1; row <= num_states; row++)
    {
      for (int col = row; col <= num_states; col++)
      {
        os << ", " << var << row << "_" << col;
      }
    }

    return os.str();
  }
};
}  // namespace mars

#endif  // WRITE_CSV_H
