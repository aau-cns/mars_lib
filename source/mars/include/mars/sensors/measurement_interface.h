// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef MEASUREMENT_INTERFACE_H
#define MEASUREMENT_INTERFACE_H

#include <Eigen/Dense>
#include <memory>

namespace mars
{
class MeasInterface
{
    public:

    MeasInterface()
    {
    }

    ///
    /// \brief get the measurement noise associated with the current sensor measurement
    /// \param sensor_data contains the current sensor measurement
    /// \return Measurement noise matrix
    ///
    virtual Eigen::MatrixXd get_meas_noise(const std::shared_ptr<void>& sensor_data) = 0;

    protected:

};
}

#endif // MEASUREMENT_INTERFACE_H
