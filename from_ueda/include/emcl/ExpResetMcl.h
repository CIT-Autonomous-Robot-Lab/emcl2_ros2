//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: BSD-3-Clause

#ifndef EXP_PF_H__
#define EXP_PF_H__

#include "emcl/Mcl.h"

namespace emcl2
{

class ExpResetMcl : public Mcl
{
public:
  ExpResetMcl(
    const Pose & p, int num, const Scan & scan, const std::shared_ptr<OdomModel> & odom_model,
    const std::shared_ptr<LikelihoodFieldMap> & map, double alpha_th, double open_space_th,
    double expansion_radius_position, double expansion_radius_orientation);
  ~ExpResetMcl();

  void sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv);

private:
  double alpha_threshold_;
  double open_space_threshold_;
  double expansion_radius_position_;
  double expansion_radius_orientation_;

  void expansionReset(void);
};

}  // namespace emcl2

#endif
