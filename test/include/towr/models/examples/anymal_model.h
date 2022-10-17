/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_ANYMAL_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_ANYMAL_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of the quadruped robot ANYmal.
 */
class AnymalKinematicModel : public KinematicModel {
public:
  AnymalKinematicModel () : KinematicModel(4)
  {
    //const double x_nominal_b = 0.34;
    //const double y_nominal_b = 0.19;
    //const double z_nominal_b = -0.42;

    //nominal_stance_.at(LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
    //nominal_stance_.at(RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
    //nominal_stance_.at(LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
    //nominal_stance_.at(RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;
    
    //ho modificato le posizioni nominali dei piedi e messo i segni corretti 
    
    const double x_nominal_b = 0.183;
    const double y_nominal_b = 0.12;
    const double z_nominal_b = -0.347; //0.347 (1,2,3,6,7) //0.297 (4) //0.277 (5)

    nominal_stance_.at(LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b; 
    nominal_stance_.at(RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b; 
    nominal_stance_.at(LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

    max_dev_from_nominal_ << 0.45, 0.1, 0.05; //0.45, 0.1, 0.05 (pitch pari a 0.08726)
  }
};

/**
 * @brief The Dynamics of the quadruped robot ANYmal.
 */
/*class AnymalDynamicModel : public SingleRigidBodyDynamics {
public:
  AnymalDynamicModel()
  : SingleRigidBodyDynamics(29.5,
                    0.946438, 1.94478, 2.01835, 0.000938112, -0.00595386, -0.00146328,
                    4) {}
};*/

//ho modificato la massa e l'inerzia (l'inerzia é stata presa quella del link base)

/*class AnymalDynamicModel : public SingleRigidBodyDynamics {
public:
  AnymalDynamicModel()
  : SingleRigidBodyDynamics(52.1348,
                    0.216712, 1.78516, 1.8337, 0.000922381, 0.0589332, -0.000163778,
                    4) {}
};*/

class AnymalDynamicModel : public SingleRigidBodyDynamics {
public:
  AnymalDynamicModel()
  : SingleRigidBodyDynamics(16.004,
                    0.22664, 0.475023, 0.452953, -0.000183395, -0.00145023, -0.000443734,
                    4) {}
};

} // namespace towr

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_ANYMAL_MODEL_H_ */
