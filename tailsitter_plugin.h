/*
 * tailsitter_plugin.h
 *
 *  Created on: Jul 12, 2019
 *      Author: czq
 */

#ifndef INCLUDE_TAILSITTER_PLUGIN_H_
#define INCLUDE_TAILSITTER_PLUGIN_H_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include <ignition/math.hh>

namespace gazebo
{
  /// \brief A plugin that simulates lift and drag.
  class GAZEBO_VISIBLE TailSitterPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: TailSitterPlugin();

    /// \brief Destructor.
    public: ~TailSitterPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    protected: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    protected: physics::ModelPtr model;

    /// \brief air density
    /// at 25 deg C it's about 1.1839 kg/m^3
    /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
    protected: double rho;

    /// \brief if the shape is aerodynamically radially symmetric about
    /// the forward direction. Defaults to false for wing shapes.
    /// If set to true, the upward direction is determined by the
    /// angle of attack.
    protected: bool radialSymmetry;

    /// \brief effective planeform surface area
    protected: double area;

    /// \brief angle of sweep
    protected: double sweep;

    /// \brief initial angle of attack
    protected: double alpha0;

    /// \brief angle of attack
    protected: double alpha;

    /// \brief center of pressure in link local coordinates
    protected: ignition::math::Vector3d cp;

    /// \brief Normally, this is taken as a direction parallel to the chord
    /// of the airfoil in zero angle of attack forward flight.
    protected: ignition::math::Vector3d forward;

    /// \brief A vector in the lift/drag plane, perpendicular to the forward
    /// vector. Inflow velocity orthogonal to forward and upward vectors
    /// is considered flow in the wing sweep direction.
    protected: ignition::math::Vector3d upward;

    /// \brief Smoothed velocity
    protected: ignition::math::Vector3d velSmooth;

    /// \brief Pointer to link currently targeted by mud joint.
    protected: physics::LinkPtr link;

    /// \brief Pointer to a joint that actuates a control surface for
    /// this lifting body
    protected: physics::JointPtr rotor1,rotor2,rotor3,rotor0,left_elevon,right_elevon,elevator;

    /// \brief how much to change CL per radian of control surface joint
    /// value.
    protected: double controlJointRadToCL;

    /// \brief SDF for this plugin;
    protected: sdf::ElementPtr sdf;
    FILE * file;
  };
}



#endif /* INCLUDE_TAILSITTER_PLUGIN_H_ */
