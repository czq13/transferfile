/*
 * tailsitter_plugin.cpp
 *
 *  Created on: Jul 12, 2019
 *      Author: czq
 */


#include <algorithm>
#include <string>

#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "tailsitter_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(TailSitterPlugin)

/////////////////////////////////////////////////
TailSitterPlugin::TailSitterPlugin()
{
  this->cp = ignition::math::Vector3d(0, 0, 0);
  this->forward = ignition::math::Vector3d(1, 0, 0);
  this->upward = ignition::math::Vector3d(0, 0, 1);
  this->alpha0 = 0;

  this->radialSymmetry = false;

  /// how much to change CL per every radian of the control joint value
  this->controlJointRadToCL = 4.0;
  //freopen("/home/czq/try.txt","w",stdout);
  this->file = fopen("/home/czq/try.txt","w");
}

/////////////////////////////////////////////////
TailSitterPlugin::~TailSitterPlugin()
{
}

/////////////////////////////////////////////////
void TailSitterPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "LiftDragPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "LiftDragPlugin world pointer is NULL");

#if GAZEBO_MAJOR_VERSION >= 9
  this->physics = this->world->Physics();
#else
  this->physics = this->world->GetPhysicsEngine();
#endif
  GZ_ASSERT(this->physics, "LiftDragPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("radial_symmetry"))
    this->radialSymmetry = _sdf->Get<bool>("radial_symmetry");

  if (_sdf->HasElement("cp"))
    this->cp = _sdf->Get<ignition::math::Vector3d>("cp");

  // blade forward (-drag) direction in link frame
  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<ignition::math::Vector3d>("forward");
  this->forward.Normalize();

  // blade upward (+lift) direction in link frame
  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<ignition::math::Vector3d>("upward");
  this->upward.Normalize();

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    // GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    // GZ_ASSERT(this->link, "Link was NULL");

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The LiftDragPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&TailSitterPlugin::OnUpdate, this));
    }
  }
  std::string rotor0Name = "rotor_0_joint";
  std::string rotor1Name = "rotor_1_joint";
  std::string rotor2Name = "rotor_2_joint";
  std::string rotor3Name = "rotor_3_joint";
  std::string left_elevon_name = "left_elevon_joint";
  std::string right_elevon_name = "right_elevon_joint";
  std::string elevator_name = "elevator_joint";
  this->rotor0 = this->model->GetJoint(rotor0Name);
  this->rotor1 = this->model->GetJoint(rotor1Name);
  this->rotor2 = this->model->GetJoint(rotor2Name);
  this->rotor3 = this->model->GetJoint(rotor3Name);
  this->left_elevon = this->model->GetJoint(left_elevon_name);
  this->right_elevon = this->model->GetJoint(right_elevon_name);
  this->elevator = this->model->GetJoint(elevator_name);
}

typedef ignition::math::Vector3d vector3d;
double cal_alpha(vector3d vel,vector3d forwardI,vector3d upwardI, vector3d spanwiseI) {
	vector3d velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI;
	vector3d liftI = spanwiseI.Cross(velInLDPlane);liftI.Normalize();
    double cosAlpha = ignition::math::clamp(liftI.Dot(upwardI), -1.0, 1.0);
	return (liftI.Dot(forwardI) >= 0.0) ? acos(cosAlpha) : -acos(cosAlpha);
}
double cal_alpha2(vector3d vel) {
	return atan2(vel[0],vel[2]);
}
/////////////////////////////////////////////////
void TailSitterPlugin::OnUpdate()
{
  GZ_ASSERT(this->link, "Link was NULL");
  // get linear velocity at cp in inertial frame
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d vel = this->link->WorldLinearVel(this->cp);
#else
  ignition::math::Vector3d vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(this->cp));
#endif
  ignition::math::Vector3d velI = vel;
  velI.Normalize();

  // pose of body
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose = this->link->WorldPose();
#else
  ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif

  // rotate forward and upward vectors into inertial frame
  ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);

  ignition::math::Vector3d upwardI;
  if (this->radialSymmetry)
  {
    // use inflow velocity to determine upward direction
    // which is the component of inflow perpendicular to forward direction.
    ignition::math::Vector3d tmp = forwardI.Cross(velI);
    upwardI = forwardI.Cross(tmp).Normalize();
  }
  else
  {
    upwardI = pose.Rot().RotateVector(this->upward);
  }

  // spanwiseI: a vector normal to lift-drag-plane described in inertial frame
  ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalize();

  double tVx = vel.Dot(forwardI),tv;
  tv = 2*this->rotor0->GetVelocity(0)*this->rotor0->GetVelocity(0)*1.875e-5/1.224/(0.203*0.203*3.1415) * 100;
  double vi1 = 0.5 * (-tVx + sqrt(tVx*tVx + tv));

  tv = 2*this->rotor1->GetVelocity(0)*this->rotor1->GetVelocity(0)*8.8889e-6/1.224/(0.127*0.127*3.1415) * 100;
  double vi2 = 0.5 * (-tVx + sqrt(tVx*tVx + tv));

  tv = 2*this->rotor2->GetVelocity(0)*this->rotor2->GetVelocity(0)*1.875e-5/1.224/(0.203*0.203*3.1415) * 100;
  double vi3 = 0.5 * (-tVx + sqrt(tVx*tVx + tv));

  tv = 2*this->rotor3->GetVelocity(0)*this->rotor3->GetVelocity(0)*8.8889e-6/1.224/(0.127*0.127*3.1415) * 100;
  double vi4 = 0.5 * (-tVx + sqrt(tVx*tVx + tv));
  double vi = (vi1+vi2+vi3+vi4) /4;
  vel = vel + forwardI*vi;
  velI = vel;
  velI.Normalize();

  const double minRatio = -1.0;
  const double maxRatio = 1.0;
  // check sweep (angle between velI and lift-drag-plane)
  double sinSweepAngle = ignition::math::clamp(
      spanwiseI.Dot(velI), minRatio, maxRatio);

  // get cos from trig identity
  double cosSweepAngle = 1.0 - sinSweepAngle * sinSweepAngle;
  this->sweep = asin(sinSweepAngle);

  // truncate sweep to within +/-90 deg
  while (fabs(this->sweep) > 0.5 * M_PI)
    this->sweep = this->sweep > 0 ? this->sweep - M_PI
                                  : this->sweep + M_PI;
  double beta = this->sweep;
  // angle of attack is the angle between
  // velI projected into lift-drag plane
  //  and
  // forward vector
  //
  // projected = spanwiseI Xcross ( vector Xcross spanwiseI)
  //
  // so,
  // removing spanwise velocity from vel
  ignition::math::Vector3d velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI;

  // get direction of drag
  ignition::math::Vector3d dragDirection = -velInLDPlane;
  dragDirection.Normalize();

  // get direction of lift
  ignition::math::Vector3d liftI = spanwiseI.Cross(velInLDPlane);
  liftI.Normalize();

  // get direction of moment
  ignition::math::Vector3d momentDirectionx = forwardI;
  ignition::math::Vector3d momentDirectiony = spanwiseI;
  ignition::math::Vector3d momentDirectionz = upwardI;

  // compute angle between upwardI and liftI
  // in general, given vectors a and b:
  //   cos(theta) = a.Dot(b)/(a.Length()*b.Lenghth())
  // given upwardI and liftI are both unit vectors, we can drop the denominator
  //   cos(theta) = a.Dot(b)
  double cosAlpha = ignition::math::clamp(liftI.Dot(upwardI), minRatio, maxRatio);

  // Is alpha positive or negative? Test:
  // forwardI points toward zero alpha
  // if forwardI is in the same direction as lift, alpha is positive.
  // liftI is in the same direction as forwardI?
  if (liftI.Dot(forwardI) >= 0.0)
    this->alpha = this->alpha0 + acos(cosAlpha);
  else
    this->alpha = this->alpha0 - acos(cosAlpha);

  // normalize to within +/-90 deg
  /*while (fabs(this->alpha) > 0.5 * M_PI)
    this->alpha = this->alpha > 0 ? this->alpha - M_PI
                                  : this->alpha + M_PI;*/


  // compute dynamic pressure
  double speedInLDPlane = velInLDPlane.Length();

  /*czq add*/
  double bref = 1.3,cref = 0.36 ,cbar = 0.36;
  double S = bref * cref, Clp = -1.0209, Clr = 0.03066, Clbeta = 0.00005,Clele = -0.006, Cnbeta = -0.0042, Cnp = 0.01297,Cnr = -0.00434;
  double CLq = 2.8932, Cmq = -1.3351, Cybeta = -0.0233;

  double alpha_deg = this->alpha * 57.3;
  double A0, A1, A2, D0, D1, D2, C0, C1, C2;
  if (alpha_deg >= -100 && alpha_deg < -10) A0 = -0.093, A1 = 0.8665, A2 = 0.5596;
  else if (alpha_deg >= -10 && alpha_deg < -6) A0 = -0.3227, A1 = -0.5475, A2 = 0;
  else if (alpha_deg >= -6 && alpha_deg < 10) A0 = 0.0226, A1 = 2.7504, A2 = 0;
  else if (alpha_deg >= 10 && alpha_deg < 15) A0 = -0.3636, A1 = 8.1481, A2 = -18.249;
  else if (alpha_deg >= 15 && alpha_deg < 23) A0 = 1.5246, A1 = -5.9874, A2 = 8.1952;
  else if (alpha_deg >= 23 && alpha_deg <= 100) A0 = 0.1935, A1 = 0.8533, A2 = -0.5856;
  else A0 = A1 = A2 = 0;

  if (alpha_deg >= -100 && alpha_deg < -35) D0 = -1.647, D1 = -4.6054, D2 = -1.4602;
  else if (alpha_deg >= -35 && alpha_deg < 35) D0 = 0.09, D1 = 0.0037, D2 = 1.43;
  else if (alpha_deg >= 35 && alpha_deg <= 100) D0 = -1.6425, D1 = 4.6054, D2 = -1.4602;
  else D0 = D1 = D2 = 0;

  if (alpha_deg >= -100 && alpha_deg < -40) C0 = -0.3319, C1 = -0.9592, C2 = -0.2456;
  else if (alpha_deg >= -40 && alpha_deg < -23) C0 = 0.2722, C1 = 1.0214, C2 = 1.3645;
  else if (alpha_deg >= -23 && alpha_deg < 23) C0 = 0.016, C1 = -0.1662, C2 = 0;
  else if (alpha_deg >= 23 && alpha_deg < 38) C0 = -0.2403, C1 = 1.0217, C2 = -1.3648;
  else if (alpha_deg >= 38 && alpha_deg <= 100) C0 = 0.3639, C1 = -0.9592, C2 = 0.24568;
  else C0 = C1 = C2 = 0;

  // compute cl at cp, check for stall, correct for sweep
  ignition::math::Vector3d pqr_world = this->link->WorldAngularVel();
  ignition::math::Vector3d pqr = pose.Inverse().Rot().RotateVector(pqr_world);

  double pv = pqr[0],qv = pqr[1],rv = pqr[2];
  double cl,cd,cy,Cl,Cm,Cn;
  cl = (A0 + A1 * this->alpha + A2 * this->alpha * this->alpha)*1.5 + cbar/2/vel.Length()*CLq*qv;
  cd = D0 + D1 * this->alpha + D2 * this->alpha * this->alpha;
  cy = Cybeta * beta * 57.3;
  Cl = Clbeta * beta * 57.3 + bref/2/vel.Length()*(Clp * pv + Clr * rv);
  Cm = C0 + C1 * this->alpha + C2 * this->alpha * this->alpha - cbar/2/vel.Length()*Cmq*qv;
  Cn = Cnbeta*beta*57.3 + bref/2/vel.Length()*(Cnp*pv+Cnr*rv);
  double vt = velInLDPlane.Length();
  double elevonl = this->left_elevon->Position(0);
  double elevonr = this->right_elevon->Position(0);
  double ele = this->elevator->Position(0);
  //double q = 0.5 * 1.224 * speedInLDPlane * speedInLDPlane;
  double q = 0.5 * 1.224 * vt * vt;
  // compute lift force at cp
  ignition::math::Vector3d lift = cl * q * S * liftI;
  // drag at cp
  ignition::math::Vector3d drag = cd * q * S * dragDirection;

  ignition::math::Vector3d Fy = cy * q * S * spanwiseI;

  ignition::math::Vector3d air_momentx = Cl * q * S * momentDirectionx * bref;
  ignition::math::Vector3d air_momenty = Cm * q * S * momentDirectiony * cref;
  ignition::math::Vector3d air_momentz = Cn * q * S * momentDirectionz * bref;

  ignition::math::Vector3d ele_momentx = (-0.006)*(elevonl-elevonr)*57.3*(0.5*1.224*vt*vt*S) * momentDirectionx * bref;
  //ignition::math::Vector3d ele_momenty = (-0.004)* ele * 57.3 * (0.5*1.224*vt*vt*S) * momentDirectiony * cref;
  ignition::math::Vector3d ele_momenty = (-0.004)* (elevonl+elevonr)/2 * 57.3 * (0.5*1.224*vt*vt*S) * momentDirectiony * cref;

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d cog = this->link->GetInertial()->CoG();
#else
  ignition::math::Vector3d cog = ignitionFromGazeboMath(this->link->GetInertial()->GetCoG());
#endif

  // moment arm from cg to cp in inertial plane
  ignition::math::Vector3d momentArm = pose.Rot().RotateVector(
    this->cp - cog
  );
  // gzerr << this->cp << " : " << this->link->GetInertial()->CoG() << "\n";

  // force and torque about cg in inertial frame
  ignition::math::Vector3d force = lift + drag;// + Fy;
  // + moment.Cross(momentArm);

  //ignition::math::Vector3d torque = air_momentx+air_momenty+ele_momentx+ele_momenty;
  //ignition::math::Vector3d torque = air_momentx+air_momenty+ele_momentx+ele_momenty;
  //ignition::math::Vector3d torque = air_momenty+ele_momenty;
  ignition::math::Vector3d torque = ele_momenty +(1.0)*air_momenty + (1.0)*(air_momentx) + ele_momentx;//* (0.05);//
  ignition::math::Vector3d momy = air_momenty + ele_momenty * 3;
  // - lift.Cross(momentArm) - drag.Cross(momentArm);

  // debug
  //
  // if ((this->link->GetName() == "wing_1" ||
  //      this->link->GetName() == "wing_2") &&
  //     (vel.Length() > 50.0 &&
  //      vel.Length() < 50.0))
  if (1)
  {
	//double tmp = (0.006)* ele * (0.5*1.224*vt*vt*S);
    //fprintf(this->file,"%lf %lf %lf %lf %lf\n",alpha,lift,drag,Cm * q * S,tmp);
    //gzdbg << cl * q * S << " " << vel[0] << " " << vel[1] << " " << vel[2] << Cm * q * S << "\n";
	  /*double tmp_alpha1 = cal_alpha(vel,forwardI,upwardI, spanwiseI);
	  double tmp_alpha2 = cal_alpha(this->link->WorldLinearVel(this->cp),forwardI,upwardI, spanwiseI);
	  double tmp_alpha3 = cal_alpha2(pose.Inverse().Rot().RotateVector(vel));
	  double tmp_alpha4 = cal_alpha2(pose.Inverse().Rot().RotateVector(this->link->WorldLinearVel(this->cp)));
	  fprintf(this->file,"%lf %lf %lf %lf %lf %lf %lf %lf\n",this->world->SimTime().Double(),alpha_deg,tmp_alpha1*57.3,tmp_alpha2*57.3,tmp_alpha3*57.3,tmp_alpha4*57.3,(-0.006)* ele * 57.3 * (0.5*1.224*vt*vt*S),ele);
	  double F1 = this->rotor0->GetVelocity(0)*this->rotor0->GetVelocity(0)*1.875e-5 * 100;
	  double F2 = this->rotor1->GetVelocity(0)*this->rotor1->GetVelocity(0)*8.8889e-6 * 100;
	  double F3 = this->rotor2->GetVelocity(0)*this->rotor2->GetVelocity(0)*1.875e-5 * 100;
	  double F4 = this->rotor3->GetVelocity(0)*this->rotor3->GetVelocity(0)*8.8889e-6 * 100;
	  //fprintf(this->file,"%lf %lf %lf %lf %lf %lf ",this->world->SimTime().Double(),F1,F2,F3,F4,tVx);
	  vector3d vel_b = pose.Inverse().Rot().RotateVector(vel);*/
	  //fprintf(this->file,"%lf %lf %lf %lf\n",vel_b[0],vel_b[1],vel_b[2],vi);
	  //fprintf(this->file,"%lf %lf %lf %lf %lf %lf %lf %lf\n",this->world->SimTime().Double(),pose.Rot().X(),pose.Rot().Y(),pose.Rot().Z(),pose.Rot().W(),pose.Pos()[0],pose.Pos()[1],pose.Pos()[2]);
	  //gzdbg << ele << " " << (elevonl+elevonr)/2 << "\n";
	  fprintf(this->file,"%lf %lf %lf %lf %lf\n",this->world->SimTime().Double(),ele*57.3,elevonl*57.3,elevonr*57.3,(elevonl+elevonr)/2*57.3);
  }

  // Correct for nan or inf
  force.Correct();
  this->cp.Correct();
  torque.Correct();

  // apply forces at cg (with torques for position shift)
  this->link->AddForceAtRelativePosition(force, cog);
  vector3d origin = this->link->WorldTorque();
  this->link->AddTorque(torque);
  vector3d after = this->link->WorldTorque();
  //printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf \n",torque[0],torque[1],torque[2],origin[0],origin[1],origin[2],after[0],after[1],after[2]);
}


