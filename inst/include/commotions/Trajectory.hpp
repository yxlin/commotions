#ifndef TRAJECTORY_H
#define TRAJECTORY_H

class Trajectory
{
public:
  KinematicState* m_ks;
  unsigned int m_nt, m_nat;
  arma::vec m_acceleration, m_yawrate;  // action variable???
  arma::mat m_position;                 //
  arma::vec m_speed, m_yawangle;

  Trajectory(KinematicState* ks, unsigned int nt, unsigned int nat);
  // copy constructor
  Trajectory(const Trajectory& traj);
  void operator=(const Trajectory &traj); 

  ~Trajectory(){}
};

#endif // TRAJECTORY_H
