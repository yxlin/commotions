#include <commotions.h> 

Trajectory::Trajectory(KinematicState* ks, unsigned int nt, unsigned int nat) : m_ks(ks), m_nt(nt), m_nat(nat)
{
  m_acceleration.set_size(m_nat); // action variables
  m_yawrate.set_size(m_nat);

  m_position.set_size(2, m_nt); // x-y x nt
  m_speed.set_size(m_nt);
  m_yawangle.set_size(m_nt);

  m_acceleration.fill(m_ks->m_acceleration); // initial acceleration = 0; initial yaw rate = 0
  m_yawrate.fill(m_ks->m_yawrate);
  m_position.fill(NA_REAL);
  m_speed.fill(NA_REAL);
  m_yawangle.fill(NA_REAL);

  m_position.col(0) = m_ks->m_position; 
  m_speed[0] = m_ks->m_speed;
  m_yawangle[0] = m_ks->m_yawangle;
}

Trajectory::Trajectory(const Trajectory& traj)
{
  m_ks = traj.m_ks;
  m_nt = traj.m_nt;
  m_acceleration = traj.m_acceleration;
  m_yawrate = traj.m_yawrate; 
  m_position = traj.m_position;                 
  m_speed = traj.m_speed;
  m_yawangle = traj.m_yawangle;
}

void Trajectory::operator=(const Trajectory& traj) 
{
  m_ks = traj.m_ks;
  m_nt = traj.m_nt;
  m_acceleration = traj.m_acceleration;
  m_yawrate = traj.m_yawrate; 
  m_position = traj.m_position;                 
  m_speed = traj.m_speed;
  m_yawangle = traj.m_yawangle;
}
