#include <commotions.h> 

KinematicState::KinematicState() 
{
  m_position.zeros();
  m_speed = 0;
  m_yawangle = 0;
  m_acceleration = 0;
  m_yawrate = 0;
}
KinematicState::KinematicState(const KinematicState& state)
{
  m_position = state.m_position;
  m_speed = state.m_speed;
  m_yawangle = state.m_yawangle;
  m_acceleration = state.m_acceleration;
  m_yawrate = state.m_yawrate;
}
KinematicState::KinematicState(double speed, double yawangle, double acceleration, double yawrate, arma::vec position) : 
m_speed(speed), m_yawangle(yawangle), m_acceleration(acceleration), m_yawrate(yawrate), m_position(position)
{
}
KinematicState::~KinematicState() {};
void KinematicState::operator=(const KinematicState& state) 
{
  m_position = state.m_position;
  m_speed = state.m_speed;
  m_yawangle = state.m_yawangle;
  m_acceleration = state.m_acceleration;
  m_yawrate = state.m_yawrate;
}
void KinematicState::print(std::string str)
{
  Rcpp::Rcout << str + " :[speed, angle, acc]:" << "[ "<< m_speed << " " << m_yawangle << 
  " " << m_acceleration << " ]\n";
  // m_position.t().print("Coordinate");
}