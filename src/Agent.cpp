#include <commotions.h>

void Agent::initialize(arma::vec pos, arma::vec goal, double ispeed, 
                        double iangle)
{
  using namespace arma;
  m_accs      = zeros<vec>(m_nat); // action_long_accs; not long_accs
  m_sw_angles = zeros<vec>(m_nat);
  m_speed_history = zeros<vec>(m_nt);
  m_angle_history = zeros<vec>(m_nt);
  m_pos_history   = zeros<mat>(2, m_nt);
  m_yr_history    = zeros<vec>(m_nt); // yr = yaw_rate
  
  m_pos_history.col(0) = pos;  // starting position
  
  if (std::isnan(iangle)) { 
    m_angle_history[0] = std::atan2(goal[1]-pos[1], goal[0]-pos[0]); 
  } else {
    m_angle_history[0] = iangle;
  }
  
  if (std::isnan(ispeed)) { 
    m_speed_history[0] = 0; 
  } else {
    m_speed_history[0] = ispeed; 
  }
  
}
Agent::Agent(arma::vec time, arma::vec pos, arma::vec goal, arma::vec so, 
      arma::vec ao, double ispeed, double iangle, double cd, Parameters * p) : 
  m_st(time[0]), m_et(time[1]), m_dt(time[2]), m_pt(time[3]), m_p(p)
{
  if (so.n_elem != 3 || ao.n_elem != 3) 
  {
    Rcpp::stop("option vector must have 3 elements");
  }
  m_goal = goal;
  m_cd   = cd;
  
  set_speeds(so(0), so(1), so(2));
  set_angles(ao(0), ao(1), ao(2));
  store_options();
  
  set_timestamps(m_st, m_et, m_dt, m_pt);
  initialize(pos, goal, ispeed, iangle);
}

void Agent::advance(unsigned int i, unsigned int nstep) 
{
  arma::vec local_pos = m_pos_history.col(i); // i = i_time_stamp
  double local_speed  = m_speed_history(i);
  double local_angle  = m_angle_history(i);
  
  for (size_t j=(i+1); j < (i+nstep+1); j++)
  {
    local_speed += m_accs(j-1) * m_dt;
    if (local_speed < 0) local_speed = 0;
    
    local_angle += m_sw_angles(j-1) * m_dt;
    
    local_pos[0] += m_dt * local_speed * std::cos(local_angle);
    local_pos[1] += m_dt * local_speed * std::sin(local_angle);
  }
  
  m_pos_history(0, i+1) = local_pos[0];
  m_pos_history(1, i+1) = local_pos[1];
  m_speed_history(i+1)  = local_speed;
  m_angle_history(i+1)  = local_angle;
}

void Agent::advance(unsigned int i, arma::vec local_acc, 
                     arma::vec local_yaw_rate, arma::vec & out) 
{
  // out[0] = x; out[1] = y; out[2] = speed; out[3] = angle
  out.subvec(0, 1) = m_pos_history.col(i);
  out[2] = m_speed_history(i);
  out[3] = m_angle_history(i);
  
  for (size_t j=(i+1); j < (i+m_npt+1); j++)
  {
    out[2] += local_acc(j-1) * m_dt;
    out[3] += local_yaw_rate(j-1) * m_dt;
    out[0] += m_dt * out[2] * std::cos(out[3]);
    out[1] += m_dt * out[2] * std::sin(out[3]);
  }
}

void Agent::add_action(arma::vec & out,double magnitude, unsigned int i)
{ 
  using namespace arma; // i = i_time_stamp
  uvec rng = regspace<uvec>(i, 1, i + m_npt - 1);
  double increment = magnitude / m_pt;
  for (size_t j = 0; j < rng.size(); j++) out[rng[j]] += increment;
}


double Agent::collide_1Agent(arma::vec pos, double speed, double angle, 
                      arma::mat obstacles)
{
  double out = 0;
  double time2collision = NA_REAL;
  arma::vec vector2obstacle, heading2obstacle;
  double heading_toward_obstacle_component, angle2obstacle_rel_heading;
  double lateralD2obstacle, longD2obstacle, D2O;
  
  for(size_t i = 0; i < obstacles.n_cols; i++)
  {
    vector2obstacle  = obstacles.col(i) - pos; 
    D2O = arma::norm(vector2obstacle);
    heading2obstacle = vector2obstacle / D2O;
    
    heading_toward_obstacle_component = 
      std::cos(angle) * heading2obstacle[0] + 
      std::sin(angle) * heading2obstacle[1];
    
    angle2obstacle_rel_heading = std::acos(heading_toward_obstacle_component);
    
    lateralD2obstacle = D2O * std::sin(angle2obstacle_rel_heading);
    
    // The only difference       
    if (lateralD2obstacle < m_cd && heading_toward_obstacle_component > 0)
    {
      longD2obstacle = D2O * std::cos(angle2obstacle_rel_heading);
      time2collision = longD2obstacle / speed;
      out += -m_p->kc / time2collision; // assume ko = kc
    }
  }
  
  return out;
}

double Agent::collide_2Agent(arma::vec look_ahead, arma::vec obstacle)
{
  arma::vec pos = look_ahead.subvec(0, 1);
  double speed = look_ahead[2];
  double angle = look_ahead[3];
  
  double time2collision, longD2O, out = 0;
  
  arma::vec heading_vector(2);
  heading_vector[0] = std::cos(angle);
  heading_vector[1] = std::sin(angle);
  
  
  arma::vec V2O = obstacle - pos;
  double D2O = arma::norm(V2O);
  arma::vec heading2O = V2O / D2O;
  double heading_toward_obstacle_component = arma::accu(heading_vector % heading2O);
  
  double angle2O_rel_heading = std::acos(heading_toward_obstacle_component);
  double lateralD2O = D2O * std::sin(angle2O_rel_heading);
  
  if (lateralD2O < m_cd && heading_toward_obstacle_component > 0) 
  {
    longD2O = D2O * std::cos(angle2O_rel_heading);
    time2collision = longD2O / speed;
    out += -m_p->kc / time2collision;
  }
  return out;
}

// One agent collide
double Agent::value_1Agent(arma::vec look_ahead, arma::mat obstacles)
{
  // look_ahead[0] = x; look_ahead[1] = y;
  // look_ahead[2] = speed; look_ahead[3] = angle
  arma::vec vector2goal = m_goal - look_ahead.subvec(0,1);
  double D2G = arma::norm(vector2goal);
  // double D2G = std::sqrt(vector2goal[0]*vector2goal[0]+ 
  //                        vector2goal[1]*vector2goal[1]);
  arma::vec heading2goal_vector = vector2goal / D2G;
  
  double heading_toward_goal_component = 
    std::cos(look_ahead[3]) * heading2goal_vector[0] + 
    std::sin(look_ahead[3]) * heading2goal_vector[1];
  
  double goal_distance_change_rate = -heading_toward_goal_component * 
    look_ahead[2];
  
  double out = -m_p->kg * goal_distance_change_rate - m_p->kdv * 
    look_ahead[2]*look_ahead[2];
  
  // Collision cost
  out += collide_1Agent(look_ahead.subvec(0,1), look_ahead[2], 
                        look_ahead[3], obstacles);
  return out;
}

double Agent::value_2Agent(arma::vec look_ahead)
{
  // look_ahead[0] = x; look_ahead[1] = y;
  // look_ahead[2] = speed; look_ahead[3] = angle
  arma::vec heading_vector(2);
  heading_vector[0] = std::cos(look_ahead[3]);
  heading_vector[1] = std::sin(look_ahead[3]);
  
  arma::vec V2G = m_goal - look_ahead.subvec(0,1); // vector 2 goal
  double D2G = arma::norm(V2G);
  arma::vec heading2goal_vector = V2G / D2G;
  // accu(A % B) is a "multiply-and-accumulate" operation
  // as operator % performs element-wise multiplication
  double heading_toward_goal_component = arma::accu(heading_vector % heading2goal_vector);
  
  double out = -m_p->kg * (-heading_toward_goal_component * look_ahead[2]) 
    -m_p->kdv * look_ahead[2]*look_ahead[2];
  // Rcpp::Rcout << "In value fun " << out << "\n";
  
  return out;
}
