#include <commotions.h> 

using arma::vec;
using arma::mat;
using arma::regspace;

Kinematics::Kinematics(arma::vec pos, arma::vec goal, double speed, double yaw)
  : m_speed(speed), m_angle(yaw)
{
  unsigned int npos  = pos.size();
  unsigned int ngoal = goal.size();
  if (npos  != 2) Rcpp::stop("position coordinate must be of length 2");
  if (ngoal != 2) Rcpp::stop("goal coordinate must be of length 2");
  m_x  = pos[0]; 
  m_y  = pos[1];
  m_gx = goal[0];
  m_gy = goal[1];
  if (ISNAN(m_angle)) { m_angle  = atan2(m_gy - m_y, m_gx - m_x); }
  if (ISNAN(m_speed)) { m_speed  = 0; }
}


Kinematics::Kinematics(arma::vec pos, arma::vec goal, double speed, double yaw,
           Parameters * p) : m_speed(speed), m_angle(yaw), m_p(p)
{
  unsigned int npos  = pos.size();
  unsigned int ngoal = goal.size();
  if (npos  != 2) Rcpp::stop("position coordinate must be of length 2");
  if (ngoal != 2) Rcpp::stop("goal coordinate must be of length 2");
  m_x  = pos[0]; 
  m_y  = pos[1];
  m_gx = goal[0];
  m_gy = goal[1];
  if (ISNAN(m_angle)) { m_angle  = atan2(m_gy - m_y, m_gx - m_x); }
  if (ISNAN(m_speed)) { m_speed  = 0; }
}

void Kinematics::set_speeds(double start, double delta, double end) 
{
  if (delta) {
    m_speed_actions = regspace(start, delta, end);
  }  else {
    m_speed_actions = arma::zeros<vec>(1);
  }
}

void Kinematics::set_angles(double start, double delta, double end)
{
  if (delta) {       // Assume inputs are in degree
    m_heading_actions = regspace(start, delta, end) * M_PI / 180;
  } else {
    m_heading_actions = arma::zeros<vec>(1);    
  } 
}

void Kinematics::store_options()
{
  unsigned ns = m_speed_actions.n_elem;
  unsigned na = m_heading_actions.n_elem;
  unsigned k;
  m_actions = arma::zeros<mat>(ns*na, 2);
  
  for (size_t i=0; i<ns; i++)
  {
    for (size_t j=0; j<na; j++)
    {
      k = (na == 1) ? j*ns + i : j + i*ns;
      m_actions(k, 0) = m_speed_actions[i];
      m_actions(k, 1) = m_heading_actions[j];
    }
  }
}


Kinematics::Kinematics(arma::vec pos, arma::vec goal, arma::vec so, arma::vec ao,
           double speed, double yaw, double cd, double steering_ratio,
           Parameters * p) : m_speed(speed), m_angle(yaw), m_cd(cd), 
           m_sr(steering_ratio), m_p(p)
{
  unsigned int npos  = pos.size();
  unsigned int ngoal = goal.size();
  if (npos  != 2) Rcpp::stop("position coordinate must be of length 2");
  if (ngoal != 2) Rcpp::stop("goal coordinate must be of length 2");
  m_x  = pos[0];
  m_y  = pos[1];
  m_gx = goal[0];
  m_gy = goal[1];
  if (ISNAN(m_angle)) {m_angle  = atan2(m_gy - m_y, m_gx - m_x);}
  if (ISNAN(m_speed)) {m_speed  = 0;}
  
  // this->
  set_speeds(so(0), so(1), so(2));
  set_angles(ao(0), ao(1), ao(2));
  store_options();
}


void Kinematics::Show(std::string str) const 
{
  using namespace Rcpp;
  Rcout << str << "\n";
  Rcout << "starts from [x = " << m_x << ", y = " << m_y << "] ";
    Rcout << "with an initial speed and heading direction: [" << 
      m_speed << " " << m_angle << " in radian (= " << m_angle * 180 / M_PI << " degree) ]\n\n";
}

void Kinematics::Show_options(std::string str) const 
{
  Rcpp::Rcout << str << "\n";
  m_actions.print("All options [speed (m/s), angle (radian)]: ");
}
double Kinematics::value_Pedestrian(arma::vec look_ahead, arma::mat obstacles)
{
  // reward for progress toward goal and cost for speed
  double x2goal = this->m_gx - look_ahead[0];  // vector2goal
  double y2goal = this->m_gy - look_ahead[1];
  double D2G = std::sqrt(x2goal*x2goal+ y2goal*y2goal);
  double x_norm = x2goal / D2G; // adjacent / hypotenuse = cosA
  double y_norm = y2goal / D2G; // opposite / hypotenuse = sinA
  
  // 3 = yaw angle (rotating along z axis);
  // 2 = speed; cos(look_ahead(3)) = cos(theta)
  double heading_toward_goal_component = 
    std::cos(look_ahead(3)) * x_norm + std::sin(look_ahead(3)) * y_norm ;
  // the rate of change of the distance to the goal
  double vg = -heading_toward_goal_component * look_ahead(2); 
  
  double out = -m_p->kg*vg - m_p->kdv*look_ahead(2)*look_ahead(2);
  // Rcpp::Rcout << out << "\n";
  // Collision cost of obstacles
  if (!obstacles.has_nan()) 
  {
    // Rcpp::Rcout << obstacles.has_nan() << "\n"; 
    out += collide_1Agent(look_ahead.subvec(0,1), look_ahead[2], 
                          look_ahead[3], obstacles);
  }
  // Rcpp::Rcout << out << "\n";
  return out;
}

double Kinematics::value(arma::vec look_ahead, arma::mat obstacles)
{
  // Vehicle one agent
  // reward for progress toward goal and cost for speed
  double x2goal = this->m_gx - look_ahead[0];  // vector2goal
  double y2goal = this->m_gy - look_ahead[1];
  double D2G = std::sqrt(x2goal*x2goal+ y2goal*y2goal);
  double x_norm = x2goal / D2G;  // heading_to_goal_vector
  double y_norm = y2goal / D2G;
  
  double heading_toward_goal_component = 
    std::cos(look_ahead[3]) * x_norm + std::sin(look_ahead[3]) * y_norm;
  double goal_distance_change_rate = -heading_toward_goal_component * look_ahead[2];
  
  // Added part
  double D2PassingGoal = heading_toward_goal_component * D2G;
  double required_acc2goal = -(look_ahead[2]*look_ahead[2] / 
                               (2.0*D2PassingGoal));
  
  double out = -m_p->kg * goal_distance_change_rate -
    m_p->ksg * required_acc2goal * required_acc2goal -
    m_p->kdv * look_ahead[2]*look_ahead[2] -
    m_p->kda * look_ahead[4]*look_ahead[4] -
    m_p->Comega * look_ahead[2]*look_ahead[2]*look_ahead[5]*look_ahead[5];
  
  // Rcpp::Rcout << "yaw rate: " << look_ahead[5] << "\n";
  double time2collision = NA_REAL;
  
  if (!obstacles.has_nan())
  {
    for(size_t i = 0; i < obstacles.n_rows; i++)
    {
      time2collision = collide_ttc(look_ahead[0], look_ahead[1], 
                                   look_ahead[3], look_ahead[2],
                                                            obstacles(i, 0), obstacles(i, 1));
      if (std::isfinite(time2collision)) out += -m_p->kc / time2collision;
    }
  }
  
  return out;
}

double Kinematics::value_Vehicle(arma::vec look_ahead, arma::mat obstacles, unsigned int na)
{
  // reward for progress toward goal and cost for speed
  double x2goal = this->m_gx - look_ahead[0];  // vector2goal
  double y2goal = this->m_gy - look_ahead[1];
  double D2G = std::sqrt(x2goal*x2goal+ y2goal*y2goal);
  double x_norm = x2goal / D2G;  // heading_to_goal_vector
  double y_norm = y2goal / D2G;
  
  double heading_toward_goal_component = 
    std::cos(look_ahead[3]) * x_norm + std::sin(look_ahead[3]) * y_norm;
  double goal_distance_change_rate = -heading_toward_goal_component * look_ahead[2];
  
  
  double ag; // required_acc2goal;
  if (na == 1)
  {
    // value_1Agent differs from value_2Agent (Vehicle)
    ag = -(look_ahead[2]*look_ahead[2] /
      (2.0*D2G * heading_toward_goal_component));
  }
  else
  {
    // value_2Agent (Vehicle)
    ag = -(look_ahead[2]*look_ahead[2] / (2.0*D2G));
  }
  
  // look_ahead[4] is acceleration
  // look_ahead[5] is this->m_agents[j]->m_sr * longitudinal speed * 
  // local_sw_angles(k-1);
  double out = -m_p->kg * goal_distance_change_rate - 
    m_p->ksg * ag*ag -
    m_p->kdv * look_ahead[2]*look_ahead[2] -
    m_p->kda * look_ahead[4]*look_ahead[4] -
    m_p->Comega * look_ahead[2]*look_ahead[2]*look_ahead[5]*look_ahead[5];
  
  double time2collision = NA_REAL;
  
  if (!obstacles.has_nan())
  {
    for(size_t i = 0; i < obstacles.n_cols; i++)
    {
      time2collision = collide_ttc(look_ahead[0], look_ahead[1], 
                                   look_ahead[3], look_ahead[2],
                                                            obstacles(0, i), obstacles(1, i));
      if (std::isfinite(time2collision)) out += -m_p->kc / time2collision;
    }
  }
  
  return out;
}

double Kinematics::collide_1Agent(arma::vec pos, double speed, double angle, 
                      arma::mat obstacles)
{
                        double out = 0;
                        double time2collision = NA_REAL;
                        arma::vec vector2obstacle, heading2obstacle;
                        double heading_toward_obstacle_component, angle2obstacle_rel_heading;
                        double lateralD2obstacle, longD2obstacle, distance;
                        
                        for(size_t i = 0; i < obstacles.n_cols; i++)
                        {
                          // See yaw rotation at https://en.wikipedia.org/wiki/Yaw_(rotation)
                          // obstacles.col(i).t().print();
                          vector2obstacle  = obstacles.col(i) - pos; 
                          distance = arma::norm(vector2obstacle);
                          heading2obstacle = vector2obstacle / distance; // cos(A), sin(A)
                          
                          heading_toward_obstacle_component = 
                            std::cos(angle) * heading2obstacle[0] + 
                            std::sin(angle) * heading2obstacle[1];
                          
                          angle2obstacle_rel_heading = std::acos(heading_toward_obstacle_component);
                          
                          lateralD2obstacle = distance * std::sin(angle2obstacle_rel_heading);
                          // Rcpp::Rcout << "lateralD2obstacle: " << lateralD2obstacle;
                          // Rcpp::Rcout << "m_cd: " << m_cd;
                          // Rcpp::Rcout << "heading_toward_obstacle_component: " << heading_toward_obstacle_component;
                          
                          if (lateralD2obstacle < m_cd && 
                              heading_toward_obstacle_component > 0)
                          {
                            // Rcpp::Rcout << "additional value cal\n";
                            longD2obstacle = distance * std::cos(angle2obstacle_rel_heading);
                            time2collision = longD2obstacle / speed;
                            out += -m_p->kc / time2collision; // ko = kc
                            // out += (1/time2collision)
                          }
                        }
                        // return -m_p->kc*out;
                        
                        return out;
}

double Kinematics::collide_otherA(arma::vec look_ahead, arma::vec other_pos, 
                      double other_speed, double other_heading)
{
                        double out = 0;
                        double delta_x = other_pos[0] - look_ahead[0];  // vector_to_obstacle
                        double delta_y = other_pos[1] - look_ahead[1];  //
                        
                        double delta_v_x = other_speed*std::cos(other_heading) -
                          look_ahead[2]*std::cos(look_ahead[3]);
                        double delta_v_y = other_speed*std::sin(other_heading) -
                          look_ahead[2]*std::sin(look_ahead[3]);
                        
                        double a = delta_v_x*delta_v_x + delta_v_y*delta_v_y;
                        double b = 2.0 * (delta_x * delta_v_x + delta_y * delta_v_y);
                        double c = delta_x*delta_x + delta_y*delta_y;
                        std::vector<double> coll_times = findRoots(a, b, c - m_cd*m_cd);
                        
                        if (coll_times.size() == 1) 
                        {
                          out = (std::isnan(coll_times[0])) ? R_PosInf : coll_times[0]; 
                        }
                        else
                        {
                          // passing collision threshold twice
                          if ( std::copysign(1, coll_times[0]) == std::copysign(1, coll_times[1]) )
                          {
                            out = (coll_times[0] < 0) ? R_PosInf : std::min(coll_times[0], coll_times[1]);
                          }
                        }
                        return out;
}

double Kinematics::collide_ttc(double x, double y, double angle, double speed, 
                   double obs_x, double obs_y)
{
                     double out, long_distance2obstacle;
                     double dx = obs_x - x; // vector_to_obstacle
                     double dy = obs_y - y;
                     double D2Obs = std::sqrt(dx*dx + dy*dy);
                     double heading2obstacle_x = dx / D2Obs;
                     double heading2obstacle_y = dy / D2Obs;
                     double heading2obstacle_component =
                       std::cos(angle)*heading2obstacle_x + std::sin(angle) * heading2obstacle_y;
                     double angle2obstacle_rel_heading = std::acos(heading2obstacle_component);
                     double lateral_distance2obstacle =
                       D2Obs * std::sin(angle2obstacle_rel_heading);
                     
                     if (speed == 0) {
                       out = R_PosInf;
                     } else if (D2Obs < m_cd) {
                       out = 1e-10;
                     } else if (lateral_distance2obstacle < m_cd &&
                       heading2obstacle_component > 0)
                     {
                       long_distance2obstacle = D2Obs * std::cos(angle2obstacle_rel_heading) -
                         m_cd;
                       out = long_distance2obstacle / speed;
                     } else {
                       out = R_PosInf;
                     }
                     return out;
}

std::vector<double> Kinematics::findRoots(double a, double b, double c) 
{
  
  std::vector<double> out(2);
  // When a is 0, the equation becomes linear 
  if (a == 0) {   
    out.resize(1);
    out[0] = NA_REAL;
  }
  
  double d = b*b - 4.*a*c; 
  
  if (d > 0) 
  {
    out[0] = (-b + sqrt(d)) / (2.*a);
    out[1] = (-b - sqrt(d)) / (2.*a);
  } 
  else if (d == 0) 
  { 
    out.resize(1);
    out[0] = -b / (2.*a);
  } 
  else // d < 0 
  {
    // std::complex<double> z1 = -b / (2*a) + 1i * sqrt(abs(d));
    // std::complex<double> z2 = -b / (2*a) - 1i * sqrt(abs(d));
    out.resize(1);
    out[0] = NAN;
  }
  return out;
}
