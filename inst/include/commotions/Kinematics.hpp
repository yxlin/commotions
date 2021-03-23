#ifndef KINEMATICS_H
#define KINEMATICS_H

using arma::vec;
using arma::mat;
using std::string;

class Kinematics
{
public:
  // m_cd = m_collision_distance, angle internally in radian 
  double m_x, m_y, m_gx, m_gy, m_speed, m_angle, m_cd, m_sr;
  vec m_speed_actions, m_heading_actions;
  mat m_actions;
  Parameters * m_p;

  // constructors
  Kinematics(vec pos, vec goal, double speed, double yaw); 
  Kinematics(vec pos, vec goal, double speed, double yaw, Parameters * p); 
  Kinematics(vec pos, vec goal, vec so, vec ao, double speed, double yaw, 
             double cd, double steering_ratio, Parameters * p); 
  
  ~Kinematics() { }

  void set_speeds(double start, double delta, double end);
  void set_angles(double start, double delta, double end);
  void store_options();
  void Show(string str) const; 
  void Show_options(string str) const; 

  double value_Pedestrian(vec look_ahead, mat obstacles);
  double value(vec look_ahead, mat obstacles); //Vehicle one agent
  double value_Vehicle(vec look_ahead, mat obstacles, unsigned int na);
  double collide_1Agent(vec pos, double speed, double angle, mat obstacles);
  double collide_otherA(vec look_ahead, vec other_pos, double other_speed, 
                        double other_heading);

  double collide_ttc(double x, double y, double angle, double speed, 
                     double obs_x, double obs_y);
    
  std::vector<double> findRoots(double a, double b, double c);
  
};

#endif // KINEMATICS_H
