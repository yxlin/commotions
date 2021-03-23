#ifndef KINEMATICSTATE_H
#define KINEMATICSTATE_H

class KinematicState 
{
public:
  double m_speed, m_yawangle;
  double m_acceleration, m_yawrate; // ActionState
  arma::vec m_position;  
 
  KinematicState(); 
  // copy constructor
  KinematicState(const KinematicState& state);

  KinematicState(double speed, double yawangle, double acceleration, double yawrate, arma::vec position); 
  ~KinematicState();
  void operator=(const KinematicState &state); 
  void print(std::string str);
};


    
      
#endif