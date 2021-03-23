#ifndef AGENT_H
#define AGENT_H

using std::string;
using arma::vec;

class Agent
{
private:
  // cd = collision distance; 
  // start time, end time, difference time &  prediction_time
  double m_st, m_et, m_dt;   
  vec m_speed_options, m_angle_options;

  void set_speeds(double start, double delta, double end) 
  {
    using namespace arma;
    if (delta) {
      m_speed_options = regspace(start, delta, end);
    }  else {
      m_speed_options = zeros<vec>(1);
    }
  }
  
  void set_angles(double start, double delta, double end)
  {
    using namespace arma;  // Assume inputs are in degree
    if (delta) {           // convert to radian 
      m_angle_options = regspace(start, delta, end) * M_PI / 180;
    } else {
      m_angle_options = zeros<vec>(1);    
    } 
  }
  
  void store_options()
  {
    using namespace arma;
    unsigned int ns = m_speed_options.n_elem;
    unsigned int na = m_angle_options.n_elem;
    unsigned int k;
    m_options = zeros<mat>(ns*na, 2);
    
    for (size_t i=0; i<ns; i++)
    {
      for (size_t j=0; j<na; j++)
      {
        k = (na == 1) ? j*ns + i : j + i*ns;
        m_options(k, 0) = m_speed_options[i];
        m_options(k, 1) = m_angle_options[j];
      }
    }
  }
  
  void set_timestamps(double start_time, double end_time, double diff_time, 
                      double pred_time) 
  {
    using namespace arma;
    double re = remainder(end_time, diff_time);  // to match numpy arange
    if (re < 1e-10) { 
      vec tmp = regspace(start_time, diff_time, end_time);
      m_times = tmp.subvec(0, tmp.n_elem - 2); 
    } else {
      m_times = regspace(start_time, diff_time, end_time);
    }
    
    m_nt  = m_times.n_elem;
    m_npt = std::ceil(pred_time / diff_time); // PREDICTION_TIME_LENGTH == ACTION_TIME_STEPS
    m_nat = m_npt + m_nt;      // ACTIONS_VECTOR_LENGTH
  }

public:
  // n_time_steps n_prediction_time_step, n_action_time (ACTION_VECTOR_LENGTH)
  double m_pt, m_cd;
  unsigned int m_nt, m_npt, m_nat;  
  arma::vec m_goal;
  arma::vec m_accs, m_sw_angles, m_speed_history, m_angle_history;
  arma::vec m_times, m_yr_history;
  arma::mat m_pos_history, m_options;
  Parameters * m_p;
 
  virtual void initialize(arma::vec pos, arma::vec goal, double ispeed, 
                          double iangle);
  Agent(arma::vec time, arma::vec pos, arma::vec goal, arma::vec so, 
        arma::vec ao, double ispeed, double iangle, double cd, Parameters * p);
  virtual ~Agent() {}

  virtual void advance(unsigned int i, unsigned int nstep);
  virtual void advance(unsigned int i, arma::vec local_acc, 
                       arma::vec local_yaw_rate, arma::vec & out);
  
  virtual void add_action(arma::vec & out,double magnitude, unsigned int i);

  double collide_1Agent(arma::vec pos, double speed, double angle, 
                     arma::mat obstacles);

  double collide_2Agent(arma::vec look_ahead, arma::vec obstacle);

  // One agent collide
  virtual double value_1Agent(arma::vec look_ahead, arma::mat obstacles);
  
  virtual double value_2Agent(arma::vec look_ahead);
  
};

// Derive a subclass
class Pedestrian : public Agent
{
private:

public:
};


#endif // AGENT_H
