#ifndef SIMULATION_H
#define SIMULATION_H

class Simulation
{
public:
  double m_st, m_et, m_dt;    // start time, end time, differernce time
  double m_pt;                // prediction_time
  unsigned int nt, npt, nat;  // n_time_steps n_prediction_time_step, n_action_time (ACTION_VECTOR_LENGTH)
  
  std::vector<Kinematics *> m_agents;   // Store agent addresses
  
  arma::vec m_times;
  arma::mat m_accs, m_sw_angles, m_yaw_rates, m_speeds, m_angles, m_xs, m_ys;

  Simulation(arma::vec t);
  ~Simulation() { }
  void Show(std::string str) const;
  
  void initialize(std::vector<Kinematics *> agents);


  void advance_Pedestrian(unsigned int j, unsigned int i, unsigned int nstep);

  void advance_Pedestrian(unsigned int j, unsigned int i,
                arma::vec pred_acc, arma::vec pred_yaw_rate,
                arma::vec & out);

  // Version 0 / old name get_future_state
  // Start from a previous time step (i-1); nstep = 1
  void advance_Vehicle(unsigned int j, unsigned int i, unsigned int nstep);

  // Version 1
  // tentatively prepare to advance from a current time step
  // s->advance1(j, predicted_acc, predicted_sw_angles, i, look_ahead);
  void advance_Vehicle(unsigned int j, 
               arma::vec local_acc,
               arma::vec local_sw_angles,
               unsigned int i,
               arma::vec & out);
  
  void get_pos_predictions(unsigned int j, unsigned int i, arma::mat & pos_preds);

  void add_action_Pedestrian(arma::vec & out, double action_magnitude, 
                             unsigned int i);
  
  void add_action_Vehicle(arma::vec & out, double action_magnitude, 
                          unsigned int i);
  bool is_cross(unsigned int i_time_stamp, std::vector<double> wall);

private:
  template <class T> inline int sgn(T v) { return (v > T(0)) - (v < T(0)); }
  // int f(double x, double y, std::vector<double> s) {
  //   double v = (x - s[0]) * (s[3] - s[1]) - (y - s[1]) * (s[2] - s[0]);
  //   return sgn(v);
  // }
  int f(double x, double y, arma::vec s) {
    double v = (x - s[0]) * (s[3] - s[1]) - (y - s[1]) * (s[2] - s[0]);
    return sgn(v);
  }
  
  void assertEquals(int a, int b) {
    if (a == b) {
    } else {
      Rcpp::stop("sgn shows unexpected behaviour");
    }
  }

};

#endif // SIMULATION_H
