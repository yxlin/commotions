#ifndef SCAGENT_H
#define SCAGENT_H

using std::string;

class SCAgent
{
private:
  std::vector<double> findRoots(double a, double b, double c); 
  arma::vec softmax(arma::vec x); 

public:
  double m_st, m_et, m_dt, m_pt, m_time;   // m_pt = prediction_time; DEFAULT_PARAMS.T_P
  unsigned int m_nt, m_i_time_step;
  unsigned int m_npt, m_nat, m_naction;  //  n_prediction_time_step, n_action_time (ACTION_VECTOR_LENGTH)
  arma::vec m_times, m_goal, m_speed_actions, m_heading_actions, m_tmp_acceleration;
  arma::mat m_actions;

  double m_collision_distance, m_free_speed; 
  std::string m_name;
  
  Parameters * m_p;
  KinematicState* m_ks;
  Trajectory* m_trajectory;

  bool m_can_reverse;
  arma::uvec m_optional_assumptions;

 
  // - states regarding my own actions; # V_a(t),  Vhat_a(t), DeltaVhat_a(t), V_a|b(t), P_a(t)
  arma::mat mom_action_vals, est_action_vals, est_action_surplus_vals, action_probs;
  arma::cube action_vals_given_behs;

  // - states regarding the behavior of the other agent;  A_b(t), ^V A_b(t), ^O A_b(t), P_b(t), V_b|a(t), P_{x_o|b}(t)
  arma::mat beh_activations, beh_activ_V, beh_activ_O, beh_probs, sensory_probs_given_behs;
  arma::mat beh_acceleration; // the acceleration that the other agent should be applying right now if doing behaviour b
  arma::cube beh_vals_given_actions;
  arma::vec m_time_left_to_CA_entry, m_time_left_to_CA_exit;   // other states 

  SCAgent(){}                   // do-nothing constructors
  SCAgent(const  SCAgent& a);   // copy constructor
  void operator=(const  SCAgent& a); 
  SCAgent(arma::vec time, arma::vec goal, arma::vec initial_position, double initial_speed, double initial_yawangle, 
      arma::vec action_option0, arma::vec action_option1, double collision_distance, Parameters* p, arma::uvec optional_assumptions);
  ~SCAgent() { }
  
  void set_actions(arma::vec action_option, bool angle); 
  void store_options();
  void set_timestamps(double start_time, double end_time, double diff_time, double pred_time);
  
  void update_kinematics(unsigned int i_time_step, unsigned int debug);

  void update_action(unsigned int i_time_step, arma::vec conflict_point, SCAgent* other_agent, arma::uvec optional_assumptions, unsigned int debug);

  double get_distance(unsigned int i_time_step, arma::vec conflict_point, SCAgent* a);

  arma::vec add_action(unsigned int i_time_step, unsigned int i_action);
  
  KinematicState get_future_kinematic_state(arma::vec local_acceleration, arma::vec local_yawrate, 
    unsigned int i_start_time_step, unsigned int n_time_steps_to_advance, SCAgent* a);

  KinematicState get_future_kinematic_state(double local_acceleration, double local_yawrate, 
    unsigned int i_start_time_step, unsigned int n_time_steps_to_advance, SCAgent* a);

  KinematicState get_predicted_own_state(unsigned int i_time_step, unsigned int i_action);
  KinematicState get_predicted_other_state(unsigned int i_time_step, unsigned int i_beh, SCAgent* other_agent);
  double get_time_to_agent_collision(KinematicState state1, KinematicState state2, double collision_distance);
  double get_value_of_state_for_agent(KinematicState own_state, KinematicState oth_state, SCAgent* a);

  double get_value_for_me(KinematicState my_state, KinematicState oth_state, double action);
  double get_value_for_other(KinematicState my_state, KinematicState oth_state, SCAgent * oth_a);
  double get_prob_of_current_state_given_beh(unsigned int i_time_step, unsigned int i_beh, SCAgent * oth_a);

  void clear_negative_acceleration(unsigned int i_time_step);
};


#endif // SCAGENT_H