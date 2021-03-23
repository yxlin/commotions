#include <commotions.h>

// copy constructor

SCAgent::SCAgent(const SCAgent& a)
{
  m_st = a.m_st;   // double type
  m_et = a.m_st;
  m_dt = a.m_dt;
  m_pt = a.m_pt;
  m_time = a.m_time; 
  m_collision_distance = a.m_collision_distance; 
  m_free_speed = a.m_free_speed;

  m_nt = a.m_nt; // unsigned int 
  m_i_time_step = a.m_i_time_step;
  m_npt = a.m_npt;
  m_nat = a.m_nat;
  m_naction = a.m_naction;  
  m_can_reverse = a.m_can_reverse; // bool

  m_times = a.m_times; //  arma::vec
  m_goal = a.m_goal;
  m_speed_actions = a.m_speed_actions;
  m_heading_actions = a.m_heading_actions;
  m_actions = a.m_actions; // arma::mat
  
  m_p = a.m_p; // Parameters pointer
  m_name = a.m_name; // std::string
  m_ks = a.m_ks; // KinematicState pointer

  m_trajectory = a.m_trajectory; // Trajectory pointer
  m_optional_assumptions = a.m_optional_assumptions; // arma::uvec 

  // - states regarding my own actions; # V_a(t),  # Vhat_a(t),  # DeltaVhat_a(t), # V_a|b(t),  # P_a(t)
  mom_action_vals = a.mom_action_vals;
  est_action_vals = a.est_action_vals;
  est_action_surplus_vals = a.est_action_surplus_vals;
  action_probs = a.action_probs;
  action_vals_given_behs = a.action_vals_given_behs;
  // - states regarding the behavior of the other agent; # A_b(t)# ^V A_b(t)# ^O A_b(t)# P_b(t)# V_b|a(t)# P_{x_o|b}(t)
  beh_activations = a.beh_activations;
  beh_activ_V = a.beh_activ_V;
  beh_activ_O = a.beh_activ_O;
  beh_probs = a.beh_probs;
  sensory_probs_given_behs = a.sensory_probs_given_behs;
  beh_acceleration = a.beh_acceleration; // the acceleration that the other agent should be applying right now if doing behaviour b
  beh_vals_given_actions = a.beh_vals_given_actions;
  /* other states */
  m_time_left_to_CA_entry = a.m_time_left_to_CA_entry;
  m_time_left_to_CA_exit = a.m_time_left_to_CA_exit;
}

void SCAgent::operator=(const SCAgent& a) 
{
  m_st = a.m_st;   // double type
  m_et = a.m_st;
  m_dt = a.m_dt;
  m_pt = a.m_pt;
  m_time = a.m_time; 
  m_collision_distance = a.m_collision_distance; 
  m_free_speed = a.m_free_speed;

  m_nt = a.m_nt; // unsigned int 
  m_i_time_step = a.m_i_time_step;
  m_npt = a.m_npt;
  m_nat = a.m_nat;
  m_naction = a.m_naction;  
  m_can_reverse = a.m_can_reverse; // bool

  m_times = a.m_times; //  arma::vec
  m_goal = a.m_goal;
  m_speed_actions = a.m_speed_actions;
  m_heading_actions = a.m_heading_actions;
  m_actions = a.m_actions; // arma::mat
  
  m_p = a.m_p; // Parameters pointer
  m_name = a.m_name; // std::string
  m_ks = a.m_ks; // KinematicState pointer

  m_trajectory = a.m_trajectory; // Trajectory pointer
  m_optional_assumptions = a.m_optional_assumptions; // arma::uvec 

  // - states regarding my own actions; # V_a(t),  # Vhat_a(t),  # DeltaVhat_a(t), # V_a|b(t),  # P_a(t)
  mom_action_vals = a.mom_action_vals;
  est_action_vals = a.est_action_vals;
  est_action_surplus_vals = a.est_action_surplus_vals;
  action_probs = a.action_probs;
  action_vals_given_behs = a.action_vals_given_behs;
  // - states regarding the behavior of the other agent; # A_b(t)# ^V A_b(t)# ^O A_b(t)# P_b(t)# V_b|a(t)# P_{x_o|b}(t)
  beh_activations = a.beh_activations;
  beh_activ_V = a.beh_activ_V;
  beh_activ_O = a.beh_activ_O;
  beh_probs = a.beh_probs;
  sensory_probs_given_behs = a.sensory_probs_given_behs;
  beh_acceleration = a.beh_acceleration; // the acceleration that the other agent should be applying right now if doing behaviour b
  beh_vals_given_actions = a.beh_vals_given_actions;
  /* other states */
  m_time_left_to_CA_entry = a.m_time_left_to_CA_entry;
  m_time_left_to_CA_exit = a.m_time_left_to_CA_exit;
}

std::vector<double> SCAgent::findRoots(double a, double b, double c) 
{
  std::vector<double> out(2);
  double d = b*b - 4.*a*c; 
  if (d < 0) {
    // std::complex<double> z1 = -b / (2*a) + 1i * sqrt(abs(d));
    // std::complex<double> z2 = -b / (2*a) - 1i * sqrt(abs(d));
    out.resize(1);
    out[0] = NA_REAL;
  } else if (d==0) {
    out.resize(1);
    if (a == 0) {
      out[0] = (b == 0) ? NA_REAL : -c/b; 
    } else {
      out[0] = -b / (2.*a); 
    }

  } else {
    out[0] = (-b + sqrt(d)) / (2.*a);
    out[1] = (-b - sqrt(d)) / (2.*a);
  }

  return out;
}
arma::vec SCAgent::softmax(arma::vec x) { return arma::exp(x) / arma::accu(arma::exp(x)); }


void SCAgent::set_timestamps(double start_time, double end_time, double diff_time, double pred_time) 
{
    using namespace arma; // Set up variables: m_times, m_nt, m_npt, m_nat
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

void SCAgent::set_actions(arma::vec action_option, bool angle) 
{
  if (angle) {
    if (action_option[1]) {
      m_heading_actions = arma::regspace(action_option[0], action_option[1], action_option[2]) * M_PI/180.;
    } else {                    
      m_heading_actions = arma::zeros<arma::vec>(1);
    }
  } else {
    if (action_option[1]) {
      m_speed_actions = arma::regspace(action_option[0], action_option[1], action_option[2]);
    } else {
      m_speed_actions = arma::zeros<arma::vec>(1);
    }
  }
}

void SCAgent::store_options()
{
  unsigned int ns = m_speed_actions.n_elem;
  unsigned int na = m_heading_actions.n_elem;
  unsigned int k;
  m_actions = arma::zeros<arma::mat>(ns*na, 2);
  
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

SCAgent::SCAgent(arma::vec time, arma::vec goal, arma::vec initial_position, double initial_speed, double initial_yawangle, 
  arma::vec action_option0, arma::vec action_option1, double collision_distance, Parameters* p, arma::uvec optional_assumptions) : 
  m_st(time[0]), m_et(time[1]), m_dt(time[2]), m_pt(time[3]), m_goal(goal), m_collision_distance(collision_distance), m_p(p)
{
  if (action_option0.n_elem != 3 || action_option1.n_elem != 3) {Rcpp::stop("option vectors must be 3 elements");}
  if (goal.n_elem != 2) Rcpp::stop("goal must store a x-y coordinate.");

  m_i_time_step = 0;

  set_timestamps(m_st, m_et, m_dt, m_pt);
  set_actions(action_option0, false);
  set_actions(action_option1, true);
  store_options();
  m_naction = m_actions.n_rows;
  switch(m_p->type)
    {
      case 0:
        m_name = "P";
        m_can_reverse = true;
        break;
      case 1:
        m_name = "V";
        m_can_reverse = false;
        break;
      default:
        m_name = "A"; // Default to Agent
        m_can_reverse = false;
    }

  if (std::isnan(initial_yawangle)) 
  {
    // goal.t().print("goal");
    // initial_position.t().print("pos");
    initial_yawangle = std::atan2(goal[1]-initial_position[1], goal[0]-initial_position[0]);  
  }

  // acceleration = 0; yawrate = 0;
  m_ks = new KinematicState(initial_speed, initial_yawangle, 0, 0, initial_position);
  m_trajectory = new Trajectory(m_ks, m_nt, m_nat);
  m_free_speed = m_p->kg / (2.*m_p->kdv);

  m_tmp_acceleration.set_size(m_nat);
  m_tmp_acceleration.fill(m_ks->m_acceleration);

    // states regarding my own actions
    mom_action_vals = arma::mat(m_naction, m_nt).fill(NA_REAL); // V_a(t)
    est_action_vals = arma::mat(m_naction, m_nt).fill(NA_REAL); // Vhat_a(t)
    est_action_surplus_vals = arma::mat(m_naction, m_nt).fill(NA_REAL); // DeltaVhat_a(t)
    action_probs = arma::mat(m_naction, m_nt).fill(NA_REAL);    //  P_a(t)
    action_vals_given_behs = arma::cube(m_naction, N_BEHAVIORS, m_nt).fill(NA_REAL); // V_a|b(t)

    // states regarding the behavior of the other agent
    beh_activations = arma::mat(N_BEHAVIORS, m_nt).fill(NA_REAL); // A_b(t)
    beh_activ_V = arma::mat(N_BEHAVIORS, m_nt).fill(NA_REAL);     // ^V A_b(t)
    beh_activ_O = arma::mat(N_BEHAVIORS, m_nt).fill(NA_REAL);     // ^O A_b(t)
    beh_probs = arma::mat(N_BEHAVIORS, m_nt).fill(NA_REAL);       // P_b(t)
    sensory_probs_given_behs = arma::mat(N_BEHAVIORS, m_nt).fill(NA_REAL); // P_{x_o|b}(t)
    // the acceleration that the other agent should be applying right now if doing behaviour b
    beh_acceleration = arma::mat(N_BEHAVIORS, m_nt).fill(NA_REAL); 
    beh_vals_given_actions = arma::cube(N_BEHAVIORS, m_naction, m_nt).fill(NA_REAL); // V_b|a(t)

    // - other states
    m_time_left_to_CA_entry = arma::vec(m_nt).fill(NA_REAL);
    m_time_left_to_CA_exit = arma::vec(m_nt).fill(NA_REAL);

    // set initial values for states that depend on the previous time step
    // Using tail because Python -1 index grab the last element
    est_action_vals.tail_cols(1).zeros(); 
    beh_activ_V.tail_cols(1).zeros();
    beh_activ_O.tail_cols(1).zeros();
    beh_activ_O(i_CONSTANT, m_nt-1) = 10;

    //  warnings.warn('****** Setting initial value of i_CONSTANT behaviour activation to arbitrary high value.')
    m_optional_assumptions = optional_assumptions;

}

double SCAgent::get_distance(unsigned int i_time_step, arma::vec conflict_point, SCAgent* a)
{
  arma::vec vector2conflict_point = conflict_point - a->m_trajectory->m_position.col(i_time_step);
  arma::vec heading_vector(2);
  heading_vector[0] = std::cos(a->m_trajectory->m_yawangle[i_time_step]);
  heading_vector[1] = std::sin(a->m_trajectory->m_yawangle[i_time_step]);

  // if(i_time_step==92)
  // {
  //   conflict_point.t().print("cp");
  //   a->m_trajectory->m_position.col(i_time_step).t().print();
    // vector2conflict_point.t().print("vector2conflict_point");

    // heading_vector.t().print("heading_vector");

  // }
  return arma::dot(heading_vector, vector2conflict_point);
}

arma::vec SCAgent::add_action(unsigned int i_time_step, unsigned int i_action)
{
  arma::vec local_acceleration = m_tmp_acceleration;
  arma::vec::iterator it0     = local_acceleration.begin() + i_time_step;
  arma::vec::iterator it_end0 = it0 + m_npt;

  arma::vec::iterator it1     = local_acceleration.begin() + i_time_step + m_npt;
  arma::vec::iterator it_end1 = local_acceleration.end(); 

  double acc_value;
  
  if (m_p->type == 0) {
    
    // 0 = speed, after divided by predictino time becomeing acceleration ; 1 = yaw angle
    acc_value = m_actions(i_action, 0) / DeltaT; 
    for(; it0 != it_end0; ++it0) { (*it0) += acc_value; }

  } else if (m_p->type == 1) {
    
    // For the vehicle, the input value is acceleration straight
    acc_value = m_actions(i_action, 0); 
    arma::vec ramp = arma::linspace(0, acc_value, m_npt); 
    
    for(; it0 != it_end0; ++it0) 
    { 
      auto idx = (it0 - local_acceleration.begin()) - i_time_step;
      (*it0) += ramp[idx];
    }
    for(; it1 != it_end1; ++it1) (*it1) += acc_value;
    
  } else {
    Rcpp::stop("Unexpected control type");
  }
  return local_acceleration;
}

void SCAgent::update_kinematics(unsigned int i_time_step, unsigned int debug)
{
  m_i_time_step = i_time_step;
  // if (m_i_time_step == debug) 
  // {
  //   Rcpp::Rcout << m_name << "[" << m_i_time_step << "]:\n"; 
  //   m_trajectory->m_acceleration.subvec(0, debug-1).t().print();
  // }

  KinematicState tmp_ks = get_future_kinematic_state(m_trajectory->m_acceleration, m_trajectory->m_yawrate, i_time_step - 1, 1, this); 
  m_trajectory->m_acceleration[i_time_step] = tmp_ks.m_acceleration;
  m_trajectory->m_yawrate[i_time_step] = tmp_ks.m_yawrate;
  m_trajectory->m_position.col(i_time_step) = tmp_ks.m_position;
  m_trajectory->m_speed[i_time_step] = tmp_ks.m_speed;
  m_trajectory->m_yawangle[i_time_step] = tmp_ks.m_yawangle;

}

  // KinematicState tmp_ks = get_future_kinematic_state(m_trajectory->m_acceleration, m_trajectory->m_yawrate, i_time_step - 1, 1, this); 
    // out = get_future_kinematic_state(acceleration_for_this_beh, local_yawrate, i_time_step, m_npt, other_agent);

KinematicState SCAgent::get_future_kinematic_state(arma::vec local_acceleration, arma::vec local_yawrate, 
  unsigned int i_start_time_step, unsigned int n_time_steps_to_advance, SCAgent* a) 
{
  double tmp_acceleration  = local_acceleration[i_start_time_step];
  double tmp_yawrate       = local_yawrate[i_start_time_step]; 

  double local_speed       = a->m_trajectory->m_speed[i_start_time_step];
  double local_yawangle    = a->m_trajectory->m_yawangle[i_start_time_step];
  arma::vec local_position = a->m_trajectory->m_position.col(i_start_time_step);

  KinematicState local_state = KinematicState(local_speed, local_yawangle, tmp_acceleration, tmp_yawrate, local_position);

  unsigned int start = i_start_time_step + 1; // 1
  unsigned int end = i_start_time_step + n_time_steps_to_advance + 1; // 6
  arma::vec local_heading_direction(2);   

  // do the Euler time-stepping
  for(size_t i=start; i<end; i++)  // 1, 2, 3, 4, 5
  {
    local_state.m_speed += local_acceleration[i-1] * m_dt;
    if( !m_can_reverse && local_state.m_speed < 0) local_state.m_speed = 0;
    local_state.m_yawangle += local_yawrate[i-1] * m_dt;
    local_heading_direction[0] = std::cos(local_state.m_yawangle);
    local_heading_direction[1] = std::sin(local_state.m_yawangle);
    local_state.m_position = local_state.m_position + m_dt * local_state.m_speed * local_heading_direction;
  }


  return local_state;
}

KinematicState SCAgent::get_future_kinematic_state(double local_acceleration, double local_yawrate, unsigned int i_start_time_step, unsigned int n_time_steps_to_advance, SCAgent* a)
{
  // Use in SCAgent::get_predicted_other_state
  double local_speed       = a->m_trajectory->m_speed[i_start_time_step];
  double local_yawangle    = a->m_trajectory->m_yawangle[i_start_time_step];
  arma::vec local_position = a->m_trajectory->m_position.col(i_start_time_step);

  KinematicState local_state = KinematicState(local_speed, local_yawangle, local_acceleration, local_yawrate, local_position);

  unsigned int start = i_start_time_step + 1; // 1
  unsigned int end = i_start_time_step + n_time_steps_to_advance + 1; // 6
  arma::vec local_heading_direction(2);   

  // do the Euler time-stepping
  for(size_t i=start; i<end; i++)  // 1, 2, 3, 4, 5
  {
    local_state.m_speed += local_acceleration * m_dt;
    if( !m_can_reverse && local_state.m_speed < 0) local_state.m_speed = 0;
    local_state.m_yawangle += local_yawrate * m_dt;
    local_heading_direction[0] = std::cos(local_state.m_yawangle);
    local_heading_direction[1] = std::sin(local_state.m_yawangle);
    local_state.m_position = local_state.m_position + m_dt * local_state.m_speed * local_heading_direction;
  }


  return local_state;

}

KinematicState SCAgent::get_predicted_own_state(unsigned int i_time_step, unsigned int i_action)
{
  arma::vec local_acceleration = add_action(i_time_step, i_action); // nat
 
  arma::vec local_yawrate = m_trajectory->m_yawrate;                
  KinematicState predicted_state = get_future_kinematic_state(local_acceleration, local_yawrate, i_time_step, m_npt, this);

  // change it to i_time_step + 5?
  predicted_state.m_acceleration = local_acceleration[i_time_step + m_npt];

  return predicted_state;
}

KinematicState SCAgent::get_predicted_other_state(unsigned int i_time_step, unsigned int i_beh, SCAgent* other_agent)
{
  // get the (longitudinal) acceleration for this behaviour, if implemented at the current time step
  double acceleration_for_this_beh = beh_acceleration(i_beh, i_time_step); // beh_accs stores info about the other agent
  double local_yawrate = 0;
  arma::vec local_position; local_position.zeros(2); 
  KinematicState out(0, 0, 0, 0, local_position);
  
  if ( ISNAN(acceleration_for_this_beh) ) {
  } else {
    // let the other agent calculates what its predicted state would be with this acceleration 
    out = get_future_kinematic_state(acceleration_for_this_beh, local_yawrate, i_time_step, m_npt, other_agent);
    out.m_acceleration = acceleration_for_this_beh; // change it to scalar
  }
  
  return out;
  
}

double SCAgent::get_time_to_agent_collision(KinematicState state1, KinematicState state2, double collision_distance)
{
  // collision already happening?
  if (arma::norm(state2.m_position - state1.m_position) <= collision_distance) return 0;
 
  double delta_x = state2.m_position[0] - state1.m_position[0];
  double delta_y = state2.m_position[1] - state1.m_position[1];
  double delta_v_x = state2.m_speed * std::cos(state2.m_yawangle) -
    state1.m_speed * std::cos(state1.m_yawangle);
  double delta_v_y = state2.m_speed * std::sin(state2.m_yawangle) -
    state1.m_speed * std::sin(state1.m_yawangle);
  // Rcpp::Rcout << "[delta_x delta_v_x delta_y delta_v_y] " << delta_x << "\t" << 
  //   delta_v_x << "\t" << delta_y << "\t" << delta_v_y << "\n";  
  // get coefficients of quadratic equation for squared distance
  // D^2 = at^2 + bt + c 
  double a = delta_v_x*delta_v_x + delta_v_y*delta_v_y;
  double b = 2. * (delta_x * delta_v_x + delta_y * delta_v_y);
  double c = delta_x*delta_x + delta_y*delta_y;
  // Rcpp::Rcout << "[a b c] " << a << " " << b << " " << c - collision_distance*collision_distance << "\n";

  // get roots t for D^2 = D_collision^2 <=> D^2 - D_collision^2 = 0
  std::vector<double> coll_times = findRoots(a, b, c - collision_distance*collision_distance);
  double time2collision;
  if (coll_times.size() == 1) {
      
      // if len(coll_times) == 0: no collision (identical agent headings)
      // return math.inf
      // elif len(coll_times) == 1: just barely touching
      // return coll_times[0]
      time2collision = (std::isnan(coll_times[0])) ? R_PosInf : coll_times[0]; 
      
  } else {
      // passing collision threshold twice (typical case) - check when this happens
      if ( std::copysign(1, coll_times[0]) == std::copysign(1, coll_times[1]) )
      {
        //  both in future or both in past 
        // if coll_times[0] > 0: both in future return min(coll_times)
        // else: both in past return math.inf
        time2collision = (coll_times[0] > 0) ? std::min(coll_times[0], coll_times[1]) : R_PosInf;
      } 
      else // one in future one in past - i.e., collision ongoing now
      {
        time2collision = 0;
      }
  }
  // Rcpp::Rcout << "time2collision: " << time2collision << "\n";
    
  return time2collision;
}

double SCAgent::get_value_of_state_for_agent(KinematicState own_state, KinematicState oth_state, SCAgent * a) 
{
  arma::vec heading_vector(2);
  heading_vector[0] = std::cos(own_state.m_yawangle);
  heading_vector[1] = std::sin(own_state.m_yawangle);
  
  // reward for progress toward goal and speed discomfort cost
  arma::vec vector_to_goal = a->m_goal - own_state.m_position;
  double goal_distance = arma::norm(vector_to_goal);
  arma::vec heading_to_goal_vector = vector_to_goal / goal_distance;
  double heading_toward_goal_component = arma::dot(heading_vector, heading_to_goal_vector);
  double goal_distance_change_rate = -heading_toward_goal_component * own_state.m_speed;
  double value = -(a->m_p->kg) * goal_distance_change_rate - a->m_p->kdv * own_state.m_speed*own_state.m_speed; 

  // Rcpp::Rcout << "get_value_of_state_for_agent: " << value << "\n";
  if (a->m_p->type == 1)
  {
    // acceleration discomfort cost
    // Rcpp::Rcout << "In type == 1; value: " << value << "\n";
    // print("In type == 1; kda: ", k._da, "\t", own_state.long_acc)
    value += (-a->m_p->kda) * own_state.m_acceleration*own_state.m_acceleration;
    // Rcpp::Rcout << "In type == 1; value: " << value << "\n";
    // cost for acceleration required to stop at goal
    double req_acc_to_goal = -(own_state.m_speed*own_state.m_speed / (2. * goal_distance));
    value += -(a->m_p->ksg) * req_acc_to_goal*req_acc_to_goal;
    // Rcpp::Rcout << "In type == 1; value: " << value << "\n";

  }

  // cost for being on collision course with the other agent
  double time_to_agent_collision = get_time_to_agent_collision(own_state, oth_state, d_C);
  if( time_to_agent_collision == 0) { time_to_agent_collision = TTC_FOR_COLLISION;}

  if( time_to_agent_collision < INFINITY)
  {
    if (a->m_p->type == 0) {      // other agent's control type
      value += -(a->m_p->kc) / time_to_agent_collision;
    } else if (m_p->type == 1) {  // self agent's cntrol type
      double term0 = (own_state.m_speed / (2. * time_to_agent_collision));
      value += -(a->m_p->ksc)*term0*term0;

    } else {

    }

  }
  
  return value;
}

double SCAgent::get_value_for_me(KinematicState my_state, KinematicState oth_state, double action)
{
  // cost for making a speed change, if any
  double value = -m_p->ke*action*action;
  // Rcpp::Rcout << "get_value_for_me before: " << m_p->ke << "\t" << action <<  "\t" << value << "\n";  
  // Rcpp::Rcout << "get_value_for_me before: " << value << "\n";
  // add value of the state
  value += get_value_of_state_for_agent(my_state, oth_state, this);
  // Rcpp::Rcout << "get_value_for_me after: " << value << "\n";
  return value;
}

double SCAgent::get_value_for_other(KinematicState oth_state, KinematicState my_state, SCAgent * oth_a)
{
  double value = 0;
  value += get_value_of_state_for_agent(oth_state, my_state, oth_a);
  return value;
}

double SCAgent::get_prob_of_current_state_given_beh(unsigned int i_time_step, unsigned int i_beh, SCAgent * oth_a)
{
  double prob_density;
  unsigned int i_prev_time_step = i_time_step - 1;

  if ( ISNAN(beh_acceleration(i_beh, i_prev_time_step)) ) {
    prob_density = 1e-10;
  } else {
    // retrieve the longitudinal acceleration for this behavior, as estimated on the previous time step
    double prev_acceleration_for_this_beh = beh_acceleration(i_beh, i_prev_time_step);
    double dummy_yawrate = 0;

    // let the other agent object calculate what its predicted state at the current time step would be with this acceleration     
    KinematicState expected_curr_state = 
      get_future_kinematic_state(prev_acceleration_for_this_beh, dummy_yawrate, i_prev_time_step, 1, oth_a);
    // if (i_time_step == 3)
    // {
    //   Rcpp::Rcout << "prev_acceleration_for_this_beh " << prev_acceleration_for_this_beh << "\n";
    //   expected_curr_state.m_position.t().print();
    // }

    // get the distance between expected and observed position
    double pos_diff = arma::norm(expected_curr_state.m_position - oth_a->m_trajectory->m_position.col(i_time_step));
    // return the probability density for this observed difference, log = FALSE
    prob_density = R::dnorm(pos_diff, 0, sigma_ao, 0);
    // if (i_time_step == 3)
    // {
    //   Rcpp::Rcout << "prob_density " << prob_density << "\n\n";
    // }
  }
  
  return std::max(prob_density, 1e-10);
}

void SCAgent::clear_negative_acceleration(unsigned int i_time_step)
{
  if (!m_can_reverse && m_trajectory->m_speed[i_time_step] == 0)
  {
      // start from column i to the last column (0 indexing)
      // Rcpp::Rcout << " 0 speed\n";
      arma::vec::iterator it     = m_tmp_acceleration.begin()+i_time_step;  
      arma::vec::iterator it_end = m_tmp_acceleration.end();
      for(; it != it_end; ++it) (*it) = std::max(0.0, (*it));
  }
}

void SCAgent::update_action(unsigned int i_time_step, arma::vec conflict_point, SCAgent* other_agent, 
  arma::uvec optional_assumptions, unsigned int debug)
{
  // When the agent can't reverse and its speed is at zero, make sure that any future negative accelerations from past actions are cleared
  clear_negative_acceleration(i_time_step);

  if (i_time_step == debug) 
  {
    Rcpp::Rcout << m_name + " at times[" << i_time_step << "]: " << m_times[i_time_step] << " s\n";
    // m_trajectory->m_speed.subvec(0,debug-1).t().print();
    // m_trajectory->m_position.cols(0, debug-1).print();
   
    // m_tmp_acceleration.t().print("m_trajectory->m_acceleration.subvec");

    // proj_speed.t().print("proj_long_speeds");
    // Rcpp::Rcout << " m_p->beta_O " <<  m_p->beta_O << "\n";
    // proj_signed_dist_to_CP.t().print("proj_signed_dist_to_CP");
  }

  // calculate my own current projected time until entering and exiting the conflict area
  arma::vec proj_speed = m_trajectory->m_speed[i_time_step] + 
    arma::cumsum( m_tmp_acceleration.subvec(i_time_step, m_nat-1)*m_dt );

  if (!m_can_reverse)
  {
    arma::vec::iterator it     = proj_speed.begin();
    arma::vec::iterator it_end = proj_speed.end();
    for(; it != it_end; ++it) (*it) = std::max(0.0, (*it));
  }

  // conflict_point.t().print("conflict point");
  double signed_dist_to_confl_pt = get_distance(i_time_step, conflict_point, this);
  arma::vec proj_signed_dist_to_CP = signed_dist_to_confl_pt - arma::cumsum(proj_speed*m_dt);


  //- entry and exit of the presumed collision zone 
  arma::uvec i_entered_CZ = arma::find(proj_signed_dist_to_CP < m_collision_distance); // m_collision_distance = d_C = 3
  arma::uvec i_exited_CZ = arma::find(proj_signed_dist_to_CP < -m_collision_distance);
  m_time_left_to_CA_entry[i_time_step] = (i_entered_CZ.n_elem==0) ? INFINITY : i_entered_CZ[0] * m_dt;
  m_time_left_to_CA_exit[i_time_step] = (i_exited_CZ.n_elem==0) ? INFINITY : i_exited_CZ[0] * m_dt;

  // if (i_time_step == debug) 
  // {
  //     beh_acceleration.col(i_time_step).t().print("Before ");
  // }
  // calculate the accelerations needed for the different behaviors of the other agent, as of the current time 
  beh_acceleration(i_CONSTANT, i_time_step) = 0;   // other agent reacts idetnically all the time???
  // prepare vector for noting if one of the behaviours is invalid for this time step
  arma::uvec beh_is_valid = arma::ones<arma::uvec>(N_BEHAVIORS);

  // proceeding behaviour??? 
  if (other_agent->m_p->type == 0) {  // other agent is a pedestrian, assuming straight acc to free speed
      beh_acceleration(i_PROCEEDING, i_time_step) = 
        (other_agent->m_free_speed - other_agent->m_trajectory->m_speed[i_time_step]) / DeltaT;
      // if (i_time_step == debug) 
      // {
      //     beh_acceleration.col(i_time_step).t().print("In proceeding if ");
      // }
  } else {
      // calculate the expected acceleration given the current deviation from the free speed (see hand written notes from 2020-07-08)
      double dev_from_v_free = other_agent->m_trajectory->m_speed[i_time_step] - other_agent->m_free_speed;
      beh_acceleration(i_PROCEEDING, i_time_step) = -other_agent->m_p->kdv * dev_from_v_free * m_pt /  
                    (0.5 * other_agent->m_p->kdv * m_pt*m_pt + 2. * other_agent->m_p->kda);

      // Rcpp::Rcout << "m_free_speed " << other_agent->m_free_speed << " " << beh_acceleration(i_PROCEEDING, i_time_step) << "\n";
  }

  // yielding behavior ??? --------------------------*/
  // if(i_time_step == debug)
  // {
  //   other_agent->m_trajectory->m_position.col(i_time_step).t().print("right before ");
  // }

  double oth_signed_dist_to_confl_pt = get_distance(i_time_step, conflict_point, other_agent);
  double oth_signed_dist_to_CA_entry = oth_signed_dist_to_confl_pt - m_collision_distance;

  // if (i_time_step == debug)
  // {
  //   Rcpp::Rcout << "pt " << oth_signed_dist_to_confl_pt <<  " m_collision_distance " << m_collision_distance << "\n";
  //   Rcpp::Rcout << "oth_signed_dist_to_CA_entry " << oth_signed_dist_to_CA_entry << "\n";
    
  // }
  if (oth_signed_dist_to_CA_entry > 0) {
      beh_acceleration(i_YIELDING, i_time_step) = -(other_agent->m_trajectory->m_speed[i_time_step] * other_agent->m_trajectory->m_speed[i_time_step]) / 
            (2. * oth_signed_dist_to_CA_entry);

      // if (i_time_step == debug) 
      // {
      //   Rcpp::Rcout << "speed " << other_agent->m_trajectory->m_speed[i_time_step] << "\n";
      //   Rcpp::Rcout << "CA entry " << oth_signed_dist_to_CA_entry << "\n";
      //   Rcpp::Rcout << "i_YI " << beh_acceleration(i_YIELDING, i_time_step) <<  "\t" ;
      // }

  } else {
      beh_acceleration(i_YIELDING, i_time_step) = NA_REAL;
      beh_is_valid[i_YIELDING] = 0;
      // if (i_time_step == debug) 
      // {
      //     Rcpp::Rcout << "other_state.long_speed " << other_agent->m_trajectory->m_speed[i_time_step] <<  "\t" <<
      //     oth_signed_dist_to_CA_entry << "\n";
      // }
      // Rcpp::Rcout << "Yielding " << beh_acceleration(i_YIELDING, i_time_step) << "\n";
  }

  // if(i_time_step == debug)
  // {
  //   beh_acceleration.col(i_time_step).t().print("after");
  // }

  // do first loops over all own actions and behaviours of the other agent, and get the predicted states 
  std::vector<KinematicState> pred_own_states(m_naction);
  std::vector<KinematicState> pred_oth_states(N_BEHAVIORS);
  for (size_t k=0; k < m_naction; k++) pred_own_states[k] = get_predicted_own_state(i_time_step, k);
  for (size_t l=0; l < N_BEHAVIORS; l++) pred_oth_states[l] = get_predicted_other_state(i_time_step, l, other_agent);

  // if(i_time_step == debug)
  // {
    // m_trajectory->m_acceleration.t().print("m_trajectory->m_acceleration");
    // for (size_t k=0; k < m_naction; k++) pred_own_states[k].print(std::to_string(k));
    // for (size_t l=0; l < N_BEHAVIORS; l++) pred_oth_states[l].print(std::to_string(l));
  // }
  // Rcpp::Rcout << "\n\n";

  //------------------------------------------------- --------------------------
  // now loop over all combinations of own actions and other's behaviors, and get values from both agents' perspectives   
  arma::vec mag; // column 0 = acceleration; column 1 = yawrate
  for (size_t k=0; k < m_naction; k++)
  {
    mag = m_actions.row(k).t();
        
    for (size_t l=0; l < N_BEHAVIORS; l++)
    {
      if (beh_is_valid[l])
      {
        // get value for me of this action/behaviour combination; naction x nbehaviour x nt
        action_vals_given_behs(k, l, i_time_step) = get_value_for_me(pred_own_states[k], pred_oth_states[l], mag[0]);
        // get value for the other agent of this action/behaviour combination
        beh_vals_given_actions(l, k, i_time_step) = get_value_for_other(pred_oth_states[l], pred_own_states[k], other_agent);
      }
    } // end of behaviour loop
  }   // end of action loop

  // if (i_time_step == debug) 
  // {
  //    action_vals_given_behs.slice(i_time_step).print("action_vals_given_behs");
  //    beh_vals_given_actions.slice(i_time_step).print("beh_vals_given_actions");
  // }

  // (i_time_step - 1) starts to appear
  // get my estimated probabilities for my own actions - based on value estimates from the previous time step
  // Resolve the Python negative indexing trick.
  if (i_time_step==0) {
    action_probs.col(i_time_step) = softmax(Lambda * est_action_vals.tail_cols(1)); 
  } else {
    action_probs.col(i_time_step) = softmax(Lambda * est_action_vals.col(i_time_step-1)); 
  }

  // if (i_time_step == debug) 
  // {
  //   est_action_vals.col(debug).t().print("Not yet fill in ");

  //   // arma::vec tmp = action_probs.col(i_time_step);
  //   // tmp.t().print("action_probs.col");
  // }

  // now loop over the other agent's behaviors to update the corresponding activations (my "belief" in these behaviors)
  for (size_t l=0; l < N_BEHAVIORS; l++)
  {
    if (!beh_is_valid[l]) {
      beh_activ_V(l, i_time_step) = 0; // V_{A_b}(t)
      beh_activ_O(l, i_time_step) = 0;
    } else {
      // - update the game theoretic activation contribution from the previous time step
      if (i_time_step == 0) {
        beh_activ_V(l, i_time_step) = gamma * beh_activ_V(l, m_nt-1);
        // beh_activ_V.col(0).t().print("beh_activ_V");
      } else {
        beh_activ_V(l, i_time_step) = gamma * beh_activ_V(l, i_time_step-1);
      }

      // if (i_time_step == debug)
      // {
      //   Rcpp::Rcout << " " << beh_activ_V(l, i_time_step) << "\n";
      // }


      // - contributions from estimated value of the behaviour to the other agent, given my estimated probabilities of my actions
      for (size_t k=0; k < m_naction; k++)
      {
        // if(i_time_step == debug)
        // {
        //   Rcpp::Rcout << "beh_activ_V" << beh_activ_V(l, i_time_step) << "\n";
        // }
        beh_activ_V(l, i_time_step) +=  (1 - gamma) * action_probs(k, i_time_step) * beh_vals_given_actions(l,k,i_time_step);

        // if(i_time_step == debug)
        // {
        //   Rcpp::Rcout << "beh_activ_V \t" << beh_activ_V(l, i_time_step) << "\n";
          // beh_activ_V.print("beh_activ_V");
          // Rcpp::Rcout << "[" << k << " ] action_probs " << action_probs(k, i_time_step) << "\t" <<
          //  beh_vals_given_actions(l,k,i_time_step) << "\n";
        // }

      }

      // if (i_time_step == debug)
      // {
      //   Rcpp::Rcout << "V " << beh_activ_V(l, i_time_step) << "\n";
      //   Rcpp::Rcout << "O " << beh_activ_O(l, i_time_step) << "\n";

        // if (i_time_step == 0) {
        //   Rcpp::Rcout << "O " << beh_activ_O(l, m_nt-1) << "\n";  
        // } else {
        //   Rcpp::Rcout << "O " << beh_activ_O(l, i_time_step-1) << "\n";
        // }
      // }

      // update the "Kalman filter" activations
      if (i_time_step==0) {
        beh_activ_O(l, i_time_step) = kappa * beh_activ_O(l, m_nt-1);
      } else {
        beh_activ_O(l, i_time_step) = kappa * beh_activ_O(l, i_time_step-1);
      }

      // if (i_time_step == debug)
      // {
      //   Rcpp::Rcout << "After 1: " << beh_activ_O(l, i_time_step) << "\n";  
      // }

      if (i_time_step > 0)
      {
        sensory_probs_given_behs(l, i_time_step) = get_prob_of_current_state_given_beh(i_time_step, l, other_agent);
        beh_activ_O(l, i_time_step) += (1. - kappa) * (1./Lambda )* std::log(sensory_probs_given_behs(l, i_time_step));
      }

      // if (i_time_step == debug)
      // {
      //   Rcpp::Rcout << "After 2: " << beh_activ_O(l, i_time_step) << "\n";
      // }

    }  // end of if-else
  }    // enf of behaviour loop

  // if (i_time_step == debug) 
  // {
  //   beh_activ_V.col(0).t().print("beh_activ_V");
  //   beh_activ_O.col(0).t().print("beh_activ_O");
  // }

  // get total activation for all behaviors of the other agent
  beh_activations.col(i_time_step) = m_p->beta_V * beh_activ_V.col(i_time_step) + m_p->beta_O * beh_activ_O.col(i_time_step);

  // if (i_time_step==debug)
  // {
  //   beh_activations.col(i_time_step).print("beh_activations");
  //   Rcpp::Rcout << "beta_v and beta_O " << m_p->beta_V << "  " << m_p->beta_O << "\n";
  // }

  // get my estimated probabilities for the other agent's behavior; OBE
  if (optional_assumptions[4]) {
      arma::uvec i_beh_is_valid = arma::find(beh_is_valid);
      arma::vec tmp0 = beh_activations.col(i_time_step);
      arma::vec tmp1 = softmax( Lambda * tmp0.elem(i_beh_is_valid) );

      for (size_t m=0; m < beh_is_valid.n_elem; m++) 
      {
        beh_probs(m, i_time_step) = !beh_is_valid[m] ? 0 : tmp1[m];
        // beh_probs(i_beh_is_valid[m], i_time_step) = tmp1[m];
      }

      // if (i_time_step==debug)
      // {
      //   i_beh_is_valid.t().print();
      // //   // tmp0.t().print();
      // //   // tmp1.t().print();
      //   beh_probs.col(i_time_step).t().print("beh_probs");
      //   Rcpp::Rcout << "Lambda " << Lambda << "\t";
      //   beh_activations.col(i_time_step).t().print("beh acti ");
      // }

  } else {
      beh_probs.col(i_time_step).zeros();
      beh_probs(i_CONSTANT, i_time_step) = 1;
  }

  // if (i_time_step==debug)
  // {
  //   beh_probs.col(i_time_step).t().print("beh_probs");
  //   // action_vals_given_behs.slice(i_time_step).print();
  // }
  
  // Loop through own action options and get momentary estimates of the actions' values to me, as weighted average over the other 
  // agent's behaviors 
  mom_action_vals.col(i_time_step).zeros();
  for (size_t k=0; k < m_naction; k++)
  {
    for (size_t l=0; l < N_BEHAVIORS; l++)
    {
      if (beh_is_valid[l] && beh_probs(l, i_time_step) > MIN_BEH_PROB) 
      {
        // if (i_time_step == debug){
        //   Rcpp::Rcout << "beh\taction: " << beh_probs(l, i_time_step) << "\t" << action_vals_given_behs(k, l, i_time_step) << "\n";
        // }
        
        mom_action_vals(k, i_time_step) += beh_probs(l, i_time_step) * action_vals_given_behs(k, l, i_time_step);
      } 
    }
  }

  // update the accumulative estimates of action value
  arma::vec value_noise = arma::randn(m_naction) * m_p->sigma_V * std::sqrt(m_dt); 

  // if (i_time_step == debug && i_time_step != 0) 
  // {
  //   // value_noise.t().print("value_noise");
  //   mom_action_vals.col(i_time_step).t().print("mom");
  //   // mom_action_vals.print("mom_action_vals");
  // }

  if (i_time_step == 0) {
    est_action_vals.col(i_time_step) = m_p->alpha * est_action_vals.tail_cols(1) + (1 - m_p->alpha) * 
      (mom_action_vals.col(i_time_step) + value_noise);

  } else {
    est_action_vals.col(i_time_step) = m_p->alpha * est_action_vals.col(i_time_step - 1) + (1 - m_p->alpha) * 
      (mom_action_vals.col(i_time_step) + value_noise);

    // if (i_time_step == debug)
    // {
    //   Rcpp::Rcout << "alpha " << m_p->alpha << "\n"; 
    //   est_action_vals.col(i_time_step-1).t().print("est_action_vals previous ");
    //   est_action_vals.col(i_time_step).t().print("filled in ");

    //   // mom_action_vals.col(i_time_step).t().print("mom_action_vals");
    // }

  }
  
  // if (i_time_step == debug && i_time_step != 0) 
  // {
  //   Rcpp::Rcout << "alpha = " << m_p->alpha << "\n";
  //   est_action_vals.col(i_time_step-1).t().print();
  //   est_action_vals.col(i_time_step).t().print();
  //   mom_action_vals.col(i_time_step).t().print();
  // }
  est_action_surplus_vals.col(i_time_step) = est_action_vals.col(i_time_step) - est_action_vals(i_NO_ACTION, i_time_step);
  unsigned int i_best_action = est_action_surplus_vals.col(i_time_step).index_max();

  // if (i_time_step == debug) 
  // {
  //   est_action_surplus_vals.col(i_time_step).t().print("surplus_vals");
  //   est_action_vals.col(i_time_step).t().print("vals");
  //   Rcpp::Rcout << "est_action_vals " << est_action_vals(i_NO_ACTION, i_time_step) << "\n";
  //   Rcpp::Rcout << "i_best_action " << i_best_action << "\n\n";

  //   bool tmp = est_action_surplus_vals(i_best_action, i_time_step) > m_p->DeltaV_th;
  //   Rcpp::Rcout << "value " <<  est_action_surplus_vals(i_best_action, i_time_step) << " " << tmp << "\n";
  // }


  // any action over threshold?
  if (est_action_surplus_vals(i_best_action, i_time_step) > m_p->DeltaV_th)
  {
    m_tmp_acceleration = add_action(i_time_step, i_best_action);
    est_action_vals.col(i_time_step).zeros(); 
  }



  // set long acc in actual trajectory
  m_trajectory->m_acceleration[i_time_step] = m_tmp_acceleration[i_time_step];
  if (i_time_step == debug) 
  {
    Rcpp::Rcout << "[i_best_action m_acceleration]: " << i_best_action << "\t " << m_tmp_acceleration[i_time_step] << "\t" <<
    m_trajectory->m_acceleration[i_time_step] << "\n";
  }


}