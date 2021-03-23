#include <commotions.h>

using arma::vec;
using arma::mat;

Simulation::Simulation(vec t) 
{
  m_st = t[0];  // start time
  m_et = t[1];  // end time
  m_dt = t[2];  // time step
  m_pt = t[3];  // prediction time
  
  double re = remainder(m_et, m_dt);  // to match numpy arange
  if (re < 1e-10) { 
    arma::vec tmp = arma::regspace(m_st, m_dt, m_et);
    m_times = tmp.subvec(0, tmp.size() - 2); 
  } else {
    m_times = arma::regspace(m_st, m_dt, m_et);
  }
  
  nt  = m_times.size();
  npt = std::ceil(m_pt / m_dt); // PREDICTION_TIME_LENGTH
  nat = npt + nt;               // ACTIONS_VECTOR_LENGTH
}

void Simulation::Show(std::string str) const 
{
  using namespace Rcpp;
  Rcout << str << ":\n";
  m_times.t().print("time_stamps");
  Rcout << "number of time stamps: " << nt << "\nprediciton time length: " 
        << npt << "\naction vector length: "<< nat << "\n";
}

void Simulation::initialize(std::vector<Kinematics *> agents)
{
  m_agents = agents;
  // Containers
  unsigned na = m_agents.size();
  m_accs      = arma::zeros<mat>(na, nat); // action_long_accs; not long_accs
  m_sw_angles = arma::zeros<mat>(na, nat); // yaw rate
  
  m_speeds    = arma::zeros<mat>(na, nt);
  m_angles    = arma::zeros<mat>(na, nt);
  m_xs        = arma::zeros<mat>(na, nt);
  m_ys        = arma::zeros<mat>(na, nt);
  m_yaw_rates = arma::zeros<mat>(na, nt);
  
  for (size_t i=0; i<na; i++)
  {
    m_xs(i, 0) = m_agents[i]->m_x;
    m_ys(i, 0) = m_agents[i]->m_y;
    m_speeds(i, 0) = m_agents[i]->m_speed;
    m_angles(i, 0) = m_agents[i]->m_angle;
  }
}

void Simulation::advance_Pedestrian(unsigned j, unsigned i, unsigned nstep)
{
  // j = i_agent; i = i_time_stamp; (always?) nstep = 1
  double lx = this->m_xs(j, i);  // local_x & local_y
  double ly = this->m_ys(j, i);  
  double ls = this->m_speeds(j, i); // local_speed & local_angle
  double la = this->m_angles(j, i);
  
  // 2; 3; 4
  for (size_t k=(i+1); k < (i+nstep+1); k++)
  {
    ls += this->m_accs(j, k-1) * this->m_dt;  // k-1 is the current step
    if (ls < 0) ls = 0;
    
    la += this->m_sw_angles(j, k-1) * this->m_dt; // yaw rate
    
    lx += this->m_dt * ls * std::cos(la);
    ly += this->m_dt * ls * std::sin(la);
  }
  this->m_xs(j, i+1) = lx;      // make a move to a next step; 
  this->m_ys(j, i+1) = ly;      // na x nt
  this->m_speeds(j, i+1) = ls;
  this->m_angles(j, i+1) = la;
}

void Simulation::advance_Pedestrian(unsigned int j, unsigned int i,
                        arma::vec pred_acc, arma::vec pred_yaw_rate,
                        arma::vec & out)
{
  out[0] = this->m_xs(j, i);
  out[1] = this->m_ys(j, i);
  out[2] = this->m_speeds(j, i);
  out[3] = this->m_angles(j, i);
  
  for (size_t k=(i+1); k < (i+this->npt+1); k++)
  {
    out[2] += pred_acc(k-1) * this->m_dt;
    out[3] += pred_yaw_rate(k-1) * this->m_dt;
    out[0] += this->m_dt * out[2] * std::cos(out[3]);
    out[1] += this->m_dt * out[2] * std::sin(out[3]);
  }
}

void Simulation::advance_Vehicle(unsigned int j, unsigned int i, unsigned int nstep)
{
  // j = i_agent; i = i_time_stamp; nstep = 1
  double lx  = this->m_xs(j, i);     // local_x & local_y
  double ly  = this->m_ys(j, i); 
  double ls  = this->m_speeds(j, i); // local_speed & local_angle
  double la  = this->m_angles(j, i);
  double lyr = NA_REAL;  // local_yaw_rate
  
  for (size_t k=(i+1); k<(i+nstep+1); k++)
  {
    ls += this->m_accs(j, k-1) * this->m_dt;
    if (ls < 0) ls = 0;
    
    // m_sr: steering ratio
    lyr = this->m_agents[j]->m_sr * ls * this->m_sw_angles(j, k-1);
    la += lyr * this->m_dt;
    
    lx += this->m_dt * ls * std::cos(la);
    ly += this->m_dt * ls * std::sin(la);
  }
  // Here used i+1, because we advanced from a previous time step
  this->m_xs(j, i+1) = lx; 
  this->m_ys(j, i+1) = ly;
  this->m_speeds(j, i+1) = ls;
  this->m_angles(j, i+1) = la;
  this->m_yaw_rates(j, i+1) = lyr;
}

void Simulation::advance_Vehicle(unsigned int j, 
                     arma::vec local_acc,
                     arma::vec local_sw_angles,
                     unsigned int i,
                     arma::vec & out)
{
  // j = i_agent; i = i_time_stamp; nstep = 1
  out[0] = this->m_xs(j, i);  // local_x & local_y
  out[1] = this->m_ys(j, i);
  out[2] = this->m_speeds(j, i);
  out[3] = this->m_angles(j, i);
  // out[5] = NA_REAL;
  
  for (size_t k=(i+1); k < (i+1+this->npt); k++)
  {
    out[2] += local_acc(k-1) * this->m_dt;
    if (out[2] < 0) out[2] = 0;
    out[5] = this->m_agents[j]->m_sr * out[2] * local_sw_angles(k-1);
    
    out[3] += out[5] * this->m_dt;
    out[0] += this->m_dt * out[2] * std::cos(out[3]);
    out[1] += this->m_dt * out[2] * std::sin(out[3]);
  }
  
  out[4] = local_acc(i+this->npt);
}


void Simulation::get_pos_predictions(unsigned int j, unsigned int i, arma::mat & pos_preds)
{
  // j = i_agent; i = i_time_stamp; nstep = 1
  double curr_x = this->m_xs(j, i);
  double curr_y = this->m_ys(j, i);
  double curr_speed = this->m_speeds(j, i);
  double curr_angle = this->m_angles(j, i);
  
  pos_preds(0, j) = curr_x + curr_speed * this->m_pt * std::cos(curr_angle);
  pos_preds(1, j) = curr_y + curr_speed * this->m_pt * std::sin(curr_angle);
}


void Simulation::add_action_Pedestrian(arma::vec & out, double action_magnitude, 
                           unsigned int i)
{ 
  // Pedestrians; i = i_time_stamp
  arma::uvec rng = arma::regspace<arma::uvec>(i, 1, i + this->npt - 1);
  // arbitrarily divided by the prediction time (usually .3) 
  double increment = action_magnitude / this->m_pt;
  for (size_t j = 0; j < rng.size(); j++) out[rng[j]] += increment;
}


void Simulation::add_action_Vehicle(arma::vec & out, double action_magnitude, 
                        unsigned int i)
{
  // Vehicles
  arma::uvec rng  = arma::regspace<arma::uvec>(i, 1, i + this->npt - 1);
  arma::vec increment = arma::linspace(0, action_magnitude, this->npt);
  for (size_t j = 0; j < rng.size(); j++) out[rng[j]] += increment[j];
  out.subvec(i+this->npt, out.n_elem - 1) += action_magnitude;
}

bool Simulation::is_cross(unsigned int i_time_stamp, std::vector<double> wall) 
{
  
  arma::vec segment0 = { this->m_xs(0, i_time_stamp),
                         this->m_ys(0, i_time_stamp),
                         this->m_xs(1, i_time_stamp),
                         this->m_ys(1, i_time_stamp) };
  
  int test0 = f(segment0[0], segment0[1], wall);
  int test1 = f(segment0[2], segment0[3], wall);
  int test2 = f(wall[0], wall[1], segment0);
  int test3 = f(wall[2], wall[3], segment0);
  bool out  = (test0 != test1) && (test2 != test3) ? true : false;
  return out;
}
