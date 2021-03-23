#include <commotions.h> 

using arma::vec;
using arma::mat;
using arma::regspace;
using Rcpp::List;
using Rcpp::stop;

//' @export
// [[Rcpp::export]]
arma::vec test_regspace(double st, double dt, double et) 
{
  arma::vec out;
  double re = remainder(et, dt);
  if (re < 1e-10) { 
    vec times = regspace(st, dt, et);
    out = times.subvec(0, times.size()-2);
  } else {
    out = regspace(st, dt, et);
  }
  return out;
}

//' @export
// [[Rcpp::export]]
arma::vec test_armaregspace(double st, double dt, double et) 
{
  return regspace(st, dt, et);
}

//' @export
// [[Rcpp::export]]
void test_Kinematics_old(arma::mat pos, arma::mat goal,
                     arma::vec speed, arma::vec angle,
                     arma::mat soption, arma::mat yoption) 
{
  if (pos.n_cols != 2 || goal.n_cols != 2) 
    Rcpp::stop("Position must be on a Cartesian coordinate");
  unsigned int na = pos.n_rows;  // number of agent
  std::vector<Kinematics *> agents(na);
  auto s0 = "Kinematics ";  // String to print
  auto s1 = " ";            // String to print
  
  std::vector<double> x = {1, .3, .3, .3};
  Parameters * p0 = new Parameters(x);

  for(size_t i=0; i<na; i++)
  {
    agents[i] = new Kinematics (pos.row(i).t(), goal.row(i).t(), speed[i], 
                                angle[i], p0);
    agents[i]->set_speeds(soption(0,i), soption(1,i), soption(2,i));
    agents[i]->set_angles(yoption(0,i), yoption(1,i), yoption(2,i));
    agents[i]->store_options();
    
    auto s2 = std::to_string(i);
    agents[i]->Show(s0 + s2);
    agents[i]->Show_options(s1);
  }
  
  delete p0;
  for(size_t i=0; i<na ; i++) delete agents[i];
  
}

//' @export
// [[Rcpp::export]]
void test_Kinematics(List para, arma::mat pos, arma::mat goal, 
                     arma::mat so, arma::mat ao, 
                     arma::vec ss, arma::vec sa, 
                     arma::vec cd, arma::vec sr) 
{
  // check what's inside 
  if (pos.n_rows != 2 || goal.n_rows != 2) 
    Rcpp::stop("Position must be on a Cartesian coordinate");
  unsigned na = pos.n_cols;  // number of agent
  
  std::vector<Parameters *> p(na);
  std::vector<Kinematics *> a(na);
  auto s0 = "Agent ";  // String to print
  auto s1 = " ";            // String to print

  for(size_t i=0; i<na; i++)
  {
    vec parameter = para[i];
    p[i] = new Parameters(parameter);
    a[i] = new Kinematics (pos.col(i), goal.col(i), so.col(i), ao.col(i), 
                           ss[i], sa[i], cd[i], sr[i], p[i]);
    
    auto s2 = std::to_string(i);
    a[i]->Show(s0 + s2);
    a[i]->Show_options(s1);
  }

  for(size_t i=0; i<na ; i++) 
  {
    delete p[i]; 
    delete a[i];
  }
}

//' @export
// [[Rcpp::export]]
void test_Parameters(Rcpp::List x)
{
  unsigned int na = x.size();
  std::vector<Parameters *> p(na); // p = parameter
  auto s0 = "Parameter ";

  for(size_t i=0; i<na; i++)
  {
    auto s1 = std::to_string(i);
    arma::vec parameter = x[i];
    p[i] = new Parameters(parameter);
    p[i]->Show(s0+s1);
  }
  
  for(size_t i=0; i<na; i++) delete p[i];
}

// void test_Pedestrian(arma::mat pos, arma::mat goal,
//                      arma::vec speed, arma::vec angle,
//                      arma::mat soption, arma::mat aoption,
//                      arma::vec collision_distrance, arma::vec steering_ratio) 
// {
//   if (pos.n_rows != 2 || goal.n_rows != 2) 
//     Rcpp::stop("Position must be on a Cartesian coordinate");
//   unsigned int na = pos.n_cols;  // number of agent
// }  


//' @export
// [[Rcpp::export]]
double test_KCollide(arma::vec pos, arma::vec goal,
                     double init_speed, double init_angle, 
                     double x, double y, double speed, double angle,
                     double obs_x, double obs_y) 
{
  if (pos.size() != 2)  Rcpp::stop("position coordinate must be of length 2");
  if (goal.size() != 2) Rcpp::stop("goal coordinate must be of length 2");
  Kinematics * k0 = new Kinematics(pos, goal, speed, angle);
  double out = k0->collide_ttc(x, y, angle, speed, obs_x, obs_y);
  delete k0;
  return out;
}


//' @export
// [[Rcpp::export]]
void test_Simulation(arma::vec ts) 
{
  Simulation * s = new Simulation(ts);
  s->Show("A Simulation instance before initialization");
  delete s;
}

//' @export
// [[Rcpp::export]]
void test_Initialize(arma::vec ts,
                      arma::mat pos, arma::mat goal, 
                      std::vector<double> speeds, 
                      std::vector<double> angles) 
{
  if (pos.n_cols != 2 || goal.n_cols != 2) stop("pos and goal must store x-y coordinate");
  if (pos.n_rows != 2 || goal.n_rows != 2) stop("pos and goal must have 2 agents");
  if (speeds.size() != 2) stop("speed must be of length 2");
  if (angles.size() != 2) stop("yaw must be of length 2");
  unsigned na = pos.n_rows;
  std::vector<Kinematics *> agents(na);
  
  std::vector<double> x = {1, .3, .3, .3};
  Parameters * p0 = new Parameters(x);
  
  
  for(size_t i=0; i<na; i++)
  {
    agents[i] = new Kinematics (pos.row(i).t(), goal.row(i).t(), speeds[i],
                                angles[i], p0);
  }
  Simulation * s = new Simulation(ts);
  s->initialize(agents);
  
  delete s;
  for(size_t i=0; i<na; i++) delete agents[i];
  delete p0;
}



//' @export
// [[Rcpp::export]]
int test_pAgent(std::string s0, std::string s1)
{
  using Rcpp::Rcout;
  using std::endl;
  // Environment base = Environment("package:base");
  // Function readline = base["readline"];
  // Function as_numeric = base["as.numeric"];
  
  return 0;
}


//' @export
// [[Rcpp::export]]
int test_List(Rcpp::List x)
{
  std::vector<std::string> s = x.attr("names");
  for_each(s.begin(), s.end(), [](std::string tmp) {Rcpp::Rcout << tmp << " ";});
  Rcpp::Rcout << std::endl;
  
  arma::vec x0 = x[0];
  arma::vec x1 = x[1];
  x0.t().print();
  x1.t().print();

  return 0;
}



//' @export
// [[Rcpp::export]]
int test_find(unsigned int a)
{
  arma::uvec beh_is_valid(a);
  beh_is_valid[0] = 0;
  beh_is_valid[1] = 1;
  beh_is_valid[2] = 1;
  beh_is_valid[3] = 0;
  beh_is_valid[4] = 1;
  beh_is_valid.print("beh");
  
    
  arma::mat A(5, 5, arma::fill::randu);
  arma::mat B(5, 5, arma::fill::randu);
  A.print("A");
  
  arma::uvec q1 = arma::find(A > B);
  arma::uvec q2 = arma::find(A > 0.5);
  arma::uvec q3 = arma::find(A > 0.5, 3, "last");

  arma::uvec q4 = arma::find(beh_is_valid);
  q4.print("q4");
  
    
  // change elements of A greater than 0.5 to 1
  A.elem( arma::find(A > 0.5) ).ones();
  A.print("changed A");
  
  A.cols(q4).print("select A by q4");

  arma::uvec q5 = arma::find(beh_is_valid == 2);
  q5.print("q5");
  unsigned int nq5 = q5.n_elem;
  Rcpp::Rcout << "How many element left? " << nq5 << std::endl;

  return 0;
}
  
//' @export
// [[Rcpp::export]]
int test_switch(unsigned int a)
{
  int day = 4;
  switch (day) {
  case 1:
    Rcpp::Rcout << "Monday";
    break;
  case 2:
    Rcpp::Rcout << "Tuesday";
    break;
  case 3:
    Rcpp::Rcout << "Wednesday";
    break;
  case 4:
    Rcpp::Rcout << "Thursday";
    break;
  case 5:
    Rcpp::Rcout << "Friday";
    break;
  case 6:
    Rcpp::Rcout << "Saturday";
    break;
  case 7:
    Rcpp::Rcout << "Sunday";
    break;
  }
  return 0;
}  

//' @export
// [[Rcpp::export]]
arma::uvec test_arma_find(double cd, arma::vec proj_signed_dist_to_CP)
{
  arma::uvec i_time_steps_entered_tmp = 
    arma::find(proj_signed_dist_to_CP < cd);
  
  // unsigned int i_time_steps_entered = i_time_steps_entered_tmp[0];
  // 
  // Rcpp::Rcout << "size " << i_time_steps_entered_tmp.size() <<  " " <<
  //   i_time_steps_entered << std::endl;
  
  return i_time_steps_entered_tmp;
}

