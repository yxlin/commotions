#include <commotions.h> 


std::vector<double> findRoots_global(double a, double b, double c) 
{
  
  std::vector<double> out(2);
  double d = b*b - 4.*a*c; 
  
  if (d > 0) {
    if (a==0) {
      out.resize(1);
      out[0] = b==0 ? NA_REAL : -c/b;
    } else {
      out[0] = (-b + sqrt(d)) / (2.*a);
      out[1] = (-b - sqrt(d)) / (2.*a);
    }
  } else if (d==0) {
    out.resize(1);
    
    if (a == 0) {
      out[0] = (b == 0) ? NA_REAL : -c/b; 
    } else {
      out[0] = -b / (2.*a); 
    }
    
  } else {
    // std::complex<double> z1 = -b / (2*a) + 1i * sqrt(abs(d));
    // std::complex<double> z2 = -b / (2*a) - 1i * sqrt(abs(d));
    out.resize(1);
    out[0] = NA_REAL;
  }
  
  return out;
}



//' Simulate Either Walkers' and Vehicles' Trajectory
//' 
//' This function uses (new) Agent Class. Not yet fully tested
//' 
//' @export
// [[Rcpp::export]]
std::vector<Rcpp::List> run(arma::vec time, Rcpp::List para, 
                            arma::mat pos, arma::mat goal, 
                            arma::mat so, arma::mat ao, 
                            std::vector<double> ss, std::vector<double> sa, 
                            arma::mat obstacles, std::vector<double> cd)
{
  if (pos.n_rows != 2 || goal.n_rows != 2) Rcpp::stop("Rows must be x-y coordinates"); 
  
  unsigned int na = pos.n_cols;
  Agent * pAgents[na];
  Parameters * pParameters[na];

  for(size_t i=0; i<na; i++)
  {
    arma::vec parameter = para[i];
    pParameters[i] = new Parameters(parameter);
    pAgents[i] = new Agent (time, pos.col(i), goal.col(i), so.col(i), 
                            ao.col(i), ss[i], sa[i], cd[i], pParameters[i]);
  }
  
  double utility, time2collision;
  arma::vec pred_acc, pred_sw_angles, magnitude;
  arma::mat pos_preds = arma::zeros<arma::mat>(2, na); // (x-y x na)
  arma::vec heading_vector(2);
  
  // Assuming all agents are on the same time stampes
  
  for (size_t i=0; i < pAgents[0]->m_nt; i++)
  {
    // Rcout << "[Time " << i << "]: " << pAgents[0]->m_times[i] << std::endl;
    
    arma::mat pos_preds = arma::zeros<arma::mat>(2, na); // (x-y x na)
    
    for (size_t j=0; j < na; j++)
    {
      if (i > 0) { pAgents[j]->advance(i-1, 1); } // Do Euler step
      heading_vector[0] = std::cos(pAgents[j]->m_angle_history(i));
      heading_vector[1] = std::sin(pAgents[j]->m_angle_history(i));
      pos_preds.col(j) = pAgents[j]->m_pos_history.col(i) + 
        pAgents[j]->m_speed_history[i] * pAgents[j]->m_pt * heading_vector;
      // pos_preds.t().print("pos_preds");
    } // end of 1st agent loop
    
    for (size_t j=0; j < na; j++)
    {
      double best_value = R_NegInf, chosen_speed = 0, chosen_angle = 0;
      arma::vec look_ahead = arma::zeros<arma::vec>(4);
      
      for (size_t k=0; k < pAgents[j]->m_options.n_rows; k++)
      {
        magnitude = pAgents[j]->m_options.row(k).t();
        pred_acc       = pAgents[j]->m_accs;
        pred_sw_angles = pAgents[j]->m_sw_angles;
        
        // Pedestrian
        if (magnitude[0]) { pAgents[j]->add_action(pred_acc, magnitude[0], i); }
        if (magnitude[1]) { pAgents[j]->add_action(pred_sw_angles, magnitude[1], i); }
        
        pAgents[j]->advance(i, pred_acc, pred_sw_angles, look_ahead);
        
        if (na == 1) {
          utility = pAgents[j]->value_1Agent(look_ahead, obstacles);
        } else {
          utility = pAgents[j]->value_2Agent(look_ahead);
          
          // Collision cost obstacles
          for (size_t l = 0; l < obstacles.n_cols; l++)
          {
            if ( obstacles.col(l).has_nan() ) continue;
            utility += pAgents[j]->collide_2Agent(look_ahead, obstacles.col(l));
          }
          
          // Collision cost other agents
          for (size_t m = 0; m < na; m++)
          {
            if (m == j) continue;
            arma::vec other_pos = pos_preds.col(m); // From the 1st agent loop
            double other_speed = pAgents[m]->m_speed_history(i);
            double other_angle = pAgents[m]->m_angle_history(i);
            
            arma::vec delta = other_pos - look_ahead.subvec(0, 1);
            double delta_v_x = other_speed * std::cos(other_angle) -
              look_ahead[2] * std::cos(look_ahead[3]);
            double delta_v_y = other_speed*std::sin(other_angle) -
              look_ahead[2] * std::sin(look_ahead[3]);
            
            double a = delta_v_x*delta_v_x + delta_v_y*delta_v_y;
            double b = 2.0 * (delta[0] * delta_v_x + delta[1] * delta_v_y);
            // current agent's tolerable distance
            double c = delta[0]*delta[0] + delta[1]*delta[1] - 
              pAgents[j]->m_cd * pAgents[j]->m_cd;  
            std::vector<double> coll_times = findRoots_global(a, b, c);
            
            if (coll_times.size() == 1) {
              
              time2collision = (std::isnan(coll_times[0])) ? R_PosInf : coll_times[0]; 
              
            } else {
              
              if ( std::copysign(1, coll_times[0]) == std::copysign(1, coll_times[1]) )
              {
                time2collision = (coll_times[0] < 0) ? R_PosInf : 
                std::min(coll_times[0], coll_times[1]);
              } 
              else // one in future one in past - i.e., collision now
              {
                time2collision = 0;
              }
              
            }
            
            if (time2collision == 0)
            {
              utility = R_NegInf;
            } else if (time2collision < R_PosInf)
            {
              utility += -pAgents[j]->m_p->kc / time2collision;  // kc == ko
            }
            else 
            {
              // Do nothing; keep utility as it is;
            }
          }
          
        }  // enf of selecting 1Agent or 2Agent ifelse
        
        
        utility -= pAgents[j]->m_p->ke * magnitude[1]*magnitude[1];
        // Rcpp::Rcout << utility << "\n"; 
        if (utility > best_value) {
          best_value   = utility;
          chosen_speed = magnitude[0];
          chosen_angle = magnitude[1];
        }
        
      } // end of option loop
      
      // Rcout << chosen_speed << " " << chosen_angle <<  "\n";
      if (chosen_speed != 0) {
        // Pedestrian
        arma::vec tmp = pAgents[j]->m_accs;
        pAgents[j]->add_action(tmp, chosen_speed, i);
        pAgents[j]->m_accs = tmp;
      }
      
      if (chosen_angle != 0) {
        // Pedestrian
        arma::vec tmp = pAgents[j]->m_sw_angles;
        pAgents[j]->add_action(tmp, chosen_angle, i);
        pAgents[j]->m_sw_angles = tmp;
      }
      
    } // end of 2nd agent loop
  } // end of time stamp loop
  
  
  std::vector<Rcpp::List> out(na);
  for(size_t j=0; j<na; j++) 
  {
    out[j] = Rcpp::List::create(
      Rcpp::Named("pos")    = pAgents[j]->m_pos_history,
      Rcpp::Named("speeds") = pAgents[j]->m_speed_history,
      Rcpp::Named("angles") = pAgents[j]->m_angle_history,
      Rcpp::Named("accelerations") = pAgents[j]->m_accs,
      Rcpp::Named("sw_angles")     = pAgents[j]->m_sw_angles,     
      Rcpp::Named("yaw_rate_history") = pAgents[j]->m_yr_history,
      Rcpp::Named("times")  = pAgents[j]->m_times,
      Rcpp::Named("goal")   = pAgents[j]->m_goal,
      Rcpp::Named("obstacles")    = obstacles,
      Rcpp::Named("collision_d")  = cd, 
      Rcpp::Named("Parameters")   = para);
    
    delete pAgents[j]; 
    delete pParameters[j];
  }
  return out;
}  

//' Simulate P2P Trajectory
//' 
//' This function uses Simulation and Kinematics Classes
//' 
//' @export
// [[Rcpp::export]]
Rcpp::List test_P2P(std::vector<double> time, 
                    Rcpp::List para, 
                    arma::mat pos, arma::mat goal, 
                    arma::mat so, arma::mat ao,
                    std::vector<double> ss, std::vector<double> sa,
                    arma::mat obstacles, 
                    std::vector<double> cd,
                    std::vector<double> sr,
                    std::vector<double> wall) 
{
  // cd = collision_distance; ss = start_speed; sa = start_angle
  // so = speed options; ao = angle options; sr = steering_ratio
  if(pos.n_rows != 2 || goal.n_rows != 2) 
  {
    Rcpp::stop("pos and goal must be on x-y coordinates");
  }
  unsigned int na   = pos.n_cols;
  unsigned int nobs = obstacles.n_cols;
  Simulation * s    = new Simulation(time);
  std::vector<Parameters *> p(na);
  std::vector<Kinematics *> a(na);
  
  for(size_t i=0; i<na; i++)
  {
    arma::vec parameter = para[i];
    p[i] = new Parameters(parameter);
    a[i] = new Kinematics (pos.col(i), goal.col(i), so.col(i), ao.col(i), ss[i],
                           sa[i], cd[i], sr[i], p[i]);
  }
  
  s->initialize(a);
  
  bool is_view_blocked;
  double utility, time2collision;
  arma::mat ttc_history = arma::zeros<arma::mat>(s->nt, nobs);
  arma::vec pred_acc, pred_sw_angles, mag, look_ahead(4);
  
  for (size_t i=0; i < s->nt; i++)
  {
    // pos_preds is not used in 1 agent case, bcz it is used to calcualted
    // collision cost onto the other agents.
    arma::mat pos_preds = arma::zeros<arma::mat>(2, na); // (x-y x na)
    // Rcpp::Rcout << "[Time " << i << "]: " << s->m_times[i] << std::endl;
    for (size_t j=0; j < na; j++)     // First agent loop
    {
      // --------------------------------------------------------------------
      // starting from 2nd time step; an agent start moving forward / backward.
      // Cannot be placed at the end?!
      if (i > 0) s->advance_Pedestrian(j, i-1, 1);  
      s->get_pos_predictions(j, i, pos_preds);
      
      if (!obstacles.has_nan())
      {
        for (size_t k=0; k < nobs; k++)
        {
          ttc_history(i, k) = a[j]->collide_ttc(s->m_xs(j, i), s->m_ys(j, i),
            s->m_angles(j, i), s->m_speeds(j, i), obstacles(0, k), 
            obstacles(1, k));
        }
      }
    }  // end of 1st agent loop
    
    for (size_t j=0; j < na; j++)     // Second agent loop
    {
      // Rcpp::Rcout << "Agent " << j << ": \n";
      double best_value = R_NegInf, chosen_speed = 0, chosen_angle = 0;
      // Going through all possible actions
      for (size_t k=0; k < a[j]->m_actions.n_rows; k++)
      {
        mag            = a[j]->m_actions.row(k).t();  // magnitude
        pred_acc       = s->m_accs.row(j).t();        // npt + npt
        pred_sw_angles = s->m_sw_angles.row(j).t();   // yaw rate?
        
        if (mag[0]) { s->add_action_Pedestrian(pred_acc, mag[0], i); } 
        if (mag[1]) { s->add_action_Pedestrian(pred_sw_angles, mag[1], i); } 
        
        s->advance_Pedestrian(j, i, pred_acc, pred_sw_angles, look_ahead);
        utility = a[j]->value_Pedestrian(look_ahead, obstacles);
        
        if (na != 1) 
        {
          is_view_blocked = s->is_cross(i, wall);
          if (!is_view_blocked)
          {
            for (size_t m=0; m < na; m++)
            {
              if (m == j) continue;
              time2collision = a[j]->collide_otherA(look_ahead,
                                             pos_preds.col(m),
                                             s->m_speeds(m,i),
                                             s->m_angles(m,i));
              if (time2collision < R_PosInf) {
                utility += -p[j]->kc / time2collision;
              } else if (time2collision==0) {
                utility = R_NegInf;
              } else {
              }
            }
          }
        }

        utility -= p[j]->ke * mag[1]*mag[1]; // ke == old Ct
        // Rcpp::Rcout << utility << "\n";
        if (utility > best_value) {
            best_value   = utility;
            chosen_speed = mag[0];
            chosen_angle = mag[1];
        }
      }  // end of option  actions
      
      if (chosen_speed != 0) {
        arma::vec tmp = s->m_accs.row(j).t();
        s->add_action_Pedestrian(tmp, chosen_speed, i);
        s->m_accs.row(j) = tmp.t();
      }
      
      if (chosen_angle != 0) {
        arma::vec tmp = s->m_sw_angles.row(j).t();
        s->add_action_Pedestrian(tmp, chosen_angle, i);
        s->m_sw_angles.row(j) = tmp.t();  // na x nat
      }

    } // end of 2nd agent loop
    
  }   // end of time loop
  Rcpp::List out = Rcpp::List::create(
    Rcpp::Named("x")      = s->m_xs,
    Rcpp::Named("y")      = s->m_ys,
    Rcpp::Named("speeds") = s->m_speeds,
    Rcpp::Named("angles") = s->m_angles,
    Rcpp::Named("accelerations")    = s->m_accs,
    Rcpp::Named("sw_angles")        = s->m_sw_angles,
    Rcpp::Named("yaw_rate_history") = s->m_yaw_rates,
    Rcpp::Named("times")  = s->m_times,
    Rcpp::Named("goal")   = goal,
    Rcpp::Named("obstacles")   = obstacles,
    Rcpp::Named("ttc_history") = ttc_history,
    Rcpp::Named("collision_distance") = cd,
    Rcpp::Named("Parameters") = para);
  
  for(size_t i=0; i<na; i++) { delete a[i]; delete p[i]; }
  delete s;
  return out;
}

//' Simulate V2V Trajectory 
//' 
//' This function uses Simulation and Kinematics Classes; one or two agents
//' 
//' @export
// [[Rcpp::export]]
Rcpp::List test_V2V(std::vector<double> time, 
                    Rcpp::List para, 
                    arma::mat pos, arma::mat goal, 
                    arma::mat so, arma::mat ao,
                    std::vector<double> ss, std::vector<double> sa,
                    arma::mat obstacles,
                    std::vector<double> cd, std::vector<double> sr)
{
  if(pos.n_rows != 2 || goal.n_rows != 2) Rcpp::stop("pos and goal must store x-y coordinate");
  unsigned int na   = pos.n_cols;
  unsigned int nobs = obstacles.n_cols;
  Simulation * s    = new Simulation(time);
  
  std::vector<Parameters *> p(na);
  std::vector<Kinematics *> a(na);

  for(size_t i=0; i<na; i++)
  {
    arma::vec parameter = para[i];
    p[i] = new Parameters (parameter);
    a[i] = new Kinematics (pos.col(i), goal.col(i), so.col(i), ao.col(i),
                           ss[i], sa[i], cd[i], sr[i], p[i]);
  }

  s->initialize(a);
  
  double utility, time2collision;
  arma::mat ttc_history = arma::zeros<arma::mat>(s->nt, nobs);
  arma::vec pred_acc, pred_sw_angles, mag, look_ahead(6);
        
  for (size_t i=0; i < s->nt; i++)
  {
    // Rcpp::Rcout << "[Time " << i << "]: " << s->m_times[i] << std::endl;
    
    arma::mat pos_preds = arma::zeros<arma::mat>(2, na); // (x-y x na)

    for (size_t j=0; j < na; j++)     // First agent loop
    {
      if (i > 0) s->advance_Vehicle(j, i-1, 1);
      s->get_pos_predictions(j, i, pos_preds); // difference
      
      if (!obstacles.has_nan())
      {
         for (size_t k=0; k < nobs; k++)
         {
            ttc_history(i, k) = a[j]->collide_ttc(s->m_xs(j, i),
                                               s->m_ys(j, i),
                                               s->m_angles(j, i),
                                               s->m_speeds(j, i),
                                               obstacles(0, k),
                                               obstacles(1, k));
         }
      }
    }

    for (size_t j=0; j < na; j++)     // Second agent loop
    {
      // Rcpp::Rcout << "Agent " << j << ": \n";
      
        double best_value = R_NegInf, chosen_speed = 0, chosen_angle = 0;
      
        for (size_t k=0; k < a[j]->m_actions.n_rows; k++)
        {
          mag            = a[j]->m_actions.row(k).t();
          pred_acc       = s->m_accs.row(j).t();
          pred_sw_angles = s->m_sw_angles.row(j).t();
          
          if (mag[0]) { 
            s->add_action_Vehicle(pred_acc, mag[0], i); 
            // if (s->m_times[i] > 3) pred_acc.t().print();
          } 
          if (mag[1]) { 
            s->add_action_Vehicle(pred_sw_angles, mag[1], i); 
            // if (s->m_times[i] >= 3.1 && s->m_times[i] < 4) pred_sw_angles.t().print();
          } 
          
          s->advance_Vehicle(j, pred_acc, pred_sw_angles, i, look_ahead);
          // if (s->m_times[i] >= 3.1 && s->m_times[i] < 4) look_ahead.t().print();

          // Original test_two
          utility = a[j]->value_Vehicle(look_ahead, obstacles, na);

          if (na > 1)
          {
            for (size_t l=0; l < na; l++)
            {
              if (l == j) continue;
              time2collision = a[j]->collide_otherA(look_ahead,
                                             pos_preds.col(l),
                                             s->m_speeds(l,i),
                                             s->m_angles(l,i));
              if (time2collision < R_PosInf) {
                // ksc = kc; speed
                utility += -p[j]->ksc * (look_ahead[2] / (2.0*time2collision)) *
                  (look_ahead[2] / (2.0*time2collision));
              } else if (time2collision==0) {
                utility = R_NegInf;
              } else {
              }
            }
          }
          

          // Is mag[0] the adjustment of acceleration; ke = Cp
          // mag[1] is the adjustment of steering angle
          utility += -p[j]->ke * mag[0]*mag[0] - p[j]->Ct * mag[1]*mag[1];

          if (utility > best_value) {
                best_value   = utility;
                chosen_speed = mag[0];  // chosen_acceleration as for a vehicle
                chosen_angle = mag[1];
          }
        }   // end of option loop

        if (chosen_speed) {
          arma::vec tmp = s->m_accs.row(j).t();
          s->add_action_Vehicle(tmp, chosen_speed, i);
          s->m_accs.row(j) = tmp.t();
        }

        if (chosen_angle) {
           arma::vec tmp = s->m_sw_angles.row(j).t();
           s->add_action_Vehicle(tmp, chosen_angle, i);
           s->m_sw_angles.row(j) = tmp.t();
        }
    }   // end of 2nd agent loop
  }     // end of time loop


  Rcpp::List out = Rcpp::List::create(
    Rcpp::Named("x")      = s->m_xs,
    Rcpp::Named("y")      = s->m_ys,
    Rcpp::Named("speeds") = s->m_speeds,
    Rcpp::Named("angles") = s->m_angles,
    Rcpp::Named("accelerations")    = s->m_accs,
    Rcpp::Named("sw_angles")        = s->m_sw_angles,
    Rcpp::Named("yaw_rate_history") = s->m_yaw_rates,
    Rcpp::Named("times")  = s->m_times,
    Rcpp::Named("goal")   = goal,
    Rcpp::Named("obstacles")   = obstacles,
    Rcpp::Named("ttc_history") = ttc_history,
    Rcpp::Named("collision_distance") = cd,
    Rcpp::Named("Parameters") = para);
  
    
  for (size_t i=0; i<na; i++) { delete a[i]; delete p[i]; }
  delete s;
  return out;
}

//' Simulate P2V Trajectory 
//' 
//' This function uses Simulation and Kinematics Classes
//' 
//' @export
// [[Rcpp::export]]
Rcpp::List test_P2V(std::vector<double> time, 
              Rcpp::List para, 
              arma::mat pos, arma::mat goal, 
              arma::mat so, arma::mat ao,
              std::vector<double> ss, std::vector<double> sa,
              arma::mat obstacles,
              std::vector<double> cd, std::vector<double> sr)
{
  if(pos.n_rows != 2 || goal.n_rows != 2) 
  {
    Rcpp::stop("pos and goal must store x-y coordinate");
  }
  unsigned int na   = pos.n_cols;
  unsigned int nobs = obstacles.n_cols;
  Simulation * s = new Simulation(time);
  
  std::vector<Parameters *> p(na); // p = parameter
  std::vector<Kinematics *> a(na); // a = agent

  for(size_t i=0; i<na; i++)
  {
    arma::vec parameter = para[i];
    p[i] = new Parameters(parameter);
    a[i] = new Kinematics (pos.col(i), goal.col(i), so.col(i), ao.col(i),
                           ss[i], sa[i], cd[i], sr[i], p[i]);
  }
  
  s->initialize(a);
  
  double utility, time2collision;
  arma::mat ttc_history = arma::zeros<arma::mat>(s->nt, nobs);
  arma::vec pred_acc, pred_sw_angles, mag;
  
  for (size_t i=0; i < s->nt; i++)
  {
    arma::mat pos_preds = arma::zeros<arma::mat>(2, na); // (x-y x na)
    // Rcpp::Rcout << "[Time " << i << "]: " << s->m_times[i] << std::endl;
  
    for (size_t j=0; j < na; j++)     // First agent loop
    {
      // Rcpp::Rcout << "Agent " << j << ": \n";
      
      if (i > 0) s->advance_Vehicle(j, i-1, 1);
      // if (i >= 1) {
      //    Rcpp::Rcout << "[" << s->m_xs(j,i) <<  " " << s->m_ys(j,i) << "]" <<
      //    " " << s->m_speeds(j,i) <<  " " << s->m_angles(j,i) << "\n";
      // }
      
      s->get_pos_predictions(j, i, pos_preds);
      // pos_preds.print("pos_preds");
      if (!obstacles.has_nan())
      {
        for (size_t k=0; k < nobs; k++)
        {
          ttc_history(i, k) = a[j]->collide_ttc(s->m_xs(j, i),
                      s->m_ys(j, i),
                      s->m_angles(j, i),
                      s->m_speeds(j, i),
                      obstacles(0, k),
                      obstacles(1, k));
        }
      }
    }
    
    for (size_t j=0; j < na; j++)     // Second agent loop
    {
      // Rcpp::Rcout << "Agent " << j << std::endl;
      /* -----------------------------------------------------
       Add something different in P2V (mix) that can cause 
       numerical difference; acceleration cannot be less than 0?
       ----------------------------------------------------- */
      if ( s->m_speeds(j, i) == 0 )
      {
        unsigned int end = s->nat - 1;
        s->m_accs.row(j).cols(i, end).for_each( 
            [](arma::rowvec::elem_type& val)
        { val = std::fmax(0.0, val); });
      }

      double best_value = R_NegInf, chosen_speed = 0, chosen_angle = 0;
     
      for (size_t k=0; k < a[j]->m_actions.n_rows; k++)
      {
        mag            = a[j]->m_actions.row(k).t();
        pred_acc       = s->m_accs.row(j).t();
        pred_sw_angles = s->m_sw_angles.row(j).t();
        
        if (mag[0]) 
        { 
          if (a[j]->m_p->type == 0) 
          {
            s->add_action_Pedestrian(pred_acc, mag[0], i); 
          } else if (a[j]->m_p->type == 1)
          {
            s->add_action_Vehicle(pred_acc, mag[0], i); 
          } else
          {
            Rcpp::stop("Unknown add_action type");
          }
          // pred_acc.t().print("pred_acc");
        }
        if (mag[1]) 
        { 
          if (a[j]->m_p->type == 0) 
          {
            s->add_action_Pedestrian(pred_sw_angles, mag[1], i); 
          } else if (a[j]->m_p->type == 1)
          {
            s->add_action_Vehicle(pred_sw_angles, mag[1], i); 
            // pred_sw_angles.t().print("pred_sw_angles");
            
          } else
          {
            Rcpp::stop("Unknown add_action type");
          }
          
        }
        
        arma::vec look_ahead = arma::zeros<arma::vec>(6);
        // apply to both?
        s->advance_Vehicle(j, pred_acc, pred_sw_angles, i, look_ahead);
        // look_ahead.t().print("look_ahead");
        
        // Cost of colliding onto other agents and obstacles
        if(a[j]->m_p->type == 0)
        {
          utility = a[j]->value_Pedestrian(look_ahead, obstacles);
          
          for (size_t l=0; l < na; l++)
          {
            if (l == j) continue;
            time2collision = a[j]->collide_otherA(look_ahead,
                                           pos_preds.col(l),
                                           s->m_speeds(l,i),
                                           s->m_angles(l,i));
            if (time2collision < R_PosInf) {
              utility += -a[j]->m_p->kc / time2collision;
            } else if (time2collision==0) {
              utility = R_NegInf;
            } else {
            }
          }

        } 
        
        else if (a[j]->m_p->type == 1) {
          utility = a[j]->value_Vehicle(look_ahead, obstacles, na);
          
          for (size_t l=0; l < na; l++)
          {
            if (l == j) continue;
            time2collision = a[j]->collide_otherA(look_ahead,
                                           pos_preds.col(l),
                                           s->m_speeds(l,i),
                                           s->m_angles(l,i));
            if (time2collision < R_PosInf) {
              utility += -a[j]->m_p->ksc * 
                (look_ahead[2] / (2.0*time2collision)) *
                (look_ahead[2] / (2.0*time2collision));
            } else if (time2collision==0) {
              utility = R_NegInf;
            } else {
            }
          }
          
          
        } else 
        {
          Rcpp::stop("Unknown add_action type (value function)");
        }

        utility += -a[j]->m_p->ke * mag[0]*mag[0];

        if (utility > best_value) {
          best_value   = utility;
          chosen_speed = mag[0];  // chosen_acceleration as for a vehicle
          chosen_angle = mag[1];
        }
      }   // end of option loop
      
      if (chosen_speed) {
        arma::vec tmp = s->m_accs.row(j).t();
        
        if(a[j]->m_p->type == 0)
        {
          s->add_action_Pedestrian(tmp, chosen_speed, i);
          
        } else if (a[j]->m_p->type == 1)
        {
          s->add_action_Vehicle(tmp, chosen_speed, i);
        } else
        {
          Rcpp::stop("Unknown add_action type (value function)");
        }
        
        s->m_accs.row(j) = tmp.t();
      }
      
      if (chosen_angle) {
        arma::vec tmp = s->m_sw_angles.row(j).t();
        
        if(a[j]->m_p->type == 0)
        {
          s->add_action_Pedestrian(tmp, chosen_angle, i);
          
        } else if (a[j]->m_p->type == 1)
        {
          s->add_action_Vehicle(tmp, chosen_angle, i);
        } else
        {
          Rcpp::stop("Unknown add_action type (value function)");
        }
        s->m_sw_angles.row(j) = tmp.t();
      }
    }   // end of 2nd agent loop
    
  }     // end of time loop
  
  Rcpp::List out = Rcpp::List::create(
    Rcpp::Named("x")      = s->m_xs,
    Rcpp::Named("y")      = s->m_ys,
    Rcpp::Named("speeds") = s->m_speeds,
    Rcpp::Named("angles") = s->m_angles,
    Rcpp::Named("accelerations")    = s->m_accs,
    Rcpp::Named("sw_angles")        = s->m_sw_angles,
    Rcpp::Named("yaw_rate_history") = s->m_yaw_rates,
    Rcpp::Named("times")  = s->m_times,
    Rcpp::Named("goal")   = goal,
    Rcpp::Named("obstacles")   = obstacles,
    Rcpp::Named("ttc_history") = ttc_history,
    Rcpp::Named("collision_distance") = cd,
    Rcpp::Named("Parameters") = para);
  
  
  for (size_t i=0; i<na; i++)  { delete a[i]; delete p[i]; } 
  delete s;
  return out;
}


// int test_add_action_to_acc_array(arma::vec time, 
//                                  Rcpp::List para, 
//                                  arma::mat pos, arma::mat goal, 
//                                  arma::mat so, arma::mat ao,
//                                  std::vector<double> ss, std::vector<double> sa,
//                                  std::vector<double> cd, std::vector<double> sr,
//                                  arma::vec conflict_point)
// {
//   if (pos.n_rows != 2 || goal.n_rows != 2) 
//     Rcpp::stop("pos and goal must store x-y coordinate");
//   unsigned int na   = pos.n_cols;
//   if (na != 2) Rcpp::stop("Must be 2 agents");
//   
//   SCSimulation * s = new SCSimulation(time);
//   std::vector<Parameters *> p(na); // p = parameter
//   std::vector<SCAgent *> a(na); // a = agent
//   
//   for(size_t i=0; i<na; i++)
//   {
//     arma::vec parameter = para[i];
//     p[i] = new Parameters(parameter);
//     // a[i] = new SCAgent (pos.col(i), goal.col(i), so.col(i), ao.col(i),
//     //                     ss[i], sa[i], cd[i], sr[i], p[i]);
//   }
//   
//   s->initialize(a);
//   
//   for(size_t i=0; i<na; i++)
//   {
//     add_action_to_acc_array(s, a[i], 0, 0, i);
//     add_action_to_acc_array(s, a[i], 0, 1, i);
//     add_action_to_acc_array(s, a[i], 0, 2, i);
//   }
//   
//   return 0;
// }

//' @export
// [[Rcpp::export]]
Rcpp::List test_SC_P2V_tmp(arma::vec time, Rcpp::List para, 
                    arma::mat initial_position, arma::mat destination, 
                    arma::mat action0, arma::mat action1, 
                    arma::vec initial_speed, arma::vec initial_angle,
                    arma::vec collision_distance, arma::vec conflict_point,
                    arma::uvec optional_assumptions,
                    unsigned int debug)
{
  if (initial_position.n_rows != 2 || destination.n_rows != 2) Rcpp::stop("pos and goal must store x-y coordinates");
  if (initial_position.n_cols != 2 || destination.n_cols != 2) Rcpp::stop("Input must be 2 agents");
  unsigned int na = initial_position.n_cols;
  std::vector<Parameters *> p(na); // p = parameter
  std::vector<SCAgent *> a(na);    // a = agent

  for(size_t i=0; i<na; i++)
  {
    arma::vec parameter = para[i];
    p[i] = new Parameters(parameter, optional_assumptions);
    a[i] = new SCAgent (time, destination.col(i), initial_position.col(i), 
                        initial_speed[i], initial_angle[i], action0.col(i), 
                        action1.col(i), collision_distance[i], p[i], optional_assumptions);
  }
  
  /////////////////////////////////////////////////////////////////////////////
  SCAgent * other_agent;
  unsigned int i_other_agent;
  
  for (size_t i = 0; i < a[0]->m_nt; i++)
  {
    if (i > 0)
    {
      for (size_t j=0; j < na; j++) { a[j]->update_kinematics(i, debug); }
    }

    for (size_t j=0; j < na; j++)  
    {
      i_other_agent = (j==0) ? 1 : 0;
      other_agent = a[i_other_agent];
      // Rcpp::Rcout << j << ": " << "\t";
      a[j]->update_action(i, conflict_point, other_agent, optional_assumptions, debug); 
    }

    // Rcpp::Rcout << std::endl;
  }
  
  arma::cube est_action_surplus_vals(a[0]->m_naction, a[0]->m_nt, na);
  arma::cube sensory_probs_given_behs(N_BEHAVIORS, a[0]->m_nt, na); // N_BEHAVIORS in Parameters.hpp
  arma::cube beh_probs(N_BEHAVIORS, a[0]->m_nt, na); 
  arma::cube beh_acceleration(N_BEHAVIORS, a[0]->m_nt, na); 
  
  arma::field<arma::cube> action_vals_given_behs(na);
 

  arma::mat xs(a[0]->m_nt, na), ys(a[0]->m_nt, na);
  arma::mat acceleration(a[0]->m_nat, na);
  arma::mat speed(a[0]->m_nt, na), yawangle(a[0]->m_nt, na);

  for (size_t j=0; j < na; j++)  
  {
    xs.col(j) = a[j]->m_trajectory->m_position.row(0).t();
    ys.col(j) = a[j]->m_trajectory->m_position.row(1).t();
    est_action_surplus_vals.slice(j) = a[j]->est_action_surplus_vals;
    sensory_probs_given_behs.slice(j) = a[j]->sensory_probs_given_behs;
    beh_probs.slice(j) = a[j]->beh_probs;
    beh_acceleration.slice(j) = a[j]->beh_acceleration;

    action_vals_given_behs[j] = a[j]->action_vals_given_behs;

    acceleration.col(j) = a[j]->m_trajectory->m_acceleration;
    speed.col(j) = a[j]->m_trajectory->m_speed;
    yawangle.col(j) = a[j]->m_trajectory->m_yawangle;

  }


  Rcpp::List out = Rcpp::List::create(
    Rcpp::Named("x") = xs,
    Rcpp::Named("y") = ys,
    Rcpp::Named("speed") = speed,
    Rcpp::Named("yawangle") = yawangle,
    Rcpp::Named("acceleration") = acceleration,
  //   Rcpp::Named("sw_angles")        = s->m_sw_angles,
  //   Rcpp::Named("yaw_rate_history") = s->m_yaw_rates,
    Rcpp::Named("times")  = a[0]->m_times,
    Rcpp::Named("goal")   = destination,
  //   Rcpp::Named("collision_distance") = cd,
    Rcpp::Named("Parameters") = para,
    Rcpp::Named("est_action_surplus_vals") = est_action_surplus_vals,
    Rcpp::Named("sensory_probs_given_behs") = sensory_probs_given_behs,
    Rcpp::Named("beh_probs") = beh_probs,
    Rcpp::Named("action_vals_given_behs") = action_vals_given_behs,
    Rcpp::Named("beh_acceleration") = beh_acceleration
    );
  
  
  for (size_t i=0; i<na; i++)  { delete a[i]; delete p[i]; } 
  return out;
}

