#ifndef SCSIMULATION_H
#define SCSIMULATION_H

class SCSimulation
{
public:
  SCAgent* m_self_agent;
  SCAgent* m_other_agent;
  unsigned int m_i_time_step;
  double m_time; 
  arma::vec m_times;

  SCSimulation(){}
  SCSimulation(unsigned int i_time_step, arma::vec times); 
  ~SCSimulation() { }  // destructor
  void set_time_step(unsigned int i_time_step);
};

#endif // SCSIMULATION_H
