#include <commotions.h>

SCSimulation::SCSimulation(unsigned int i_time_step, arma::vec times) 
{
  m_times = times;
  set_time_step(i_time_step);
} 

void SCSimulation::set_time_step(unsigned int i_time_step) 
{
  m_i_time_step = i_time_step;
  m_time = m_times[m_i_time_step];
}

