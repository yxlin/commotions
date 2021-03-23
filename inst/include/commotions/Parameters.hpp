/* Parameters.hpp - A class to contain the model parameters
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA.
 */

#ifndef PARAMETERS_H
#define PARAMETERS_H

#define gamma 0.5 // = DEFAULT_PARAMS.alpha
#define kappa 0.3
#define Lambda 10
#define sigma_ao 0.01 // #.05
#define DeltaT 0.5  // T_P is the prediction time

#define d_C 3         // collision distance
#define TTC_FOR_COLLISION 0.1 // ???
#define MIN_BEH_PROB 0.0 // behaviour probabilities below this value are regarded???
#define i_NO_ACTION 2
#define N_BEHAVIORS 3
#define i_CONSTANT 0
#define i_PROCEEDING 1
#define i_YIELDING 2

using arma::vec;
using arma::uvec;

class Parameters
{
public:
  unsigned int type;
  
  double kg;    // -kg controls the travelling speed (vg) towards goal
  double kc;    // -kc controls reverse of TIC (1/tau_i) with obstacle i (i == 1 or more) 
  double kdv;  // Relabel Cv to kdv; controls the square of the longitudinal speed
  double ke;   // changed from Ct; -Ct controls the square of the change in yaw 
  // rotation effectuated for the evaluated action
  
  double ksg;   // Cg = ksg
  double Ct;
  double kda;   // Ca = kda
  double Comega;
  double ksc;

  // optional assumptions
  double alpha, beta_O, beta_V, sigma_V, DeltaV_th;  
  
  Parameters(vec params);
  Parameters(vec params, uvec optional_assumptions);
  ~Parameters();

  void Show(std::string str) const;
};

#endif // PARAMETERS_H
