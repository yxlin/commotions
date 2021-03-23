#include <commotions.h> 

using arma::vec;
using arma::uvec;

Parameters::Parameters(vec params)
{
  type = params[0];
  
  if (type== 0) {
    kg  = params[1];   // 1 = kg
    kc  = params[2];   // 2 = kc = ko
    kdv = params[3];   // 3 = Cv = kdv
    ke  = params[4];   // 4 = ke for yaw rate? old Ct? Cp?
    
  } else if (type == 1) {
    kg  = params[1];   // 1 = kg
    kc  = params[2];   // 2 = kc; collision for obstacles and others? control type speed
    kdv = params[3];   // 3 = kdv 
    ke  = params[4];   // 4 = ke == Cp? car and pedestrian are different on this parameter
    
    ksg  = params[5];   // 5; from Cg; how car stops at the target / goal  
    Ct   = params[6];   // 6 = new Ct  for steering angle 
    kda  = params[7];   // 7; from Ca; the longitudinal acceleration discomfort
    Comega = params[8]; // 8; the lateral acceleration discomfort term
    ksc  = params[9];   // 9; for collision of other agents vehicle / control type acc
    
  } else {
    Rcpp::stop("Undefined parameters");
  }
}

Parameters::Parameters(vec params, uvec optional_assumptions)
{
  type = params[0];
  alpha = 0.5;
  DeltaV_th = 0.11;
  sigma_V = 0.1;
  beta_O  = 1;
  beta_V  = 0.5;

  if (!optional_assumptions[0]) { alpha = 0;     DeltaV_th = 0; }// oEA is FALSE
  if (!optional_assumptions[1]) sigma_V = 0; // oAN is FALSE
  if (!optional_assumptions[2]) 
  {
    // Rcpp::Rcout << "beta_O set to 0\n";
    beta_O = 0;  // oBEao is FALSE
  }
  if (!optional_assumptions[3]) 
  {
    // Rcpp::Rcout << "beta_V set to 0\n";
    beta_V = 0;  // oBEvs is FALSE
  }

  if (type== 0) {
    kg  = params[1];   // 1 = kg
    kc  = params[2];   // 2 = kc = ko
    kdv = params[3];   // 3 = Cv = kdv
    ke  = params[4];   // 4 = ke for yaw rate? old Ct? Cp?
  } else if (type == 1) {
    kg  = params[1];  // 1 = kg
    kc  = params[2];  // 2 = kc; collision for obstacles and others? control type speed
    kdv = params[3];  // 3 = kdv 
    ke  = params[4];  // 4 = ke == Cp? car and pedestrian are different on this parameter
    
    ksg  = params[5];   // 5; from Cg; how car stops at the target / goal  
    Ct   = params[6];   // 6 = new Ct  for steering angle 
    kda  = params[7];   // 7; from Ca; the longitudinal acceleration discomfort
    Comega = params[8]; // 8; the lateral acceleration discomfort term
    ksc  = params[9];   // 9; for collision of other agents vehicle / control type acc
    
  } else {
    Rcpp::stop("Undefined parameters");
  }
}

// destructor
Parameters::~Parameters() { }
void Parameters::Show(std::string str) const
{
    using namespace Rcpp;
    if (type == 0) {
      
      Rcout << str << ":\n";
      Rcout << "[kg\tkc\tkdv\tke] = " << "[" << kg << "\t" << kc << "\t" << kdv
            << "\t" << ke << "]" << std::endl;
    } else if (type == 1) {
      
      Rcout << str << ":\n";
      Rcout << "[kg\tkc\tkdv\tke] = " << "[" << kg << "\t" << kc << "\t" << 
        kdv << "\t" << ke << "]\n";
      Rcout << "[ksg\tCt\tkda\tComega\tksc] = " << "[" << ksg << "\t" << Ct << 
        "\t" << kda << "\t" << Comega << "\t" << ksc << "]" << std::endl;
      
    } else {
      Rcpp::stop("Undefined parameters");
    }
    
 }
  
