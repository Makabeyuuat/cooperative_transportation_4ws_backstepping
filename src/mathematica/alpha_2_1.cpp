#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_alpha_2_1_()
{
double ret;
ret = -(sr.0Cs*(1 - sr.0Cs*sr.0d)*Power(Sec(Thetap0),2)) - sr.0Cs*(1 - sr.0Cs*sr.0d)*Power(Tan(Thetap0),2) - sr.0d*Tan(Thetap0)*sr.0Cs1;
return ret;
}
