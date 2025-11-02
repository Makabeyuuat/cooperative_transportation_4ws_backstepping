#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_d_SX_d_t_1_1_()
{
double ret;
ret = -(((u2 - (u1*sr.0Cs*Cos(Thetap0))/(1 - sr.0Cs*sr.0d))*Sin(Thetap0))/(1 - sr.0Cs*sr.0d)) - (Cos(Thetap0)*(-(u1*sr.0Cs*Sin(Thetap0)) - (u1*Cos(Thetap0)*sr.0d*sr.0Cs1)/(1 - sr.0Cs*sr.0d)))/Power(1 - sr.0Cs*sr.0d,2);
return ret;
}
