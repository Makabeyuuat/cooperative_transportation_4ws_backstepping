#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_d_SX_d_t_3_1_()
{
double ret;
ret = (sr.0Cs*(u2 - (u1*sr.0Cs*Cos(Thetap0))/(1 - sr.0Cs*sr.0d))*Sin(Thetap0))/(1 - sr.0Cs*sr.0d) - (u1*Power(Cos(Thetap0),2)*sr.0Cs1)/Power(1 - sr.0Cs*sr.0d,2) + (sr.0Cs*Cos(Thetap0)*(-(u1*sr.0Cs*Sin(Thetap0)) - (u1*Cos(Thetap0)*sr.0d*sr.0Cs1)/(1 - sr.0Cs*sr.0d)))/Power(1 - sr.0Cs*sr.0d,2);
return ret;
}
