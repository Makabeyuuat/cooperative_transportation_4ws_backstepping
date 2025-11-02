#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_d_SX_d_t_2_1_()
{
double ret;
ret = Cos(Thetap0)*(u2 - (u1*sr.0Cs*Cos(Thetap0))/(1 - sr.0Cs*sr.0d));
return ret;
}
