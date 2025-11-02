#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_Z3_pd_X_1_2_()
{
double ret;
ret = -(Power(sr.0Cs,2)/(1 - sr.0Cs*sr.0d)) - sr.0Cs*Sec(Thetap0)*(-((sr.0Cs*Cos(Thetap0))/(1 - sr.0Cs*sr.0d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1);
return ret;
}
