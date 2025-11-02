#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha10_pd_X_2_2_()
{
double ret;
ret = -(sr.0Cs*Sec(Thetap0)*((sr.0Cs*Sin(Thetap0))/(1 - sr.0Cs*sr.0d) + (Sec(x_old[18])*(2*Cos(x_old[4] - Thetap0 + Thetap1)*Cos(x_old[18] - Thetap0 + Thetap8)*Sec(x_old[4])*Sin(PAI/6.0 + Thetap0 - Thetap1) - 2*Cos(PAI/6.0 + Thetap0 - Thetap1)*Cos(x_old[18] - Thetap0 + Thetap8)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1) + Cos(x_old[18] - Thetap0 + Thetap8)*(1 + 2*Cos(PAI/6.0 + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)) - 2*Sec(x_old[4])*Sin(PAI/6.0 + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[18] - Thetap0 + Thetap8) - (-2*Cos(PAI/6.0 + Thetap0 - Thetap1)*Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]) - 2*Sec(x_old[4])*Sin(PAI/6.0 + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[18] - Thetap0 + Thetap8)))/l2)) - sr.0Cs*Sec(Thetap0)*(-((sr.0Cs*Cos(Thetap0))/(1 - sr.0Cs*sr.0d)) + (Sec(x_old[18])*(-2*Cos(x_old[18] - Thetap0 + Thetap8)*Sec(x_old[4])*Sin(PAI/6.0 + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1) - (1 + 2*Cos(PAI/6.0 + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[18] - Thetap0 + Thetap8)))/l2)*Tan(Thetap0);
return ret;
}
