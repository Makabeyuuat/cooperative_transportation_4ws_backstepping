#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha8_pd_X_3_15_()
{
double ret;
ret = ((1 - sr.0Cs*sr.0d)*Sec(x_old[14])*Sec(Thetap0)*(-(Sin(x_old[12] - x_old[14] + Thetap5 - Thetap6)*(-2*Cos(x_old[4] - Thetap0 + Thetap1)*Cos(x_old[12] - Thetap0 + Thetap5)*Sec(x_old[4])*Sin(PAI/6.0 - Thetap0 + Thetap1) - 2*Cos(x_old[12] - Thetap0 + Thetap5)*Sec(x_old[4])*Sin(PAI/6.0 - Thetap0 + Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]) - Sin(x_old[12] - Thetap0 + Thetap5)*(-2*Cos(PAI/6.0 - Thetap0 + Thetap1)*Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]) - 2*Cos(PAI/6.0 - Thetap0 + Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))) - Cos(x_old[12] - x_old[14] + Thetap5 - Thetap6)*(-2*Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4])*Sin(PAI/6.0 - Thetap0 + Thetap1)*Sin(x_old[12] - Thetap0 + Thetap5) - 2*Sec(x_old[4])*Sin(PAI/6.0 - Thetap0 + Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[12] - Thetap0 + Thetap5)*Tan(x_old[4]) + Cos(x_old[12] - Thetap0 + Thetap5)*(-2*Cos(PAI/6.0 - Thetap0 + Thetap1)*Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]) - 2*Cos(PAI/6.0 - Thetap0 + Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])) - 2*(-2*Cos(x_old[4] - Thetap0 + Thetap1)*Cos(x_old[12] - Thetap0 + Thetap5)*Sec(x_old[4])*Sin(PAI/6.0 - Thetap0 + Thetap1) - 2*Cos(x_old[12] - Thetap0 + Thetap5)*Sec(x_old[4])*Sin(PAI/6.0 - Thetap0 + Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]) - Sin(x_old[12] - Thetap0 + Thetap5)*(-2*Cos(PAI/6.0 - Thetap0 + Thetap1)*Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]) - 2*Cos(PAI/6.0 - Thetap0 + Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))*Tan(x_old[12]))))/l3;
return ret;
}
