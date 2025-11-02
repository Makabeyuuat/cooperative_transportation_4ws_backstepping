#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha4_pd_X_4_3_()
{
double ret;
ret = (1 - sr.0Cs*sr.0d)*Sec(Thetap0)*((Sec(x_old[6])*(Cos(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2) + Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Sin(x_old[4] - Thetap0 + Thetap1) - 2*Cos(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))))/l2 + (Sec(x_old[6])*(Cos(x_old[4] - Thetap0 + Thetap1)*Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2) - Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Sin(x_old[4] - Thetap0 + Thetap1) - 2*Cos(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))*Tan(x_old[6]))/l2) + (1 - sr.0Cs*sr.0d)*Sec(Thetap0)*((Sec(x_old[6])*(-(Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)) + Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))))/l2 + (Sec(x_old[6])*(-(Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*Sin(x_old[4] - Thetap0 + Thetap1)) - Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))*Tan(x_old[6]))/l2)*Tan(Thetap0);
return ret;
}
