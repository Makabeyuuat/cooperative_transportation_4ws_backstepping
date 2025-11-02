#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha5_pd_X_2_2_()
{
double ret;
ret = -(sr.0Cs*Sec(Thetap0)*((sr.0Cs*Sin(Thetap0))/(1 - sr.0Cs*sr.0d) + (Sec(x_old[8])*(Cos(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(-(Cos(x_old[4] - Thetap0 + Thetap1)*Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Sin(x_old[4] - Thetap0 + Thetap1) - 2*Cos(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))) - Sin(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(Cos(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2) + Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Sin(x_old[4] - Thetap0 + Thetap1) - 2*Cos(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])) - 2*(-(Cos(x_old[4] - Thetap0 + Thetap1)*Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Sin(x_old[4] - Thetap0 + Thetap1) - 2*Cos(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))*Tan(x_old[6]))))/l3)) - sr.0Cs*Sec(Thetap0)*(-((sr.0Cs*Cos(Thetap0))/(1 - sr.0Cs*sr.0d)) + (Sec(x_old[8])*(Cos(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*Sin(x_old[4] - Thetap0 + Thetap1) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))) - Sin(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(-(Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)) + Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])) - 2*(Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*Sin(x_old[4] - Thetap0 + Thetap1) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))*Tan(x_old[6]))))/l3)*Tan(Thetap0);
return ret;
}
