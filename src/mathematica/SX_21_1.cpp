#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_SX_21_1_()
{
double ret;
ret = -((sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d)) + (Sec(x_old[20])*(-(Cos(x_old[18] - x_old[20] + Thetap8 - Thetap9)*(-2*Cos(x_old[18] - Thetap0 + Thetap8)*Sec(x_old[4])*Sin(Pi/6. + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1) - (1 + 2*Cos(Pi/6. + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[18] - Thetap0 + Thetap8))) + Sin(x_old[18] - x_old[20] + Thetap8 - Thetap9)*(Cos(x_old[18] - Thetap0 + Thetap8)*(1 + 2*Cos(Pi/6. + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)) - 2*Sec(x_old[4])*Sin(Pi/6. + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[18] - Thetap0 + Thetap8) - 2*(-2*Cos(x_old[18] - Thetap0 + Thetap8)*Sec(x_old[4])*Sin(Pi/6. + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1) - (1 + 2*Cos(Pi/6. + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[18] - Thetap0 + Thetap8))*Tan(x_old[18]))))/l3;
return ret;
}
