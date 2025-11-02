#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_Z11_pd_X_1_18_()
{
double ret;
ret = ((1 - sr.0Cs*sr.0d)*Sec(x_old[20])*Sec(Thetap0)*(-(Cos(x_old[18] - x_old[20] + Thetap8 - Thetap9)*(-(Cos(x_old[18] - Thetap0 + Thetap8)*(1 + 2*Cos(PAI/6.0 + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))) + 2*Sec(x_old[4])*Sin(PAI/6.0 + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[18] - Thetap0 + Thetap8))) + (-2*Cos(x_old[18] - Thetap0 + Thetap8)*Sec(x_old[4])*Sin(PAI/6.0 + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1) - (1 + 2*Cos(PAI/6.0 + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[18] - Thetap0 + Thetap8))*Sin(x_old[18] - x_old[20] + Thetap8 - Thetap9) + Sin(x_old[18] - x_old[20] + Thetap8 - Thetap9)*(-2*Cos(x_old[18] - Thetap0 + Thetap8)*Sec(x_old[4])*Sin(PAI/6.0 + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1) - (1 + 2*Cos(PAI/6.0 + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[18] - Thetap0 + Thetap8) - 2*Power(Sec(x_old[18]),2)*(-2*Cos(x_old[18] - Thetap0 + Thetap8)*Sec(x_old[4])*Sin(PAI/6.0 + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1) - (1 + 2*Cos(PAI/6.0 + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[18] - Thetap0 + Thetap8)) - 2*(-(Cos(x_old[18] - Thetap0 + Thetap8)*(1 + 2*Cos(PAI/6.0 + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))) + 2*Sec(x_old[4])*Sin(PAI/6.0 + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[18] - Thetap0 + Thetap8))*Tan(x_old[18])) + Cos(x_old[18] - x_old[20] + Thetap8 - Thetap9)*(Cos(x_old[18] - Thetap0 + Thetap8)*(1 + 2*Cos(PAI/6.0 + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)) - 2*Sec(x_old[4])*Sin(PAI/6.0 + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[18] - Thetap0 + Thetap8) - 2*(-2*Cos(x_old[18] - Thetap0 + Thetap8)*Sec(x_old[4])*Sin(PAI/6.0 + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1) - (1 + 2*Cos(PAI/6.0 + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[18] - Thetap0 + Thetap8))*Tan(x_old[18]))))/l3;
return ret;
}
