#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha11_pd_X_3_2_()
{
double ret;
ret = -((sr.Cs*Sec(x_old[20])*Sec(Thetap0)*(-(Cos(x_old[18] - x_old[20] + Thetap8 - Thetap9)*(-2*Cos(x_old[4] - Thetap0 + Thetap1)*Cos(x_old[18] - Thetap0 + Thetap8)*Sec(x_old[4])*Sin(Pi/6. + Thetap0 - Thetap1) - 2*Cos(x_old[18] - Thetap0 + Thetap8)*Sec(x_old[4])*Sin(Pi/6. + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]) - Sin(x_old[18] - Thetap0 + Thetap8)*(2*Cos(Pi/6. + Thetap0 - Thetap1)*Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]) + 2*Cos(Pi/6. + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))) + Sin(x_old[18] - x_old[20] + Thetap8 - Thetap9)*(-2*Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4])*Sin(Pi/6. + Thetap0 - Thetap1)*Sin(x_old[18] - Thetap0 + Thetap8) - 2*Sec(x_old[4])*Sin(Pi/6. + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[18] - Thetap0 + Thetap8)*Tan(x_old[4]) + Cos(x_old[18] - Thetap0 + Thetap8)*(2*Cos(Pi/6. + Thetap0 - Thetap1)*Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]) + 2*Cos(Pi/6. + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])) - 2*(-2*Cos(x_old[4] - Thetap0 + Thetap1)*Cos(x_old[18] - Thetap0 + Thetap8)*Sec(x_old[4])*Sin(Pi/6. + Thetap0 - Thetap1) - 2*Cos(x_old[18] - Thetap0 + Thetap8)*Sec(x_old[4])*Sin(Pi/6. + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]) - Sin(x_old[18] - Thetap0 + Thetap8)*(2*Cos(Pi/6. + Thetap0 - Thetap1)*Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]) + 2*Cos(Pi/6. + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))*Tan(x_old[18]))))/l3);
return ret;
}
