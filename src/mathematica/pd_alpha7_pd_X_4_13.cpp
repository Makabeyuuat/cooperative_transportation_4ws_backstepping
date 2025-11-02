#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha7_pd_X_4_13_()
{
double ret;
ret = (1 - sr.0Cs*sr.0d)*Sec(Thetap0)*((Sec(x_old[12])*(2*Cos(x_old[12] - Thetap0 + Thetap5)*Sec(x_old[4])*Sin(PAI/6.0 - Thetap0 + Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1) + (1 - 2*Cos(PAI/6.0 - Thetap0 + Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[12] - Thetap0 + Thetap5)))/l2 + (Sec(x_old[12])*(-(Cos(x_old[12] - Thetap0 + Thetap5)*(1 - 2*Cos(PAI/6.0 - Thetap0 + Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))) + 2*Sec(x_old[4])*Sin(PAI/6.0 - Thetap0 + Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[12] - Thetap0 + Thetap5))*Tan(x_old[12]))/l2);
return ret;
}
