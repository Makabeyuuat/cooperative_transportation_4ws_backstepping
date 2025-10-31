#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha7_pd_X_4_12_()
{
double ret;
ret = (1 - sr.Cs*sr.d)*Sec(Thetap0)*((Power(Sec(x_old[12]),3)*(-2*Cos(x_old[12] - Thetap0 + Thetap5)*Sec(x_old[4])*Sin(Pi/6. - Thetap0 + Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1) - (1 - 2*Cos(Pi/6. - Thetap0 + Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[12] - Thetap0 + Thetap5)))/l2 + (Sec(x_old[12])*(2*Cos(x_old[12] - Thetap0 + Thetap5)*Sec(x_old[4])*Sin(Pi/6. - Thetap0 + Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1) + (1 - 2*Cos(Pi/6. - Thetap0 + Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[12] - Thetap0 + Thetap5)))/l2 + (2*Sec(x_old[12])*(-(Cos(x_old[12] - Thetap0 + Thetap5)*(1 - 2*Cos(Pi/6. - Thetap0 + Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))) + 2*Sec(x_old[4])*Sin(Pi/6. - Thetap0 + Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[12] - Thetap0 + Thetap5))*Tan(x_old[12]))/l2 + (Sec(x_old[12])*(-2*Cos(x_old[12] - Thetap0 + Thetap5)*Sec(x_old[4])*Sin(Pi/6. - Thetap0 + Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1) - (1 - 2*Cos(Pi/6. - Thetap0 + Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[12] - Thetap0 + Thetap5))*Power(Tan(x_old[12]),2))/l2);
return ret;
}
