#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_Z4_pd_X_1_1_()
{
double ret;
ret = -(sr.0d*Sec(Thetap0)*(-((sr.0Cs*Cos(Thetap0))/(1 - sr.0Cs*sr.0d)) + (Sec(x_old[6])*(-(Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*Sin(x_old[4] - Thetap0 + Thetap1)) - Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))))/l2)*sr.0Cs1) + (1 - sr.0Cs*sr.0d)*Sec(Thetap0)*(-((sr.0Cs*Cos(Thetap0)*sr.0d*sr.0Cs1)/Power(1 - sr.0Cs*sr.0d,2)) - (Cos(Thetap0)*sr.0Cs1)/(1 - sr.0Cs*sr.0d));
return ret;
}
