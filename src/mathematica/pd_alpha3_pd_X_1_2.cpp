#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha3_pd_X_1_2_()
{
double ret;
ret = -((Power(sr.0Cs,2)*Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4])*Sec(Thetap0))/l1) - (2*sr.0Cs*Cos(x_old[4] - Thetap0 + Thetap1)*(1 - sr.0Cs*sr.0d)*Sec(x_old[4])*Power(Sec(Thetap0),2)*(-((sr.0Cs*Cos(Thetap0))/(1 - sr.0Cs*sr.0d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1))/l1 - sr.0Cs*(-(Power(sr.0Cs,2)/(1 - sr.0Cs*sr.0d)) - sr.0Cs*Sec(Thetap0)*(-((sr.0Cs*Cos(Thetap0))/(1 - sr.0Cs*sr.0d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1))*Tan(Thetap0) - sr.0Cs*(-(sr.0Cs*Sec(Thetap0)*(-((Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]))/l1) + (sr.0Cs*Sin(Thetap0))/(1 - sr.0Cs*sr.0d))) - sr.0Cs*Sec(Thetap0)*(-((sr.0Cs*Cos(Thetap0))/(1 - sr.0Cs*sr.0d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1)*Tan(Thetap0)) + (Power(sr.0Cs,2)*sr.0d*sr.0Cs1)/Power(1 - sr.0Cs*sr.0d,2) - Sec(Thetap0)*(-((sr.0Cs*Cos(Thetap0))/(1 - sr.0Cs*sr.0d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1)*sr.0Cs1 + (1 - sr.0Cs*sr.0d)*Sec(Thetap0)*((-2*Power(sr.0Cs,2)*Cos(Thetap0)*sr.0d*sr.0Cs1)/Power(1 - sr.0Cs*sr.0d,3) - (2*sr.0Cs*Cos(Thetap0)*sr.0Cs1)/Power(1 - sr.0Cs*sr.0d,2)) - sr.0Cs*Sec(Thetap0)*(-((sr.0Cs*Cos(Thetap0)*sr.0d*sr.0Cs1)/Power(1 - sr.0Cs*sr.0d,2)) - (Cos(Thetap0)*sr.0Cs1)/(1 - sr.0Cs*sr.0d));
return ret;
}
