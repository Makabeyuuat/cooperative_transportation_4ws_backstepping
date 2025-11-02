#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha3_pd_X_2_3_()
{
double ret;
ret = (1 - sr.0Cs*sr.0d)*Sec(Thetap0)*((sr.0Cs*Cos(Thetap0))/(1 - sr.0Cs*sr.0d) - (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1) + (1 - sr.0Cs*sr.0d)*Power(Sec(Thetap0),3)*(-((sr.0Cs*Cos(Thetap0))/(1 - sr.0Cs*sr.0d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1) + 2*(1 - sr.0Cs*sr.0d)*Sec(Thetap0)*(-((Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]))/l1) + (sr.0Cs*Sin(Thetap0))/(1 - sr.0Cs*sr.0d))*Tan(Thetap0) + (1 - sr.0Cs*sr.0d)*Sec(Thetap0)*(-((sr.0Cs*Cos(Thetap0))/(1 - sr.0Cs*sr.0d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1)*Power(Tan(Thetap0),2);
return ret;
}
