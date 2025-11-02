#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_alpha_3_3_()
{
double ret;
ret = (1 - sr.0Cs*sr.0d)*Sec(Thetap0)*((Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]))/l1 + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))/l1);
return ret;
}
