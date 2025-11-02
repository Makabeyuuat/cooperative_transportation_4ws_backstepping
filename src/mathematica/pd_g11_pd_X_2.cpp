#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_g11_pd_X_2_()
{
double ret;
ret = (sr.0Cs*Cos(Thetap0))/Power(1 - sr.0Cs*sr.0d,2);
return ret;
}
