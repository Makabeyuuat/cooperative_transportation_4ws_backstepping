#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_g11_pd_X_1_()
{
double ret;
ret = (Cos(Thetap0)*sr.0d*sr.0Cs1)/Power(1 - sr.0Cs*sr.0d,2);
return ret;
}
