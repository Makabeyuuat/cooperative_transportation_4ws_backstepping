#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_Z2_pd_X_1_1_()
{
double ret;
ret = -(sr.d*Tan(Thetap0)*sr.Cs1);
return ret;
}
