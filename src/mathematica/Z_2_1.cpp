#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_z_2_1_()
{
double ret;
ret = (1 - sr.Cs*sr.d)*Tan(Thetap0);
return ret;
}
