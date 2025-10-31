#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_Axi_16_3_()
{
double ret;
ret = -2*lv*Sin(Pi/6. + q_map(2));
return ret;
}
