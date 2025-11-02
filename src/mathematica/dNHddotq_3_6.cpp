#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_dNHddotq_3_6_()
{
double ret;
ret = -(lv*Cos(q_map(5))*Cos(q_map(8) + q_map(5)))/2.0 - (lv*Sin(q_map(5))*Sin(q_map(8) + q_map(5)))/2.0;
return ret;
}
