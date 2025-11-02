#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_aqd_14_()
{
double ret;
ret = athetap10d + asd*sr.0Cs + sr.0Cs1*Power(calc_SX_1_1_()*u1_act,2);
return ret;
}
