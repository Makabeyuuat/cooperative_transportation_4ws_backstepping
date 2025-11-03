#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_aqd_2_()
{
double ret;
ret = nu1*Sin(Thetap0 + thetaT) + Cos(Thetap0 + thetaT)*uact1(t)*(u_act(1) + sr.Cs*calc_SX_1_1_()*u_act(0));
return ret;
}
