#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_dNHother_9_()
{
double ret;
ret = -(lv*Cos(q_map(22) + q_map(21))*Sin(q_map(21))*Power(qdot_map(21),2))/2.0 + (lv*Cos(q_map(21))*Sin(q_map(22) + q_map(21))*Power(qdot_map(21),2))/2.0 + Cos(q_map(22) + q_map(21))*(qdot_map(22) + qdot_map(21))*((lv*Sin(q_map(21))*qdot_map(21))/2.0 + qdot_map(19)) + Sin(q_map(22) + q_map(21))*(qdot_map(22) + qdot_map(21))*(-(lv*Cos(q_map(21))*qdot_map(21))/2.0 + qdot_map(20));
return ret;
}
