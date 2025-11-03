#include "cooperative_transportation_4ws_backstepping/dynamics_integrator.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"        // 数学関数のヘッダーファイル
#include "cooperative_transportation_4ws_backstepping/Bezier.h"         // Bezier 曲線の関数
#include "cooperative_transportation_4ws_backstepping/vehicle.hpp"       // Vehicle クラスの宣言
#include "cooperative_transportation_4ws_backstepping/callback.hpp"     // コールバック関数の宣言
#include "cooperative_transportation_4ws_backstepping/getInputValue_dynamics.hpp"
#include "cooperative_transportation_4ws_backstepping/differential_equations_dynamics.hpp"
#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <Eigen/SVD> 
#include <Eigen/Core>   
#include <iostream>

// ここで IOFormat を定義しておく
static const Eigen::IOFormat CleanFmt(
    /*precision=*/6,
    /*flags=*/Eigen::DontAlignCols,
    /*coeffSeparator=*/", ",
    /*rowSeparator=*/"\n",
    /*rowPrefix=*/"[", 
    /*rowSuffix=*/"]"
);

using namespace std;

DynamicsIntegrator::DynamicsIntegrator(double m_b,
                                       double I_theta,
                                       double lv,
                                       double g,
                                       double rho,
                                       const PIDGains& drive_gains,
                                       const PIDGains& steer_gains,
                                       double dt,
                                       std::array<getInputValue,3>& inputValues)
                                       :kinematics_solver_() ,
                                       inputValues_ref_(inputValues)

 {}


//状態変数の目標加速度を計算
Eigen::Matrix<double,23,1> DynamicsIntegrator::computeXAlpha(
    const std::vector<double>& x,
    const std::vector<double>& x_d,
    const Eigen::Matrix<double,12,1>& u_kinematics)
    {

      double x_pos      = x[1];
      double y_pos      = x[2];
      double thetap0 = x[3];
      double phi1    = x[4];
      double phi2    = x[5];
      double xdot   = x_d[1];
      double ydot   = x_d[2];
      double thetap0dot = u_kinematics(1);
      double phi1dot = u_kinematics(2);
      double phi2dot = u_kinematics(3);
      double phi3dot = x_d[8];
      double phi4dot = x_d[10];
      double phi5dot = u_kinematics(6);
      double phi6dot = x_d[14];
      double phi7dot = x_d[16];
      double phi8dot = u_kinematics(9);
      double phi9dot = x_d[20];
      double phi10dot = x_d[22];
      Eigen::Matrix<double,23,12> SX =  kinematics_solver_.SX_mat();
      Eigen::Matrix<double,23,12> dSXdt =  kinematics_solver_.dSXdt_mat();
      Eigen::Matrix<double,12,1> pdud =  kinematics_solver_.pd_ud_vec();
      
      

      // 実際の前進速度と前輪操舵角速度
      u1_act = xdot * cos(thetap0) + ydot * sin(thetap0);
      u2_act = thetap0dot;
      u3_act = phi1dot;
      u4_act = phi2dot;
      u5_act = phi3dot;
      u6_act = phi4dot;
      u7_act = phi5dot;
      u8_act = phi6dot;
      u9_act = phi7dot;
      u10_act = phi8dot;
      u11_act = phi9dot;
      u12_act = phi10dot;

      Eigen::Matrix<double,23,1> Xalpha;
      
      // 結果格納用ベクトル
      u_act<<
          u1_act, u2_act, u3_act, u4_act, u5_act, u6_act, 
          u7_act, u8_act, u9_act, u10_act, u11_act, u12_act;
      
      
    
      //偏差ベクトル
      Eigen::Matrix<double,12,1> r_b = u_act - u_kinematics;
      
      //ゲイン
      Eigen::Matrix<double,12,1> gains;
      gains << 10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0;
      Eigen::Matrix<double,12,12> C = gains.asDiagonal();
          
      Eigen::Matrix<double,12,1> dot_C_rb = C*r_b;

      //目標加速度νを計算
      nu = -dot_C_rb + pdud;
      
      
      //状態変数ベクトルの目標加速度
      Xalpha = dSXdt*u_act + SX * nu;
      
      return Xalpha;
    }


//一般化座標の目標加速度を計算
Eigen::Matrix<double,27,1> DynamicsIntegrator::computeAlpha(
    const Eigen::Matrix<double,27,1>& q,
    const Eigen::Matrix<double,27,1>& qdot,
    const Eigen::Matrix<double,12,1>& u_kinematics)
    {
        Eigen::Matrix<double,27,1> alpha;
        //状態変数ベクトルの目標加速度 
        Eigen::Matrix<double,23,1> Xalpha = computeXAlpha(x_old, x_d, u_kinematics);

        asd = Xalpha(0);
        athetap0d = Xalpha(2);

        alpha = kinematics_solver_.aqd_vec();

        return alpha;
      }


void DynamicsIntegrator::step(
    const Eigen::Matrix<double,27,1>& q,
    const Eigen::Matrix<double,27,1>& qdot,
    const Eigen::Matrix<double,12,1>& u_kinematics)
    {

      Eigen::Matrix<double,27,27> C =  kinematics_solver_.Cxi_mat();
      Eigen::Matrix<double,27,27> M =  kinematics_solver_.Mxi_mat();
      Eigen::Matrix<double,18,27> A =  kinematics_solver_.Axi_mat();
      Eigen::Matrix<double,27,1>  K =  kinematics_solver_.Kxi_vec();

      //目標加速度の取得
      Eigen::Matrix<double,27,1> alpha = computeAlpha(q, qdot, u_kinematics);

      //qの目標加速度を分解
      Eigen::Matrix<double,15,1> alpha_xi;
      alpha_xi
      <<
      alpha(0), alpha(1), alpha(2), alpha(3), alpha(4), alpha(5), alpha(6), alpha(7), alpha(8), 
      alpha(9), alpha(10), alpha(11), alpha(12), alpha(13), alpha(14);

      Eigen::Matrix<double,12,1> alpha_zeta;
      alpha_zeta 
      <<
      alpha(15), alpha(16), alpha(17), alpha(18), alpha(19), alpha(20), alpha(21), alpha(22), 
      alpha(23), alpha(24), alpha(25), alpha(26);

      //qdotを分解
      Eigen::Matrix<double,15,1> qdot_xi;
      qdot_xi 
      <<
      qdot(0), qdot(1), qdot(2), qdot(3), qdot(4), qdot(5), qdot(6), qdot(7), qdot(8), 
      qdot(9), qdot(10), qdot(11), qdot(12), qdot(13), qdot(14);
      Eigen::Matrix<double,12,1> qdot_zeta;
      qdot_zeta 
      <<
      qdot(15), qdot(16), qdot(17), qdot(18), qdot(19), qdot(20), qdot(21), qdot(22), 
      qdot(23), qdot(24), qdot(25), qdot(26);


      // ラムダの導出
      constexpr int NXI   = 15;  // q_xi の次元
      constexpr int NZETA = 12;  // q_zeta の次元

      // 1) 慣性行列 M (27x27) のブロック分け
      Eigen::Matrix<double,NXI,   NXI>   M_xixi   = M.topLeftCorner   (NXI,   NXI);   // M_ξξ
      Eigen::Matrix<double,NXI,   NZETA> M_xizeta = M.topRightCorner  (NXI,   NZETA); // M_ξζ
      Eigen::Matrix<double,NZETA, NXI>   M_zetaxi = M.bottomLeftCorner(NZETA, NXI);   // M_ζξ
      Eigen::Matrix<double,NZETA, NZETA> M_zetazeta = M.bottomRightCorner(NZETA, NZETA); // M_ζζ

      // 2) コリオリ(あるいは速度係数)行列 C (27x27) のブロック分け
      Eigen::Matrix<double,NXI,   NXI>   C_xixi   = C.topLeftCorner   (NXI,   NXI);
      Eigen::Matrix<double,NXI,   NZETA> C_xizeta = C.topRightCorner  (NXI,   NZETA);
      Eigen::Matrix<double,NZETA, NXI>   C_zetaxi = C.bottomLeftCorner(NZETA, NXI);
      Eigen::Matrix<double,NZETA, NZETA> C_zetazeta = C.bottomRightCorner(NZETA, NZETA);

      // 3) 拘束行列 A (18x27) の分割（そのまま）
      Eigen::Matrix<double,18, NXI>   A_xi   = A.leftCols (NXI);     // 18x15
      Eigen::Matrix<double,18, NZETA> A_zeta = A.rightCols(NZETA);   // 18x12

      // 3) A を転置した版（よく使うやつ）
      Eigen::Matrix<double,27,18> AT = A.transpose();                 // 27x18
      Eigen::Matrix<double,NXI,   18> AT_xi   = AT.topRows(NXI);      // 15x18 … ξに対応
      Eigen::Matrix<double,NZETA, 18> AT_zeta = AT.bottomRows(NZETA); // 12x18 … ζに対応

      // 4) ddotq,dotqを含まない項 K (27x1) の分割
      Eigen::Matrix<double, NXI,   1> K_xi   = K.topRows   (NXI);
      Eigen::Matrix<double, NZETA, 1> K_zeta = K.bottomRows(NZETA);

      
      Eigen::Matrix<double,15,1> rhs =  (M_xixi * alpha_xi + C_xixi * qdot_xi + K_xi);
      // COD による最小ノルム解
      Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double,15,18>> cod(AT_xi);
      Eigen::Matrix<double,18,1> lambda = cod.solve(rhs);
      lamda_data = lambda;


      //駆動力の導出
      Eigen::Matrix<double,12,1> rhs_zeta_only = M_zetazeta * alpha_zeta + C_zetazeta * qdot_zeta  + K_zeta;                   
      // 拘束反力を引いて最終的な駆動トルク
      Eigen::Matrix<double,12,1> Q_zeta = rhs_zeta_only - AT_zeta * lambda;


      
     
      //駆動力計算
      Q_phiR1   = Q_zeta(0);
      Q_varphiR1 = Q_zeta(1);
      Q_phiF1 = Q_zeta(2);
      Q_varphiF1 = Q_zeta(3);
      Q_phiR2   = Q_zeta(4);
      Q_varphiR2 = Q_zeta(5);
      Q_phiF2 = Q_zeta(6);
      Q_varphiF2 = Q_zeta(7);
      Q_phiR3   = Q_zeta(8);
      Q_varphiR3 = Q_zeta(9);
      Q_phiF3 = Q_zeta(10);
      Q_varphiF3 = Q_zeta(11);
     

      // 3) step() の最後でそれぞれに詰める
      // 車両1
      inputValues_ref_[0].rearTorque  = inputValues_ref_[0].computeRearWheelTorque(Q_varphiR1, q(17),  q(15));
      inputValues_ref_[0].frontTorque = inputValues_ref_[0].computeFrontWheelTorque(Q_varphiF1, q(17), q(15));

      // 車両2
      inputValues_ref_[1].rearTorque  = inputValues_ref_[1].computeRearWheelTorque(Q_varphiR2, q(21), q(19));
      inputValues_ref_[1].frontTorque = inputValues_ref_[1].computeFrontWheelTorque(Q_varphiF2, q(21), q(19));

      // 車両3
      inputValues_ref_[2].rearTorque  = inputValues_ref_[2].computeRearWheelTorque(Q_varphiR3, q(25), q(23));
      inputValues_ref_[2].frontTorque = inputValues_ref_[2].computeFrontWheelTorque(Q_varphiF3, q(25), q(23));



      v1_torque_rear[0] = inputValues_ref_[0].rearTorque[0];  // 左後輪
      v1_torque_rear[1] = inputValues_ref_[0].rearTorque[1];  // 右後輪
      v1_torque_front[0] = inputValues_ref_[0].frontTorque[0];  // 左後輪
      v1_torque_front[1] = inputValues_ref_[0].frontTorque[1];  // 右後輪

      v2_torque_rear[0] = inputValues_ref_[1].rearTorque[0];  // 左後輪
      v2_torque_rear[1] = inputValues_ref_[1].rearTorque[1];  // 右後輪
      v2_torque_front[0] = inputValues_ref_[1].frontTorque[0];  // 左後輪
      v2_torque_front[1] = inputValues_ref_[1].frontTorque[1];  // 右後輪

      v3_torque_rear[0] = inputValues_ref_[2].rearTorque[0];  // 左後輪
      v3_torque_rear[1] = inputValues_ref_[2].rearTorque[1];  // 右後輪
      v3_torque_front[0] = inputValues_ref_[2].frontTorque[0];  // 左後輪
      v3_torque_front[1] = inputValues_ref[2]_frontTorque[1];  // 右後輪
     

      //積分用の配列に代入
      x_dd[0] = 0.0;
      x_dd[1] = 0.0;
      x_dd[2] = 0.0;
      x_dd[3] = 0.0;
      x_dd[4] = 0.0;
      x_dd[5] = 0.0;
      x_dd[6] = 0.0;
      x_dd[7] = 0.0;
      }




// void DynamicsIntegrator::step(const Eigen::Vector3d& q,
//                               const Eigen::Vector3d& qdot,
//                               double& phi,
//                               double& phidot,
//                               double u1,
//                               double u2)
// {
//     // 状態展開
//     double x        = q(0);
//     double y        = q(1);
//     double theta    = q(2);
//     double xdot     = qdot(0);
//     double ydot     = qdot(1);
//     double thetadot = qdot(2);

//     //PID制御でtauを計算
//     double u1_act = xdot * cos(theta) + ydot * sin(theta);
//     double tau1   = drive_pid_.compute(u1, u1_act);
//     Tau1 = tau1;
//     double tau2   = steer_pid_.compute(u2, phidot);
//     Tau2 = tau2;
//     //駆動力
//     Eigen::Vector3d Qc;
//     Qc << tau1 * std::cos(theta),
//           tau1 * std::sin(theta),
//           0.0;
//     double Qphi = tau2;

//     //各行列を定義
//     // 質量行列(3x3)
//     Eigen::Matrix3d Mxi;
//     Mxi <<  m_b, 0.0, -(lv *m_b * sin(theta))/2.0,
//             0.0, m_b, (lv *m_b * cos(theta))/2.0,
//             -(lv *m_b * sin(theta))/2.0, (lv *m_b * cos(theta))/2.0, (2*I_theta_ + m_b*((pow(lv,2)*pow(cos(theta),2))/2.0 + (pow(lv,2)*pow(sin(theta),2))/2.0))/2.0;

//     // コリオリ行列(3x3)
//     Eigen::Matrix3d Cxi;
//     Cxi <<  0.0, 0.0, -(lv*m_b*cos(theta)*thetadot),
//             0.0, 0.0, -(lv*m_b*sin(theta)*thetadot),
//             0.0, 0.0, 0.0;

//     // 重力ベクトル(3x1)
//     Eigen::Vector3d Kxi;
//     Kxi << GRAV*m_b*sin(rho_), 0.0, -(GRAV*lv*m_b*sin(rho_)*sin(theta))/2.0;

//     // 拘束行列(3x2)
//     Eigen::Matrix<double,3,2> Axi;
//     Axi <<  sin(theta + phi), sin(theta),
//             -cos(theta + phi), -cos(theta),
//             -lv * cos(phi), 0.0;

//     // ヤコビ行列(2x3)
//     Eigen::Matrix<double,2,3> J;
//     J <<  sin(theta + phi), -cos(theta + phi), -lv * cos(phi),
//             sin(theta), -cos(theta), 0.0;

//     // 拘束付き運動方程式行列 H (5x5)
//     Eigen::Matrix<double,5,5> H;
//     H.setZero();
//     H.block<3,3>(0,0) = Mxi;
//     H.block<3,2>(0,3) = Axi;
//     H.block<2,3>(3,0) = J;
//     // H.block<2,2>(3,3) = Zero

//     // 右辺ベクトル b (5x1)
//     Eigen::Matrix<double,5,1> b;
//     b.setZero();
//     b.block<3,1>(0,0) = Qc - Cxi * qdot - Kxi;
//     // 非ホロを微分した値dのddq以外の項を代入
//     b(3) = -xdot*(thetadot+phidot)*cos(theta+phi) - ydot*(thetadot+phidot)*sin(theta+phi)-lv*thetadot*phidot*sin(phi);
//     b(4) = -xdot*thetadot*cos(theta) + ydot*thetadot*sin(theta);

//     // Hの疑似逆行列を計算
//     Eigen::MatrixXd Hxi_pinv = (H.transpose() * H).inverse()* H.transpose();
//     Eigen::VectorXd sol= Hxi_pinv * b;
    
//     Eigen::Vector3d qdd = sol.block<3,1>(0,0);
//     Eigen::Vector2d lambda = sol.block<2,1>(3,0); 
//     double phidd = Qphi;

//     //積分用の配列に代入
//     x_dd[0] = 0.0;
//     x_dd[1] = qdd(0);
//     x_dd[2] = qdd(1);
//     x_dd[3] = qdd(2);
//     x_dd[4] = phidd;
// }