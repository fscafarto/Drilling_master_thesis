#ifndef ANYMALS_LIKE_H
#define ANYMALS_LIKE_H

//C headers
#include <cstdlib>

//#define NDEBUG //disable flags for debug, for saving time

//#define EIGEN_FAST_MATH 0 //disable some optimization techniques used by Eigen, more accuracy in results

//Eigen headers
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>

//STD headers
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <sstream>

//graph
#include <iostream>
#include <sstream>
#include <iostream>
#include <fstream>

//raisim
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#if WIN32
#include <timeapi.h>
#endif

//iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

//iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>

//ros
//#include <ros/ros.h>
#include "boost/thread.hpp"
#include <chrono>
//#include <ros/package.h> 

//tf
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"
#include <tf/transform_broadcaster.h>

//alglib
#include "alglib/optimization.h"

//my header files
#include "optimal.h"
#include "planning.h"

#include <atomic>

template <typename Type, int Size> using Vector = Eigen::Matrix<Type, Size, 1>;

using namespace std;

struct EigenRobotState
{
    void resize(int nrOfInternalDOFs)
    {
        jointPos.resize(nrOfInternalDOFs); //resize del vettore, noto il numero dei giunti
        jointVel.resize(nrOfInternalDOFs);
    }

    void random()
    {
        world_H_base.setIdentity();
        jointPos.setRandom();
        baseVel.setRandom();
        jointVel.setRandom();
        gravity[0] = 0;
        gravity[1] = 0;
        gravity[2] = -9.8;
    }

    Eigen::Matrix4d world_H_base;
    //Eigen::VectorXd jointPos;
    Eigen::Matrix<double,12,1> jointPos;
    Eigen::Matrix<double,6,1> baseVel;
    //Eigen::VectorXd jointVel;
    Eigen::Matrix<double,12,1> jointVel;
    Eigen::Vector3d gravity;
    
    Eigen::MatrixXd v; //full velocity vector

    // See https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class QUADRUPED{

    public:

    QUADRUPED();

    int init_model();

    void set_anymalC(raisim::ArticulatedSystem* data);
    
    void set_balla(raisim::SingleBodyObject* data);

    int update(int flag);

    Eigen::Matrix4d get_world_base();

    void get_raisim(int flag);

    void compute_m22_bar();

    void compute_c2_bar();

    int compute_j_st(int flag);

    void compute_j_st_j_bar(int flag);

    int stand_phase();

    void loop();

    void run();

    Eigen::Matrix<double,6,1> get_init_pos();

    Eigen::MatrixXd getBRpos();

    Eigen::MatrixXd getBLpos();

    Eigen::MatrixXd getFRpos();

    Eigen::MatrixXd getFLpos();

    Eigen::Vector3d get_pc();

    Eigen::Matrix<double,6,1> get_init_vel();

    Eigen::Matrix<double,6,1> get_r_c();

    Eigen::Matrix<double,6,1> get_r_c_ref();

    Eigen::Matrix<double,6,1> get_r_c_dot();

    Eigen::Matrix<double,6,1> get_r_c_ref_dot();

    double get_m();

    Eigen::Matrix<double,18,18> get_mass_matrix();

    Eigen::Matrix<double,6,1> get_r_c_ref_dot_dot();

    void set_w_com_des(Eigen::Matrix<double,6,1> data);

    double get_mu();

    Eigen::Matrix<double,6,1> get_accd();

    Eigen::Matrix<double,6,1> get_veldelta();

    Eigen::Matrix<double,6,1> get_posdelta();

    Eigen::Matrix<double,12,6> get_j_st_c_bar_0();

    Eigen::Matrix<double,6,1> get_w_com_des();

    Eigen::Matrix<double,12,12> get_j_st_j_bar_0();

    Eigen::Matrix<double,18,1>  get_bias_com();

    Eigen::Matrix<double,12,18> get_j_st_dot_0();

    Eigen::Matrix<double,12,1> get_joint_pos();

    Eigen::Matrix<double,18,1> get_v_c();

    void set_q_dot_dot_des(Eigen::Matrix<double,12,1> data);

    void set_f_gr_star(Eigen::Matrix<double,12,1> data);

    Eigen::Matrix<double,6,6> get_j_st_c_bar_1();

    Eigen::Matrix<double,6,12> get_j_st_j_bar_1();

    Eigen::Matrix<double,6,1> get_j_st_dot_1();

    Eigen::Matrix<double,6,6> get_j_sw_c_bar();

    Eigen::Matrix<double,6,12> get_j_sw_j_bar();

    Eigen::Matrix<double,6,1> get_j_sw_dot();

    //void set_world(raisim::World data);

    void set_init_pos(Eigen::Matrix<double,6,1> data);

    void set_init_vel(Eigen::Matrix<double,6,1> data);

    int swing_phase();

    int swing_phase2();

    void set_duration(double data);

    Eigen::MatrixXd getBLvel();

    Eigen::MatrixXd getBRvel();

    Eigen::MatrixXd getFLvel();

    Eigen::MatrixXd getFRvel();

    bool get_raisim_ready();

    Eigen::Matrix<double,18,18> get_eig_mass_matrix();

    void pd_controller();

    raisim::ArticulatedSystem* get_anymal();

    void set_world(raisim::World data);

    raisim::World* get_world();
    
    raisim::RaisimServer* get_server();

    void integrate_world();

    void pd_controller_current();

    Eigen::Vector3d get_com_raisim();

    void compute_estimation(int flag);

    Eigen::Matrix<double,12,1> get_f_ext();

    int get_stl1();

    int get_stl2();

    int get_swl1();

    int get_swl2();

    Eigen::Matrix<double,24,18> get_j_bar();

    void drilling();

    void walking();

    void push();

    void compute_robustness();

    Eigen::Matrix<double,3,1> get_tip_force();

    bool get_do_replanning();

    double get_pitch();

    double get_actuator_torque();

    void stack_tip_force();

    void low_pass_filter(double *x, double *y, int M);

    double get_tip_force_filt();

    double get_yy();

    double get_yz();

    bool get_force_reduced();

    void set_force_reduced(bool data);

    bool get_replanning_2_acceleration();

    double get_gain();

    bool get_impact();

    bool get_drill();

    bool get_force_found();
    


    private:

    EigenRobotState _eig_robot_state;

    raisim::ArticulatedSystem* _anymalC;

    raisim::SingleBodyObject* _balla;

    Eigen::Vector3d _pb;

    Eigen::Matrix<double,4,1> _base_quaternion;

    double _roll, _pitch, _yaw;

    iDynTree::KinDynComputations kinDynComp;

    Eigen::Matrix<double,6,1> _r_c;

    Eigen::Vector3d _pc;

    Eigen::Matrix<double,18,18> _eig_mass_matrix;

    Eigen::Vector3d _pc_dot;

    Eigen::Matrix<double,6,1> _r_c_dot;

    Eigen::Matrix<double,18,1> _v_c;

    Eigen::Vector3d _pbc;

    Eigen::Matrix3d _s;

    Eigen::Matrix<double,6,6> _x; 

    Eigen::Matrix<double,6,6> _m11; 

    Eigen::Matrix<double,6,12> _m12;

    Eigen::Matrix<double,6,12> _j_s;

    Eigen::Matrix<double,18,18> _t_bar;

    Eigen::Matrix<double,18,18> _m_bar;

    Eigen::Matrix<double,12,12> _m22_bar;

    const iDynTree::Model *model;

    Eigen::Matrix<double,18,1> _h;

    Eigen::Vector3d _pb_dot;

    Eigen::Vector3d _pbc_dot;

    Eigen::Vector3d _m_dr;

    double _robot_mass;

    Eigen::Matrix3d _s_dot;

    Eigen::Matrix<double,6,6> _dx;

    Eigen::Matrix3d _s_mdr;

    Eigen::Matrix<double,6,6> _dmb;

    Eigen::Matrix<double,6,6> _dmb_1;

    Eigen::Matrix<double,6,6> _dmb_2;

    Eigen::Matrix<double,6,12> _dj_s;

    Eigen::Matrix<double,18,18> _t_inv_der;

    Eigen::Matrix<double,6,18> _j_t_1;

    Eigen::Matrix<double,6,18> _j_t_2;

    Eigen::Matrix<double,6,18> _j_t_3;

    Eigen::Matrix<double,6,18> _j_t_4;

    Eigen::Matrix<double,24,18> _j;

    Eigen::Matrix<double,12,18> _j_st_temp;

    Eigen::Matrix<double,12,18> _j_st_0;

    int _swl1;

	int _swl2;

	int _stl1;

	int _stl2;

    Eigen::Matrix<double,6,18> _j_sw;

    Eigen::Matrix<double,12,18> _j_st_1_new;

    Eigen::Matrix<double,6,1> _j_t_1_dot_temp;

    Eigen::Matrix<double,6,1> _j_t_2_dot_temp;

    Eigen::Matrix<double,6,1> _j_t_3_dot_temp;

    Eigen::Matrix<double,6,1> _j_t_4_dot_temp;

    Eigen::Matrix<double,24,1> _Jdqd;

    Eigen::Matrix<double,24,1> _JdqdCOM;

    Eigen::Matrix<double,12,1> _JdqdCOM_lin;

	Eigen::Matrix<double,12,24> _B;

    Eigen::Matrix<double,12,18> _j_st_dot_0;

    Eigen::Matrix<double,6,1> _j_st_dot_1;

    Eigen::Matrix<double,6,1> _j_sw_dot;

    Eigen::Matrix<double,6,18> _j_st_1;
    
    Eigen::Matrix<double,6,18> _eig_jacobian;
    
    Eigen::Matrix<double,3,18> _eig_com_jacobian;

    Eigen::Matrix<double,12,18> _j_st_bar_0;
    
    Eigen::Matrix<double,12,12> _j_st_j_bar_0;
    
    Eigen::Matrix<double,12,6> _j_st_c_bar_0;

    Eigen::Matrix<double,6,18> _j_st_bar_1;
    
    Eigen::Matrix<double,6,12> _j_st_j_bar_1;
    
    Eigen::Matrix<double,6,6> _j_st_c_bar_1;

    Eigen::Matrix<double,6,6> _j_sw_c_bar;

    Eigen::Matrix<double,24,18> _j_st_dot;

    Eigen::Matrix<double,24,1> _j_st_dot_temp;

    Eigen::Matrix<double,18,1> _bias_com;

    Eigen::Matrix<double,6,18> _j_sw_bar;

    Eigen::Matrix<double,6,12> _j_sw_j_bar;

    Eigen::Matrix<double,12,18> _j_st_bar_1_new;
    
    Eigen::Matrix<double,12,12> _j_st_j_bar_1_new;

    Eigen::Matrix<double,24,18> _j_bar;

    double _begin;

    double _duration=0.0;

    Eigen::Matrix<double,6,1> _r_c_ref;

    Eigen::Matrix<double,6,1> _r_c_ref_dot;

    Eigen::Matrix<double,6,1> _r_c_ref_dot_dot;

    Eigen::VectorXd _torque_array;

    Eigen::Matrix<double,12,1> _jnt_torques_star;

    Eigen::Matrix<double,12,1> _q_dot_dot_des;

    Eigen::Matrix<double,12,1> _f_gr_star;

    double _begin_first;

    Eigen::Matrix<double,6,1> _init_vel;

    double _mu=0.6;

    Eigen::Matrix<double,6,1> _accd;

    Eigen::Matrix<double,6,1> _posdelta;

    Eigen::Matrix<double,6,1> _veldelta;

    Eigen::Matrix<double,6,1> _init_pos;

    Eigen::Matrix<double,6,1> _w_com_des;

    bool _server_launched=false;

    bool _raisim_ready=false;

    std::array<bool, 4> _contact;

    raisim::World _world;

    raisim::RaisimServer _server;

    int _index=0;

    double _tempo_stop=0.0;

    double _tempo_stop_effettivo=0.0;

    double _begin_set_mutex_false=0.0;

    std::atomic_bool _integration_started = ATOMIC_VAR_INIT(false);

    Vector<double,20> _jointNominalConfig; //(in caso di linear actuator sostituire 20 con 19)

    Vector<double,19> _jointVelocityTarget; //(in caso di linear actuator sostituire 19 con 18)

    Vector<double,19> _jointPgain; //(in caso di linear actuator sostituire 19 con 18)

    Vector<double,19> _jointDgain; //(in caso di linear actuator sostituire 19 con 18)

    bool _first_cycle=false;

    std::atomic_bool _start_integrate = ATOMIC_VAR_INIT(false);

    Eigen::Vector3d vec_sample_com_eig;
    
    bool _start_estimate=false;

    iDynTree::Transform  Tbl, Tbr, Tfl, Tfr;

    Eigen::Matrix<double,3,3> _Tbl;

    Eigen::Matrix<double,3,3> _Tbr;

    Eigen::Matrix<double,3,3> _Tfl;

    Eigen::Matrix<double,3,3> _Tfr;

    Eigen::Matrix<double,12,1> _rho=Eigen::MatrixXd::Zero(12,1);

    Eigen::Matrix<double,12,1> _Fgrf=Eigen::MatrixXd::Zero(12,1);

    Eigen::Matrix<double,12,1> _f_gr=Eigen::MatrixXd::Zero(12,1);   

    Eigen::Matrix<double,12,1> _f_ext=Eigen::MatrixXd::Zero(12,1);

    std::vector<Eigen::Matrix<double,12,1>>  _yd_prev;
    
    std::vector<Eigen::Matrix<double,12,1>>  _yw_prev;

    std::vector<Eigen::Matrix<double,12,1>>  _w_prev;

    std::vector<Eigen::Matrix<double,12,1>>  _ygamma_prev;

    std::vector<Eigen::Matrix<double,12,1>>  _yd;
    
    std::vector<Eigen::Matrix<double,12,1>>  _yw;

    std::vector<Eigen::Matrix<double,12,1>>  _w;

    std::vector<Eigen::Matrix<double,12,1>>  _ygamma;

    Eigen::VectorXd _coeffs=Eigen::VectorXd::Zero(4);

    double _T=0.001;

    Eigen::Matrix<double,12,12> _k_1;

    Eigen::Matrix<double,12,12> _k_2;

    Eigen::Matrix<double,12,12> _k_3;

    Eigen::Matrix<double,12,1> _d;

    Eigen::Matrix<double,18,1> _g;

    Eigen::Matrix<double,1,18> _v_pseudo;

    Eigen::Matrix<double,18,18> _c;

    Eigen::Matrix<double,18,18> _Ccom;

    Eigen::Matrix<double,18,1> _Ctq;

    bool _replanning_done=false;

    Eigen::Matrix3d _contact_frame;

    Eigen::Matrix3d _contact_frame_rig;

    double _delay;

    bool _drill=true;

    bool _spin=true;

    double _begin_law=0.0;

    double _drill_rig_pos=0.0;

    std::atomic_bool _start_pushing = ATOMIC_VAR_INIT(false);

    bool _drill_done=false;

    Eigen::Matrix<double,3,1> _tip_force;

    bool _pitch_ok=true; 

    Eigen::Vector3d _fc_i;

    int _index_rob = 0;

    Eigen::Vector3d _z_i;

    double _product=0.0;

    double _alpha_i=0.0;

    double _sum=0.0;

    double _theta_i = 0.0;

    double _lambda = 4;

    double _h_rob = 0.0;

    double _robustness = 0.0;

    bool _do_replanning = false;

    Eigen::Matrix<double, 300, 1> _robustness_array;

    int _index_rob_array = 0;

    double _gradient = 0.0;

    Eigen::Matrix<double, 500, 2> _tip_force_array = Eigen::MatrixXd::Zero(500,1);

    Eigen::Matrix<double, 500, 2> _tip_force_array_clean = Eigen::MatrixXd::Zero(500,1);

    int _index_for_array = 0;

    int _index_for_array_new = 0;

    bool _01_done = false;

    bool _02_done = false;

    bool _force_found = false;

    //low pass filter

    Eigen::Matrix<double, 3, 1> _tip_force_filtered = Eigen::MatrixXd::Zero(3,1);

    double _K;

    double _a;

    double _f_cutoff;

    double _t_camp;

    double _omega_0;
   
    int _M = 0;

    double _x_x[500];
    double _y_x[500];
      
    double _x_y[500];
    double _y_y[500];
      
    double _x_z[500];
    double _y_z[500];

    double _previous_time = 0.0;

    double _previuos_value = 0.0;

    //replanning strategy

    bool _force_reduced = false;

    bool _replanning_2_acceleration = false;

    double _gain = 70.0;

    double _force_mean_sum = 0.0;

    double _force_mean = 0.0;

    bool _impact = false;

    double _save_gain = 0.0;

    bool _gain_found = false;



};

#endif
