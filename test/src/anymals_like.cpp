// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "anymals_like.h"

QUADRUPED* quad;

PLANNING* plan;

OPTIMAL* opti;

//compute_estimation

std::ofstream f_ext_file("f_ext_file.txt");

std::ofstream f_gr_file("f_gr_file.txt");

//stand_phase

std::ofstream f_des_file("f_des_file.txt");

std::ofstream acc_des_file("acc_des_file.txt");

std::ofstream torque_des_file("torque_des_file.txt");

std::ofstream r_c_ref_file("r_c_ref.txt");

std::ofstream r_c_ref_dot_file("r_c_ref_dot.txt");

std::ofstream r_c_ref_dot_dot_file("r_c_ref_dot_dot.txt");

std::ofstream mean_file("mean_file.txt");


//get_raisim

std::ofstream com_rpy_file("com_rpy_file.txt");

std::ofstream forze_balla_file("forze_balla_file.txt");

std::ofstream joint_pos_file("joint_pos_file.txt");

std::ofstream tip_force_filt_file("tip_force_filt_file.txt");

//update

std::ofstream com_pos_file("com_pos_file.txt");

std::ofstream feet_pos_file("feet_pos_file.txt");


QUADRUPED::QUADRUPED() : _server(&_world){

}

int QUADRUPED::init_model(){
  
 
  model = &kinDynComp.model();

  kinDynComp.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);

  std::string path = "/home/fortunato/raisim_ws/raisimLib/rsc/a1/urdf/idyn/a1.urdf"; 

  iDynTree::ModelLoader mdlLoader;  
    
  bool ok = mdlLoader.loadModelFromFile(path); 

  std::string modelFile = "/home/fortunato/raisim_ws/raisimLib/rsc/a1/urdf/idyn/a1.urdf";

  if( !ok ){
    std::cerr << "KinDynComputationsWithEigen: impossible to load model from " << modelFile << std::endl;
    return EXIT_FAILURE;
  }

  ok = kinDynComp.loadRobotModel(mdlLoader.model());
 
  if( !ok ){
    std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
              << mdlLoader.model().toString() << std::endl;
    return EXIT_FAILURE;
  }
  
  //per plottare gli indici corrispondenti a ciascun link/giunto
  std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
            << mdlLoader.model().toString() << std::endl;


  _eig_mass_matrix=Eigen::MatrixXd::Identity(6+12, 6+12);

  _eig_jacobian=Eigen::MatrixXd::Zero(6, 6+12);

  _eig_com_jacobian=Eigen::MatrixXd::Zero(3, 6+12);

  _eig_robot_state.random();

  _dx = Eigen::MatrixXd::Zero(6, 6);

  _dmb = Eigen::MatrixXd::Zero(6, 6);

  _t_inv_der = Eigen::MatrixXd::Zero(18, 18);

  _j_st_bar_0 = Eigen::MatrixXd::Zero(12, 18);

  _j_st_c_bar_0 = Eigen::MatrixXd::Zero(12, 6);

  _j_st_j_bar_0 = Eigen::MatrixXd::Zero(12, 12);

  _j_st_bar_1 = Eigen::MatrixXd::Zero(6, 18);
  
  _j_st_c_bar_1 = Eigen::MatrixXd::Zero(6, 6);
  
  _j_st_j_bar_1 = Eigen::MatrixXd::Zero(6, 12);

  _j_sw_c_bar = Eigen::MatrixXd::Zero(6, 6);

  _j_st_dot_0 = Eigen::MatrixXd::Zero(12,18);
  
  _j_st_dot_1 = Eigen::MatrixXd::Zero(6,1);

  _j_st_dot=Eigen::MatrixXd::Zero(24,18);

  _j_sw_dot= Eigen::MatrixXd::Zero(6,1);

  _j_st_temp=Eigen::MatrixXd::Zero(12,18);

  _j_t_1_dot_temp=Eigen::MatrixXd::Zero(6,1);

  _j_t_2_dot_temp=Eigen::MatrixXd::Zero(6,1);

  _j_t_3_dot_temp=Eigen::MatrixXd::Zero(6,1);

  _j_t_4_dot_temp=Eigen::MatrixXd::Zero(6,1);

  _j_st_dot_temp = Eigen::MatrixXd::Zero(24,1);

  _Jdqd=Eigen::MatrixXd::Zero(24,1);

  _JdqdCOM=Eigen::MatrixXd::Zero(24,1);

  _JdqdCOM_lin=Eigen::MatrixXd::Zero(12,1);

  _B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
       Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
	     Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
	     Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);

  _bias_com=Eigen::MatrixXd::Zero(18,1);

  _yd_prev.resize(4);  
  _yw_prev.resize(4); 
  _w_prev.resize(4); 
  _ygamma_prev.resize(4);

  std::fill(_yd_prev.begin(), _yd_prev.end(), Eigen::Matrix<double,12,1>::Zero()); 
  std::fill(_yw_prev.begin(), _yw_prev.end(), Eigen::Matrix<double,12,1>::Zero());
  std::fill(_w_prev.begin(), _w_prev.end(), Eigen::Matrix<double,12,1>::Zero());
  std::fill(_ygamma_prev.begin(), _ygamma_prev.end(), Eigen::Matrix<double,12,1>::Zero());

  _yd.resize(4);  
  _yw.resize(4); 
  _w.resize(4); 
  _ygamma.resize(4);

  std::fill(_yd.begin(), _yd.end(), Eigen::Matrix<double,12,1>::Zero()); 
  std::fill(_yw.begin(), _yw.end(), Eigen::Matrix<double,12,1>::Zero());
  std::fill(_w.begin(), _w.end(), Eigen::Matrix<double,12,1>::Zero());
  std::fill(_ygamma.begin(), _ygamma.end(), Eigen::Matrix<double,12,1>::Zero());

  _jnt_torques_star=Eigen::MatrixXd::Zero(12,1);

  _f_ext=Eigen::MatrixXd::Zero(12,1);

  _coeffs<<1000, 10000, 100000, 1;


  std::vector<Eigen::Matrix<double,12,12>> _k(4);

  for (int i=0; i<4; i++){
    _k[i]=_coeffs[i]*Eigen::Matrix<double,12,12>::Identity();
  }

  _k_1=(Eigen::Matrix<double,12,12>::Identity()+_k[0]*_T).inverse()*_k[0];

  _k_2=(Eigen::Matrix<double,12,12>::Identity()+_k[1]*_T).inverse()*_k[1];

  _k_3=(Eigen::Matrix<double,12,12>::Identity()+_k[2]*_T).inverse()*_k[2];

  //questa modifica sulla contact frame mi serve per ottenere una componente x con il segno giusto (vedi documentazione RaiSim)
  _contact_frame=Eigen::MatrixXd::Identity(3,3);
  _contact_frame(0,0)=-1.0;

  _theta_i = atan(_mu);

  _z_i << 0.0, 0.0, 1.0;

  //low pass filter
  _K = 1.1;

  _f_cutoff = 50;

  _omega_0 = 2*3.14*_f_cutoff;

  _t_camp = 0.001;

  _a = 2/(_omega_0*_t_camp);



  

  return 0;
    
}

void QUADRUPED::set_anymalC(raisim::ArticulatedSystem* data){
  _anymalC=data;
}

void QUADRUPED::set_balla(raisim::SingleBodyObject* data){
  _balla=data;
}

void QUADRUPED::get_raisim(int flag){

  //joint position

  raisim::VecDyn vec_sample=_anymalC->getGeneralizedCoordinate();
  Vector<double, 20> vec_sample_eig_pos=vec_sample.e(); //(in caso di linear actuator sostituire 19 con 20)


  //FR
  _eig_robot_state.jointPos(3,0)=vec_sample_eig_pos[7];
  _eig_robot_state.jointPos(4,0)=vec_sample_eig_pos[8];
  _eig_robot_state.jointPos(5,0)=vec_sample_eig_pos[9];

  //FL
  _eig_robot_state.jointPos(1,0)=vec_sample_eig_pos[10];
  _eig_robot_state.jointPos(8,0)=vec_sample_eig_pos[11];
  _eig_robot_state.jointPos(9,0)=vec_sample_eig_pos[12];

  //RR
  _eig_robot_state.jointPos(2,0)=vec_sample_eig_pos[13];
  _eig_robot_state.jointPos(6,0)=vec_sample_eig_pos[14];
  _eig_robot_state.jointPos(7,0)=vec_sample_eig_pos[15];

  //RL
  _eig_robot_state.jointPos(0,0)=vec_sample_eig_pos[16];
  _eig_robot_state.jointPos(10,0)=vec_sample_eig_pos[17];
  _eig_robot_state.jointPos(11,0)=vec_sample_eig_pos[18];

  _drill_rig_pos=vec_sample_eig_pos[19]; //linear actuator

  joint_pos_file<<_eig_robot_state.jointPos(3,0)<<" "<<_eig_robot_state.jointPos(4,0)<<" "<<_eig_robot_state.jointPos(5,0)<<" "<<_eig_robot_state.jointPos(1,0)<<" "<<_eig_robot_state.jointPos(8,0)<<" "<<_eig_robot_state.jointPos(9,0)<<" "<<_eig_robot_state.jointPos(2,0)<<" "<<_eig_robot_state.jointPos(6,0)<<" "<<_eig_robot_state.jointPos(7,0)<<" "<<_eig_robot_state.jointPos(0,0)<<" "<<_eig_robot_state.jointPos(10,0)<<" "<<_eig_robot_state.jointPos(11,0)<<" "<<_world.getWorldTime()<<"\n";
  joint_pos_file.flush();

  //joint velocity

  raisim::VecDyn vec_sample_vel=_anymalC->getGeneralizedVelocity();
  Vector<double, 18> vec_sample_eig_vel=vec_sample_vel.e();

  _eig_robot_state.jointVel(3,0)=vec_sample_eig_vel[6];
  _eig_robot_state.jointVel(4,0)=vec_sample_eig_vel[7];
  _eig_robot_state.jointVel(5,0)=vec_sample_eig_vel[8];

  _eig_robot_state.jointVel(1,0)=vec_sample_eig_vel[9];
  _eig_robot_state.jointVel(8,0)=vec_sample_eig_vel[10];
  _eig_robot_state.jointVel(9,0)=vec_sample_eig_vel[11];

  _eig_robot_state.jointVel(2,0)=vec_sample_eig_vel[12];
  _eig_robot_state.jointVel(6,0)=vec_sample_eig_vel[13];
  _eig_robot_state.jointVel(7,0)=vec_sample_eig_vel[14];

  _eig_robot_state.jointVel(0,0)=vec_sample_eig_vel[15];
  _eig_robot_state.jointVel(10,0)=vec_sample_eig_vel[16];
  _eig_robot_state.jointVel(11,0)=vec_sample_eig_vel[17]; 

  //base position

  raisim::Vec<3> vec_sample_base=_anymalC->getBasePosition();
  Eigen::Vector3d vec_sample_base_eig=vec_sample_base.e();

  _pb=vec_sample_base_eig;

  //base orientation

  raisim::Mat<3, 3> mat_sample_base=_anymalC->getBaseOrientation();
  Eigen::Matrix3d R=mat_sample_base.e();

  //attenzione: raisim segue una convenzione diversa per il quaternione, valutare la possibilitá 
  //di cambiare la modalitá di calcolo del quaternione con metodo fornito da raisim

  //Eigen::Quaterniond q(R);
  //_base_quaternion(0,0)=q.x();
  //_base_quaternion(1,0)=q.y();
  //_base_quaternion(2,0)=q.z();
  //_base_quaternion(3,0)=q.w();

  raisim::Vec<4> quaternion;
  _anymalC->getBaseOrientation(quaternion);
  Vector<double,4> quaternion_eig=quaternion.e();
  _base_quaternion(0,0)=quaternion[1];
  _base_quaternion(1,0)=quaternion[2];
  _base_quaternion(2,0)=quaternion[3];
  _base_quaternion(3,0)=quaternion[0];



  tf::Quaternion q_tf(_base_quaternion(0,0), _base_quaternion(1,0), _base_quaternion(2,0),  _base_quaternion(3,0));

  q_tf.normalize();

  tf::Matrix3x3(q_tf).getRPY(_roll, _pitch, _yaw);

  com_rpy_file<<_roll<<" "<<_pitch<<" "<<_yaw<<" "<<_world.getWorldTime()<<" "<<"\n";
  com_rpy_file.flush();

    
  //non funziona a causa della dipendenza eigen_conversions
  tf::Matrix3x3 m(q_tf);  
  //Eigen::Matrix3d R;
  tf::matrixTFToEigen(m,R);

  _eig_robot_state.world_H_base<<R(0,0), R(0,1), R(0,2), _pb[0],
                                 R(1,0), R(1,1), R(1,2), _pb[1],
                                 R(2,0), R(2,1), R(2,2), _pb[2],
                                 0,      0,      0,      1;

  
  //base velocity (ho sfruttato il vettore delle velocitá generalizzate)
  _eig_robot_state.baseVel(0,0)=vec_sample_eig_vel[0];
  _eig_robot_state.baseVel(1,0)=vec_sample_eig_vel[1];
  _eig_robot_state.baseVel(2,0)=vec_sample_eig_vel[2];
  _eig_robot_state.baseVel(3,0)=vec_sample_eig_vel[3];
  _eig_robot_state.baseVel(4,0)=vec_sample_eig_vel[4];
  _eig_robot_state.baseVel(5,0)=vec_sample_eig_vel[5];

 
  //ground reaction forces by raisim

  auto footIndex_RH = _anymalC->getBodyIdx("RR_calf");
  for(auto& contact: _anymalC->getContacts()) {
    if (contact.skip()) continue; /// if the contact is internal, one contact point is set to 'skip'
    if ( footIndex_RH == contact.getlocalBodyIndex() ) {
      Eigen::Matrix<double,3,1> f_gr_RH=contact.getImpulse().e()/0.001;
      
      
      //assegno il valore precedente nel caso mi venisse restituita un forza nulla oppure una forza maggiore di 300 N
      if((flag==0 || flag==2)){
        if(f_gr_RH == Eigen::MatrixXd::Zero(3,1) || f_gr_RH(2,0)>300){
          _f_gr.block(3,0,3,1)=_f_gr_star.block(3,0,3,1);
        }else{
          _f_gr.block(3,0,3,1)=_contact_frame*f_gr_RH;
        }
      }

      
      //per stampare la matrice di rotazione
      //std::cerr<<"contact frame RH raisim: "<<std::endl;
      //std::cerr<<contact.getContactFrame().e()<<std::endl;   
      
    }
  }

  auto footIndex_LH = _anymalC->getBodyIdx("RL_calf");
  for(auto& contact: _anymalC->getContacts()) {
    if (contact.skip()) continue; /// if the contact is internal, one contact point is set to 'skip'
    if ( footIndex_LH == contact.getlocalBodyIndex() ) {
      Eigen::Matrix<double,3,1>  f_gr_LH=contact.getImpulse().e()/0.001;


      if((flag==0 || flag==1)){
        if(f_gr_LH == Eigen::MatrixXd::Zero(3,1) || f_gr_LH(2,0)>300){
          _f_gr.block(9,0,3,1)=_f_gr_star.block(9,0,3,1);
        }else{
          _f_gr.block(9,0,3,1)=_contact_frame*f_gr_LH;
        }
      }
    
    }
  }

  
  auto footIndex_RF = _anymalC->getBodyIdx("FR_calf");
  for(auto& contact: _anymalC->getContacts()) {
    if (contact.skip()) continue; /// if the contact is internal, one contact point is set to 'skip'
    if ( footIndex_RF == contact.getlocalBodyIndex() ) {
      Eigen::Matrix<double,3,1>  f_gr_RF=contact.getImpulse().e()/0.001;


      if((flag==0 || flag==1)){
        
        if(f_gr_RF == Eigen::MatrixXd::Zero(3,1) || f_gr_RF(2,0)>300){
          _f_gr.block(0,0,3,1)=_f_gr_star.block(0,0,3,1);
        }else{
          _f_gr.block(0,0,3,1)=_contact_frame*f_gr_RF;
        }
      }
      
    }
  }
  

  auto footIndex_LF = _anymalC->getBodyIdx("FL_calf");
  for(auto& contact: _anymalC->getContacts()) {
    if (contact.skip()) continue; /// if the contact is internal, one contact point is set to 'skip'
    if ( footIndex_LF == contact.getlocalBodyIndex() ) {
      Eigen::Matrix<double,3,1>  f_gr_LF=contact.getImpulse().e()/0.001;

      if((flag==0 || flag==2)){
        if(f_gr_LF == Eigen::MatrixXd::Zero(3,1) || f_gr_LF(2,0)>300){
          _f_gr.block(6,0,3,1)=_f_gr_star.block(6,0,3,1);
        }else{
          _f_gr.block(6,0,3,1)=_contact_frame*f_gr_LF;
        }
      }
 
    }
  }

  auto rig_index = _anymalC->getBodyIdx("hole_saw"); //getBodyIdx
  for(auto& contact: _anymalC->getContacts()) {
    if (contact.skip()) continue; /// if the contact is internal, one contact point is set to 'skip'
    if ( rig_index == contact.getlocalBodyIndex() ) {

      _impact = true;

      _tip_force=contact.getImpulse().e()/0.001;

      forze_balla_file<<_tip_force(2,0)<<" "<<-_tip_force(1,0)<<" "<<-_tip_force(0,0)<<" "<<_world.getWorldTime()<<"\n";
      forze_balla_file.flush();

      
      _x_x[_M] = _tip_force(2,0);
        
      _x_y[_M] = -_tip_force(1,0);

      _x_z[_M] = -_tip_force(0,0);

      _M++;

      /////////////////////////////////////////////////
      //inizializzo i campioni all'istante successivo

      _x_x[_M] = 0.0;
        
      _x_y[_M] = 0.0;

      _x_z[_M] = 0.0;

      _y_x[_M] = 0.0;
        
      _y_y[_M] = 0.0;

      _y_z[_M] = 0.0;

      ///////////////////////////////////////////////

      low_pass_filter(_x_x, _y_x, _M);

      low_pass_filter(_x_y, _y_y, _M);

      low_pass_filter(_x_z, _y_z, _M);

      tip_force_filt_file<<_y_x[_M -1]<<" "<<_y_y[_M -1]<<" "<<_y_z[_M -1]<<" "<<_world.getWorldTime()<<"\n";
      tip_force_filt_file.flush();

      //approccio con media

      _tip_force_array(_index_for_array, 0) = _y_x[_M -1];
      _tip_force_array(_index_for_array, 1) = _world.getWorldTime();


      _index_for_array++;
      
    }
  }

  
 
}

Eigen::Matrix4d QUADRUPED::get_world_base(){
  return _eig_robot_state.world_H_base;
}

void QUADRUPED::compute_m22_bar(){

  _pbc = _pc - _pb;
    
  _s<<0.0, -_pbc[2], _pbc[1],
      _pbc[2], 0.0, -_pbc[0],
      -_pbc[1], _pbc[0], 0.0;

  _x.block(0,0,3,3)=  Eigen::MatrixXd::Identity(3, 3);
  _x.block(0,3,3,3)= _s.transpose();
  _x.block(3,0,3,3)= Eigen::MatrixXd::Zero(3, 3);
  _x.block(3,3,3,3)= Eigen::MatrixXd::Identity(3, 3);

    
  _m11=_eig_mass_matrix.block(0,0,6,6);

  _m12=_eig_mass_matrix.block(0,6,6,12);

  _j_s=_x*(_m11.inverse()*_m12);


  //Some elements for centroidal dynamics

  _t_bar.block(0,0,6,6)= _x;
  _t_bar.block(0,6,6,12)= _j_s;
  _t_bar.block(6,0,12,6)=   Eigen::MatrixXd::Zero(12, 6);
  _t_bar.block(6,6,12,12)=   Eigen::MatrixXd::Identity(12, 12);   
    
  _m_bar=_t_bar.transpose().inverse()*_eig_mass_matrix*_t_bar.inverse();

  _m22_bar=_m_bar.block(6,6,12,12);   

}

void QUADRUPED::compute_c2_bar(){

  /////////COMPUTE C///////////////////

  iDynTree::FreeFloatingGeneralizedTorques bias_force(* model); 
  kinDynComp.generalizedBiasForces(bias_force);
     
  _h<<iDynTree::toEigen(bias_force.baseWrench()),
      iDynTree::toEigen(bias_force.jointTorques());

  iDynTree::FreeFloatingGeneralizedTorques gravity_force(* model); 
  kinDynComp.generalizedGravityForces(gravity_force); 

  _g<<iDynTree::toEigen(gravity_force.baseWrench()),
     iDynTree::toEigen(gravity_force.jointTorques());
    
  _c=(_h - _g)*_v_pseudo;

  ///////////////////////////////////////////////

  //////////COMPUTE T_inv_der/////////////////
    
  Eigen::Matrix<double,6,1> pb_dot_temp=iDynTree::toEigen(kinDynComp.getBaseTwist()); //getBaseTwist mi dà 6 componenti, ma a me ne servono solo 3 per avere consistenza dimensionale con _pc_dot

  _pb_dot[0]=pb_dot_temp(0,0);
  _pb_dot[1]=pb_dot_temp(1,0);
  _pb_dot[2]=pb_dot_temp(2,0);    

  _pbc_dot = _pc_dot - _pb_dot;

  _robot_mass=model->getTotalMass();

  _m_dr = _robot_mass*_pbc_dot;

  _s_dot<<0.0, -_pbc_dot[2], _pbc_dot[1],
          _pbc_dot[2], 0.0, -_pbc_dot[0],
          -_pbc_dot[1], _pbc_dot[0], 0.0;
  
  _dx.block(0,3,3,3)= _s_dot.transpose(); 

  _s_mdr<<0.0, -_m_dr[2], _m_dr[1],
          _m_dr[2], 0.0, -_m_dr[0],
          -_m_dr[1], _m_dr[0], 0.0;

      
  _dmb.block(0,3,3,3)= _s_mdr.transpose();
  _dmb.block(3,0,3,3)= _s_mdr;   
    
  _dmb_1=(_m11.transpose().inverse()*_dmb.transpose()).transpose();

  _dmb_2=-_m11.inverse()*_dmb_1;

  _dj_s=_dx*(_m11.inverse()*_m12)+_x*_dmb_2*_m12;

  _t_inv_der.block(0,3,3,3)=_s_dot;
  _t_inv_der.block(0,6,3,12)= -_dj_s.block(0,0,3,12);

  //////////////////////////////////////////////

}

int QUADRUPED::compute_j_st(int flag){

  //per maggiori dettagli consulta model_anymalC.txt sul Desktop
  //rispetto al dogbot dove la sequenza é BL, BR; FL, FR
  //nel caso di anymal la sequenza é RH, LH, RF, LF
  //per A1 la sequenza é FR, RR, FL, RL

  bool ok = kinDynComp.getFrameFreeFloatingJacobian(10, iDynTree::make_matrix_view(_j_t_1)); //linear actuator 10 al posto di 11

  if (!ok){
    std::cerr << "Wrong frame index or wrong size passed to KinDynComputations::getFrameFreeFloatingJacobian" << std::endl;
    return EXIT_FAILURE;
  }

  ok = kinDynComp.getFrameFreeFloatingJacobian(13, iDynTree::make_matrix_view(_j_t_2)); //linear actuator 13 al posto di 14

  if (!ok){
    std::cerr << "Wrong frame index or wrong size passed to KinDynComputations::getFrameFreeFloatingJacobian" << std::endl;
    return EXIT_FAILURE;
  }

  ok = kinDynComp.getFrameFreeFloatingJacobian(16, iDynTree::make_matrix_view(_j_t_3)); //linear actuator 16 al posto di 17

  if (!ok){
    std::cerr << "Wrong frame index or wrong size passed to KinDynComputations::getFrameFreeFloatingJacobian" << std::endl;
    return EXIT_FAILURE;
  }

  ok = kinDynComp.getFrameFreeFloatingJacobian(19, iDynTree::make_matrix_view(_j_t_4)); //linear actuator 19 al posto di 20

  if (!ok){
    std::cerr << "Wrong frame index or wrong size passed to KinDynComputations::getFrameFreeFloatingJacobian" << std::endl;
    return EXIT_FAILURE;
  }

  

  ///////////////////CONTACT JACOBIAN (ONLY THE FIRST 3 COMPONENTS)//////////////////////
    
  _j.block(0,0,6,18)=_j_t_1;
  _j.block(6,0,6,18)=_j_t_2;
  _j.block(12,0,6,18)=_j_t_3;
  _j.block(18,0,6,18)=_j_t_4;

  _j_st_temp.block(0,0,3,6+12)=_j_t_1.block(0,0,3,18);
  _j_st_temp.block(3,0,3,6+12)=_j_t_2.block(0,0,3,18);
  _j_st_temp.block(6,0,3,6+12)=_j_t_3.block(0,0,3,18);
  _j_st_temp.block(9,0,3,6+12)=_j_t_4.block(0,0,3,18);

  _j_st_0=_j_st_temp;

  if(flag==0){  
    _j_st_0=_j_st_temp;                   
  }    

  if(flag==1 || flag==2){

    if(flag==1){
      _stl1=0;
      _swl1=3;
      _swl2=6;
      _stl2=9;
    }
      
    if(flag==2){
      _swl1=0;
      _stl1=3;
      _stl2=6;
      _swl2=9;
    }
      
       
    _j_st_1.block(0,0,3,18)=_j_st_temp.block(_stl1,0,3,18);
    _j_st_1.block(3,0,3,18)=_j_st_temp.block(_stl2,0,3,18);
      
    _j_sw.block(0,0,3,18)=_j_st_temp.block(_swl1,0,3,18);
    _j_sw.block(3,0,3,18)=_j_st_temp.block(_swl2,0,3,18);     
  
    _j_st_1_new.block(_swl1,0,3,18)=Eigen::MatrixXd::Zero(3,18);
    _j_st_1_new.block(_stl1,0,3,18)=_j_st_temp.block(_stl1,0,3,18);
    _j_st_1_new.block(_swl2,0,3,18)=Eigen::MatrixXd::Zero(3,18);
    _j_st_1_new.block(_stl2,0,3,18)=_j_st_temp.block(_stl2,0,3,18);
          
  }

  _j_t_1_dot_temp = iDynTree::toEigen(kinDynComp.getFrameBiasAcc(10));
  _j_t_2_dot_temp = iDynTree::toEigen(kinDynComp.getFrameBiasAcc(13));    
  _j_t_3_dot_temp = iDynTree::toEigen(kinDynComp.getFrameBiasAcc(16));    
  _j_t_4_dot_temp = iDynTree::toEigen(kinDynComp.getFrameBiasAcc(19));  

  ////////////////////////////////////////////////     
    
  _Jdqd.block(0,0,6,1)=_j_t_1_dot_temp;
  _Jdqd.block(6,0,6,1)=_j_t_2_dot_temp;
  _Jdqd.block(12,0,6,1)=_j_t_3_dot_temp;
  _Jdqd.block(18,0,6,1)=_j_t_4_dot_temp;
    
  _JdqdCOM=_Jdqd+_j*_t_inv_der*_v_c;
  _JdqdCOM_lin= Eigen::MatrixXd::Zero(12,1);
  _JdqdCOM_lin=_B*_JdqdCOM;

  _j_st_dot_0.block(0,0,3,1)=_JdqdCOM_lin.block(0,0,3,1);
  _j_st_dot_0.block(3,0,3,1)=_JdqdCOM_lin.block(3,0,3,1);
  _j_st_dot_0.block(6,0,3,1)=_JdqdCOM_lin.block(6,0,3,1);
  _j_st_dot_0.block(9,0,3,1)=_JdqdCOM_lin.block(9,0,3,1);

  ////////////////////////////////////////////////
    
  if(flag==1 || flag==2){
      
    _j_st_dot_1.block(0,0,3,1)=_j_st_dot_0.block(_stl1,0,3,1);
    _j_st_dot_1.block(3,0,3,1)=_j_st_dot_0.block(_stl2,0,3,1);       
    _j_sw_dot.block(0,0,3,1)=_j_st_dot_0.block(_swl1,0,3,1);
    _j_sw_dot.block(3,0,3,1)=_j_st_dot_0.block(_swl2,0,3,1);
              
  }

  //////////////////////////////////////////////////////// 

  return 0;

}

void QUADRUPED::compute_j_st_j_bar(int flag){

  ////////COMPUTE J_ST_J_BAR////////////////

  _j_st_bar_0=_j_st_0*_t_bar.inverse();

  _j_st_j_bar_0=_j_st_bar_0.block(0,6,12,12); //calcola questo termine sempre, mi serve per lo stimatore

  if(flag==0){
    _j_st_c_bar_0=_j_st_bar_0.block(0,0,12,6);
  }

  if(flag==1 || flag==2){
      //_j_st_bar_1=_j_st_1*_t_bar.inverse();
      _j_st_bar_1.block(0,0,3,18)=_j_st_bar_0.block(_stl1,0,3,18);
      _j_st_bar_1.block(3,0,3,18)=_j_st_bar_0.block(_stl2,0,3,18);
      _j_st_j_bar_1=_j_st_bar_1.block(0,6,6,12);
      _j_st_c_bar_1=_j_st_bar_1.block(0,0,6,6);
      //_j_sw_bar=_j_sw*_t_bar.inverse();
      _j_sw_bar.block(0,0,3,18)=_j_st_bar_0.block(_swl1,0,3,18);
      _j_sw_bar.block(3,0,3,18)=_j_st_bar_0.block(_swl2,0,3,18);
      _j_sw_j_bar=_j_sw_bar.block(0,6,6,12);
      _j_sw_c_bar=_j_sw_bar.block(0,0,6,6);
      _j_st_bar_1_new=_j_st_1_new*_t_bar.inverse();
      _j_st_j_bar_1_new=_j_st_bar_1_new.block(0,6,12,12);
  }

  //////////////////////////////////////////

  _j_bar=_j*_t_bar.inverse();
}

int QUADRUPED::update(int flag){

  

  iDynTree::Transform world_H_base;
  iDynTree::fromEigen(world_H_base,quad->get_world_base());

  Eigen::Vector3d worldeigen=toEigen(world_H_base.getPosition());
  while(worldeigen==Eigen::Vector3d::Zero()){
    worldeigen=toEigen(world_H_base.getPosition());
  }

  //Now we create the KinDynComputations class, so we can set the state (TRAVERSARO)
  kinDynComp.setRobotState(iDynTree::make_matrix_view(_eig_robot_state.world_H_base),
                           iDynTree::make_span(_eig_robot_state.jointPos),
                           iDynTree::make_span(_eig_robot_state.baseVel),
                           iDynTree::make_span(_eig_robot_state.jointVel),
                           iDynTree::make_span(_eig_robot_state.gravity));

  //Once we called the setRobotState, we can call all the methods of KinDynComputations

  /////////////////////////////////////////////////////////////////////////
  //controllo se idyntree non funziona

  _pc=toEigen(kinDynComp.getCenterOfMassPosition());
  while(_pc==Eigen::Vector3d::Zero()){
    _pc=toEigen(kinDynComp.getCenterOfMassPosition());
  }
  
  //questo if fa bloccare il programma e l'ho tolto (per una verifica anche sulla velocità, ma la velocità é 0 all'inizio!)
  //if(_index>=2){
  //    worldeigen=toEigen(kinDynComp.getCenterOfMassVelocity());
  //    while (worldeigen==Eigen::Vector3d::Zero()){
  //        //iDynTree::fromEigen(world_H_base,quad->get_world_base());
  //        worldeigen=toEigen(kinDynComp.getCenterOfMassVelocity());
  //    }
  //}

  /////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////
  //mass matrix

  bool ok = kinDynComp.getFreeFloatingMassMatrix(iDynTree::make_matrix_view(_eig_mass_matrix));

  if (!ok){
    std::cerr << "Matrix of wrong size passed to KinDynComputations::getFreeFloatingMassMatrix" << std::endl;
    //return EXIT_FAILURE;
  }
  ////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////
  //UPDATE RC

  //_pc=iDynTree::toEigen(kinDynComp.getCenterOfMassPosition());

  for(int i=0; i<3; i++){
    _r_c(i,0)=_pc[i];
  }

  com_pos_file<<_r_c(0,0)<<" "<<_r_c(1,0)<<" "<<_r_c(2,0)<<" "<<_world.getWorldTime()<<"\n";
  com_pos_file.flush();

  feet_pos_file<<quad->getFLpos()(0,0)<<" "<<quad->getFLpos()(1,0)<<" "<<quad->getFRpos()(0,0)<<" "<<quad->getFRpos()(1,0)<<" "<<quad->getBLpos()(0,0)<<" "<<quad->getBLpos()(1,0)<<" "<<quad->getBRpos()(0,0)<<" "<<quad->getBRpos()(1,0)<<" "<<_world.getWorldTime()<<"\n";
  feet_pos_file.flush();

  /////////////
  //aggiustamenti angolo
  /*if(_ang_prec<0.0 && std::abs(_ang_prec)>3.0){
     _ang_was_negative=true;
  }

  if(_ang_prec>0.0  && std::abs(_ang_prec)>3.0){
      _ang_was_positive=true;
  }*/
  /////////////


  iDynTree::Vector3 base_angle=world_H_base.getRotation().asRPY();
  Eigen::Vector3d base_angle_eig = toEigen(base_angle);

  //////////////
  //aggiustamenti angolo
  /*_ang_prec=base_angle_eig[2];

  if(base_angle_eig[2]>0.0 && _ang_was_negative){
      base_angle_eig[2]=base_angle_eig[2]-2*M_PI;
  }

  if(base_angle_eig[2]<0.0 && _ang_was_positive){
      base_angle_eig[2]=base_angle_eig[2]+2*M_PI;
  }

  if(std::abs(base_angle_eig[2])>=2*3.14){
      base_angle_eig[2]=0.0;
      _ang_was_negative=false;
      _ang_was_positive=false;
  }*/
  /////////////

  _r_c(3,0)=base_angle_eig[0];
  _r_c(4,0)=base_angle_eig[1];
  _r_c(5,0)=base_angle_eig[2];

  ///////////////////////////////////////////////////////

  //////////////////////////////////////////////////////
  //UPDATE RC_DOT

  _pc_dot=iDynTree::toEigen(kinDynComp.getCenterOfMassVelocity());
    
  for(int i=0; i<3; i++){
    _r_c_dot(i,0)=_pc_dot(i);
  }


  for(int i=3; i<6; i++){
    _r_c_dot(i,0)=_eig_robot_state.baseVel[i];
  }


  ////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////
  //UPDATE V_C

  for(int i=0; i<3; i++){
    _v_c(i,0)=_pc_dot[i];
  }

  for(int i=3; i<6;i++){
    _v_c(i,0)=_eig_robot_state.baseVel[i];
  }

  for(int i=6; i<6+12; i++){
    _v_c(i,0)=_eig_robot_state.jointVel(i-6,0);
  }

  ////////////////////////////////////////////////////////////

  //Stacking v
  _eig_robot_state.v.resize(_eig_robot_state.baseVel.size() + _eig_robot_state.jointVel.size(),1);
  _eig_robot_state.v.block(0,0,6,1)=_eig_robot_state.baseVel;
  _eig_robot_state.v.block(6,0,12,1)=_eig_robot_state.jointVel;

  //Pseudo-inverse for the velocity vector v (needed to compute the Coriolis matrix)
  _v_pseudo.resize(1, _eig_robot_state.baseVel.size() + _eig_robot_state.jointVel.size());
  _v_pseudo = Eigen::MatrixXd::Zero(1, _eig_robot_state.baseVel.size() + _eig_robot_state.jointVel.size());
  _v_pseudo = _eig_robot_state.v.completeOrthogonalDecomposition().pseudoInverse();

  compute_m22_bar();

  compute_c2_bar();
    
  compute_j_st(flag);

  compute_j_st_j_bar(flag);

  _bias_com=_t_bar.transpose().inverse()*_h+_t_bar.transpose().inverse()*_eig_mass_matrix*_t_inv_der*_v_c;

    
  return 0;

}

int QUADRUPED::stand_phase(){  

   
  while( ((_world.getWorldTime()-_begin) < _duration && !_do_replanning) ){  

    double t = _world.getWorldTime()-_begin; 

    _r_c_ref<<plan->get_solution().base_linear_->GetPoint(t).p(), plan->get_solution().base_angular_->GetPoint(t).p();
    
    _r_c_ref_dot<<plan->get_solution().base_linear_->GetPoint(t).v(), plan->get_solution().base_angular_->GetPoint(t).v();
    
    _r_c_ref_dot_dot<<plan->get_solution().base_linear_->GetPoint(t).a(), plan->get_solution().base_angular_->GetPoint(t).a();

    //replanning strategy with treshold on acceleration (prima_2, prima_3, prima_4, prima_5)

    //if( (_r_c_ref_dot_dot(0,0) < 4.374) && _force_found){
    //  _duration = t;
    //}
    

    r_c_ref_file<<_r_c_ref(0,0)<<" "<<_r_c_ref(1,0)<<" "<<_r_c_ref(2,0)<<" "<<_r_c_ref(3,0)<<" "<<_r_c_ref(4,0)<<" "<<_r_c_ref(5,0)<<" "<<_world.getWorldTime()<<"\n";
    r_c_ref_file.flush();
    

    r_c_ref_dot_file<<_r_c_ref_dot(0,0)<<" "<<_r_c_ref_dot(1,0)<<" "<<_r_c_ref_dot(2,0)<<" "<<_r_c_ref_dot(3,0)<<" "<<_r_c_ref_dot(4,0)<<" "<<_r_c_ref_dot(5,0)<<" "<<_world.getWorldTime()<<"\n";
    r_c_ref_dot_file.flush();

    r_c_ref_dot_dot_file<<_r_c_ref_dot_dot(0,0)<<" "<<_r_c_ref_dot_dot(1,0)<<" "<<_r_c_ref_dot_dot(2,0)<<" "<<_r_c_ref_dot_dot(3,0)<<" "<<_r_c_ref_dot_dot(4,0)<<" "<<_r_c_ref_dot_dot(5,0)<<" "<<_world.getWorldTime()<<"\n";
    r_c_ref_dot_dot_file.flush();
    

    get_raisim(0);

    update(0);

    //////////////////ESTIMATOR////////////////////////

    if(_replanning_done){
      raisim::VecDyn vec_sample_force=_anymalC->getGeneralizedForce();
      Vector<double, 18> vec_sample_eig_force=vec_sample_force.e();
      _jnt_torques_star=vec_sample_eig_force.block(6,0,12,1);
      _replanning_done=false;
    }

    compute_estimation(0);

    ////////////////////////////////////////////////////

    ////////////COMPUTE F_GR_STAR & Q_DOT_DOT_DES//////////  

    opti->compute_optimization(0);
 
    //////////////////////////////////////////////////////

    //////////COMPUTE JNT_TORQUES_STAR///////////////////

    _jnt_torques_star=_m22_bar*_q_dot_dot_des + _bias_com.block(6,0,12,1) - _j_st_j_bar_0.transpose()*_f_gr_star;

    f_des_file<<_f_gr_star(0,0)<<" "<<_f_gr_star(1,0)<<" "<<_f_gr_star(2,0)<<" "<<" "<<_f_gr_star(3,0)<<" "<<_f_gr_star(4,0)<<" "<<_f_gr_star(5,0)<<" "<<_f_gr_star(6,0)<<" "<<_f_gr_star(7,0)<<" "<<_f_gr_star(8,0)<<" "<<_f_gr_star(9,0)<<" "<<_f_gr_star(10,0)<<" "<<_f_gr_star(11,0)<<" "<<_world.getWorldTime()<<"\n";
    f_des_file.flush();

    acc_des_file<<_q_dot_dot_des(0,0)<<" "<<_q_dot_dot_des(1,0)<<" "<<_q_dot_dot_des(2,0)<<" "<<_q_dot_dot_des(3,0)<<" "<<_q_dot_dot_des(4,0)<<" "<<_q_dot_dot_des(5,0)<<" "<<_q_dot_dot_des(6,0)<<" "<<_q_dot_dot_des(7,0)<<" "<<_q_dot_dot_des(8,0)<<" "<<_q_dot_dot_des(9,0)<<" "<<_q_dot_dot_des(10,0)<<" "<<_q_dot_dot_des(11,0)<<" "<<_world.getWorldTime()<<"\n";
    acc_des_file.flush();

    torque_des_file<<_jnt_torques_star(0,0)<<" "<<_jnt_torques_star(1,0)<<" "<<_jnt_torques_star(2,0)<<" "<<_jnt_torques_star(3,0)<<" "<<_jnt_torques_star(4,0)<<" "<<_jnt_torques_star(5,0)<<" "<<_jnt_torques_star(6,0)<<" "<<_jnt_torques_star(7,0)<<" "<<_jnt_torques_star(8,0)<<" "<<_jnt_torques_star(9,0)<<" "<<_jnt_torques_star(10,0)<<" "<<_jnt_torques_star(11,0)<<" "<<_world.getWorldTime()<<"\n";
    torque_des_file.flush();

    ////////////////////////////////////////////////////

    _torque_array[12]=_jnt_torques_star(2,0); //right hind hip
    _torque_array[13]=_jnt_torques_star(6,0); //right hind thigh
    _torque_array[14]=_jnt_torques_star(7,0); //right hind shank

    _torque_array[15]=_jnt_torques_star(0,0); //left hind hip
    _torque_array[16]=_jnt_torques_star(10,0); //left hind thigh
    _torque_array[17]=_jnt_torques_star(11,0); //left hind shank

    _torque_array[6]=_jnt_torques_star(3,0); //right front hip
    _torque_array[7]=_jnt_torques_star(4,0); //right front thigh
    _torque_array[8]=_jnt_torques_star(5,0); //right front shank

    _torque_array[9]=_jnt_torques_star(1,0); //left front hip
    _torque_array[10]=_jnt_torques_star(8,0); //left front thigh
    _torque_array[11]=_jnt_torques_star(9,0); //left front shank

    _torque_array[18] = 0.001; //low torque for the rotation of the 'fake' revolute joint

    _anymalC->setGeneralizedForce(_torque_array); //setto le coppie 

    /////////////////////////////////////////////////////

    
    //strategia di controllo sezna soglia massima spostamento x
    /*compute_robustness();

    if( t>0.02 ){

      _gradient = ( _robustness_array(_index_rob_array-1, 0) - _robustness_array(0,0) )/t;

      if(_gradient < 0.0){
        _do_replanning = true;
      }else{
        _do_replanning = false;
      }
      
      _robustness_array = Eigen::MatrixXd::Zero(300,1);

      _index_rob_array = 0;

    }*/

    /*if( t >= 0.1 && !_01_done && !_force_found){

      //calcolo media

      double _force_mean = 0.0;

      for(int i=0; i<_index_for_array; i++){

        if( _tip_force_array(i,0) < 200 && _tip_force_array(i,0) >= 0.0){
          _force_mean_sum += _tip_force_array(i,0);
        }else{
          _force_mean_sum += 0.0;
        }
        
        _force_mean = _force_mean_sum/_index_for_array;

        mean_file<<_force_mean<<" "<<_world.getWorldTime()<<" "<<"\n";
        mean_file.flush();
      }

      //controllo media

      if( _force_mean > 70.0 ){
        _do_replanning = false;

        _force_found = true;
      }else{
        _do_replanning = true;
      }

      _01_done = true;

      _index_for_array = 0;
      _force_mean_sum = 0.0;
      _force_mean = 0.0;

    }*/

    /*if( t >= 0.2 && !_02_done && !_force_found){

      //calcolo media

      double _force_mean = 0.0;

      for(int i=0; i<_index_for_array; i++){

        if( _tip_force_array(i,0) < 200 && _tip_force_array(i,0) >= 0.0){
          _force_mean += _tip_force_array(i,0);
        }else{
          _force_mean += 0.0;
        }

        _force_mean = _force_mean/_index_for_array;
      }

      //controllo media

      if( _force_mean > 70.0){
        _do_replanning = false;

        _force_found = true;
      }else{
        _do_replanning = true;
      }

      _02_done = true;

      _index_for_array = 0;

      std::cerr<<_force_mean<<std::endl;
      std::cerr<<"---"<<std::endl;
    }else{
      _index_for_array = 0;
    }*/
  


  }

  return 0;
}

int QUADRUPED::swing_phase(){

  //bool endswing= false;

  //bool takeoff=false;  


  while((_world.getWorldTime()-_begin) < _duration){ 

    double t = _world.getWorldTime()-_begin; 

    _r_c_ref<<plan->get_solution().base_linear_->GetPoint(t).p(), plan->get_solution().base_angular_->GetPoint(t).p();
    _r_c_ref_dot<<plan->get_solution().base_linear_->GetPoint(t).v(), plan->get_solution().base_angular_->GetPoint(t).v();
    _r_c_ref_dot_dot<<plan->get_solution().base_linear_->GetPoint(t).a(), plan->get_solution().base_angular_->GetPoint(t).a();
    
    _accd<< plan->get_solution().ee_motion_.at(3)->GetPoint(t).a(),  
            plan->get_solution().ee_motion_.at(0)->GetPoint(t).a();  

    
    _posdelta<< plan->get_solution().ee_motion_.at(3)->GetPoint(t).p()-quad->getBRpos(), 
                plan->get_solution().ee_motion_.at(0)->GetPoint(t).p()-quad->getFLpos(); 

    
    _veldelta<< plan->get_solution().ee_motion_.at(3)->GetPoint(t).v()-quad->getBRvel(), 
                plan->get_solution().ee_motion_.at(0)->GetPoint(t).v()-quad->getFLvel(); 



    get_raisim(1); 

    update(1);

    //////////////ESTIMATOR///////////////////////////////////
    
    compute_estimation(1);

    //////////////////////////////////////////////////////////

    ////////////COMPUTE F_GR_STAR & Q_DOT_DOT_DES//////////

    opti->compute_optimization(1);
 
    //////////////////////////////////////////////////////


    //////////COMPUTE JNT_TORQUES_STAR//////

    _jnt_torques_star=_m22_bar*_q_dot_dot_des + _bias_com.block(6,0,12,1) - _j_st_j_bar_1_new.transpose()*_f_gr_star;  

    //f_des_file<<_f_gr_star(0,0)<<" "<<_f_gr_star(1,0)<<" "<<_f_gr_star(2,0)<<" "<<"\n";
    //f_des_file.flush();


    //////////////////////////////////////

    _torque_array[12]=_jnt_torques_star(2,0); //right hind hip
    _torque_array[13]=_jnt_torques_star(6,0); //right hind thigh
    _torque_array[14]=_jnt_torques_star(7,0); //right hind shank

    _torque_array[15]=_jnt_torques_star(0,0); //left hind hip
    _torque_array[16]=_jnt_torques_star(10,0); //left hind thigh
    _torque_array[17]=_jnt_torques_star(11,0); //left hind shank

    _torque_array[6]=_jnt_torques_star(3,0); //right front hip
    _torque_array[7]=_jnt_torques_star(4,0); //right front thigh
    _torque_array[8]=_jnt_torques_star(5,0); //right front shank

    _torque_array[9]=_jnt_torques_star(1,0); //left front hip
    _torque_array[10]=_jnt_torques_star(8,0); //left front thigh
    _torque_array[11]=_jnt_torques_star(9,0); //left front shank

    //_anymalC->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

    _anymalC->setGeneralizedForce(_torque_array); //setto le coppie

  }

    return 0;
}

int QUADRUPED::swing_phase2(){   

  //bool takeoff=false;

  //bool endswing=false;

  while((_world.getWorldTime()-_begin) < _duration){ 

    double t = _world.getWorldTime()-_begin;

    _r_c_ref<<plan->get_solution().base_linear_->GetPoint(t).p(), plan->get_solution().base_angular_->GetPoint(t).p();
    _r_c_ref_dot<<plan->get_solution().base_linear_->GetPoint(t).v(), plan->get_solution().base_angular_->GetPoint(t).v();
    _r_c_ref_dot_dot<<plan->get_solution().base_linear_->GetPoint(t).a(), plan->get_solution().base_angular_->GetPoint(t).a();

    //std::cerr<<_r_c(2,0)<<std::endl;

       
    _accd<< plan->get_solution().ee_motion_.at(1)->GetPoint(t).a(),
            plan->get_solution().ee_motion_.at(2)->GetPoint(t).a();

    
    _posdelta<< plan->get_solution().ee_motion_.at(1)->GetPoint(t).p()-quad->getFRpos(),
                plan->get_solution().ee_motion_.at(2)->GetPoint(t).p()-quad->getBLpos();

    
    _veldelta<< plan->get_solution().ee_motion_.at(1)->GetPoint(t).v()-quad->getFRvel(),
                plan->get_solution().ee_motion_.at(2)->GetPoint(t).v()-quad->getBLvel();
  

    get_raisim(2);

    update(2);

    //////////////ESTIMATOR//////////////////////////////

    compute_estimation(2);

    /////////////////////////////////////////////////////

    ////////////COMPUTE F_GR_STAR & Q_DOT_DOT_DES//////////

    opti->compute_optimization(2);  

    //////////////////////////////////////////////////////


    //////////COMPUTE JNT_TORQUES_STAR//////

    _jnt_torques_star=_m22_bar*_q_dot_dot_des + _bias_com.block(6,0,12,1) - _j_st_j_bar_1_new.transpose()*_f_gr_star;

    //f_des_file<<0.0<<" "<<0.0<<" "<<0.0<<" "<<"\n";
    //f_des_file.flush();
    
    //////////////////////////////////////

    _torque_array[12]=_jnt_torques_star(2,0); //right hind hip
    _torque_array[13]=_jnt_torques_star(6,0); //right hind thigh
    _torque_array[14]=_jnt_torques_star(7,0); //right hind shank

    _torque_array[15]=_jnt_torques_star(0,0); //left hind hip
    _torque_array[16]=_jnt_torques_star(10,0); //left hind thigh
    _torque_array[17]=_jnt_torques_star(11,0); //left hind shank

    _torque_array[6]=_jnt_torques_star(3,0); //right front hip
    _torque_array[7]=_jnt_torques_star(4,0); //right front thigh
    _torque_array[8]=_jnt_torques_star(5,0); //right front shank

    _torque_array[9]=_jnt_torques_star(1,0); //left front hip
    _torque_array[10]=_jnt_torques_star(8,0); //left front thigh
    _torque_array[11]=_jnt_torques_star(9,0); //left front shank

    //_anymalC->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

    _anymalC->setGeneralizedForce(_torque_array); //setto le coppie

  
  }

  return 0;
}


void QUADRUPED::loop(){

  //set time world
  _world.setWorldTime(0.0);

  _torque_array=Eigen::VectorXd::Zero(_anymalC->getDOF()); 

  //_world.getContactSolver().setOrder(false);

  //_world.getContactSolver().setTimestep(0.0005);

  //_world.setERP(0,0);
  

  while(1){

    if(!_first_cycle){

      while(!(_server.isConnected())){
        std::cerr<<"Waiting for the server to connect..."<<std::endl;
      }
      usleep(2*1e6);

      //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
      
      _start_integrate=true;

      while(!_integration_started){
        usleep(0.1*1e6);
      }

      //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
      //std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

      //uncomment if you want the robot to stand still doing nothing...
      //cin.get();

    }

    get_raisim(0);
  
    update(0);

    if(_drill){
      drilling();
    }else{
      walking();
    }

  }

}

void QUADRUPED::integrate_world(){
   
 
  //ros::Rate r(1000);
  while(1){

    while(!_start_integrate){
      usleep(0.1*1e6);
    }
    _integration_started=true;

    //applying external force for 1 sec
    

    //if(_world.getWorldTime() > 1.0 && _world.getWorldTime() < 7.0){
    // Eigen::Vector3d force_sample; 
    // force_sample << 10.0, 0.0, 0.0;
    // Eigen::Vector3d pos_sample; 
    // pos_sample << 0.0, 0.0, -0.20;
    // auto bodyIdx = _anymalC->getBodyIdx("FL_calf");
    // _anymalC->setExternalForce(bodyIdx, pos_sample, force_sample);
//
    // auto bodyIdx_new = _anymalC->getBodyIdx("FR_calf");
    // _anymalC->setExternalForce(bodyIdx_new, pos_sample, force_sample);
    //}
    //else{
    //  _start_integrate=false;
    //  cin.get();
    //}

    //quando applico questa forza esterna il programma viene sempre bloccato da errori di segmentation fault (per risolvere questo problema applicare la forza in integrate_world)
    //auto body_index=_anymalC->getBodyIdx("hole_saw");
    //_anymalC->setExternalForce(body_index, _hole_saw_pos,  _external_force);

    
    _server.integrateWorldThreadSafe();

    usleep(1000);

  }
}

void QUADRUPED::push(){
   

  while(1){

    while(!_start_pushing){
      usleep(0.1*1e6);
    }

    //if(((_world.getWorldTime()-_begin) < 0.1) || ( ((_world.getWorldTime()-_begin) > 0.2) && ((_world.getWorldTime()-_begin) < 0.3) )){
    //  _torque_array[18]=100;
    //}else{
    //  _torque_array[18]=0;
    //}

    _torque_array[18]=70;

    usleep(1000);
    
  }
}

void QUADRUPED::run(){

  //ros::MultiThreadedSpinner spinner(2); //change number if you have multiple threads

  //boost::thread integrate_world_t ( &QUADRUPED::integrate_world, this ); 

  //boost::thread loop_t ( &QUADRUPED::loop, this );   
   
  //spinner.spin(); //prima era decommentata



  boost::thread integrate_world_t ( &QUADRUPED::integrate_world, this ); 
  boost::thread loop_t ( &QUADRUPED::loop, this ); 
  //boost::thread push_t ( &QUADRUPED::push, this ); 

  integrate_world_t.join();
  loop_t.join();
  //push_t.join();
  

}

Eigen::Matrix<double,6,1> QUADRUPED::get_init_pos(){
  return _r_c;
}

Eigen::MatrixXd QUADRUPED::getBRpos(){
	iDynTree::Transform  World_br;
  World_br=kinDynComp.getWorldTransform(13); //linear actuator 13 al posto di 14
	return toEigen(World_br.getPosition());
}

Eigen::MatrixXd QUADRUPED::getBLpos(){
	iDynTree::Transform  World_bl;
  World_bl=kinDynComp.getWorldTransform(19); //linear actuator 19 al posto di 20
	return toEigen(World_bl.getPosition());
}

Eigen::MatrixXd QUADRUPED::getFLpos(){
	iDynTree::Transform  World_fl;
  World_fl=kinDynComp.getWorldTransform(16); //linear actuator 16 al posto di 17
	return toEigen(World_fl.getPosition());
}

Eigen::MatrixXd QUADRUPED::getFRpos(){
	iDynTree::Transform  World_fr;
  World_fr=kinDynComp.getWorldTransform(10); //linear actuator 10 al posto di 11
 	return toEigen(World_fr.getPosition());
}

Eigen::Vector3d QUADRUPED::get_pc(){
  return _pc;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_init_vel(){
  return _v_c.block(0,0,6,1);
}

Eigen::Matrix<double,6,1> QUADRUPED::get_r_c(){
  return _r_c;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_r_c_ref(){
  return _r_c_ref;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_r_c_dot(){
  return _r_c_dot;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_r_c_ref_dot(){
  return _r_c_ref_dot;
}

double QUADRUPED::get_m(){
  return _robot_mass;
}

Eigen::Matrix<double,18,18> QUADRUPED::get_mass_matrix(){
  return _m_bar;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_r_c_ref_dot_dot(){
  return _r_c_ref_dot_dot;
}

void QUADRUPED::set_w_com_des(Eigen::Matrix<double,6,1> data){
  _w_com_des=data;
}

double QUADRUPED::get_mu(){
  return _mu;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_accd(){
  return _accd;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_veldelta(){
  return _veldelta;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_posdelta(){
  return _posdelta;
}

Eigen::Matrix<double,12,6> QUADRUPED::get_j_st_c_bar_0(){
  return _j_st_c_bar_0;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_w_com_des(){
  return _w_com_des;
}

Eigen::Matrix<double,12,12> QUADRUPED::get_j_st_j_bar_0(){
  return _j_st_j_bar_0;
}

Eigen::Matrix<double,18,1>  QUADRUPED::get_bias_com(){
  return _bias_com;
}

Eigen::Matrix<double,12,18> QUADRUPED::get_j_st_dot_0(){
  return _j_st_dot_0;
}

Eigen::Matrix<double,12,1> QUADRUPED::get_joint_pos(){
  return _eig_robot_state.jointPos;
}

Eigen::Matrix<double,18,1> QUADRUPED::get_v_c(){
  return _v_c;
}

void QUADRUPED::set_q_dot_dot_des(Eigen::Matrix<double,12,1> data){
  _q_dot_dot_des=data;
}

void QUADRUPED::set_f_gr_star(Eigen::Matrix<double,12,1> data){
  _f_gr_star=data;
}

Eigen::Matrix<double,6,6> QUADRUPED::get_j_st_c_bar_1(){
  return _j_st_c_bar_1;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_j_st_dot_1(){
  return _j_st_dot_1;
}

Eigen::Matrix<double,6,6> QUADRUPED::get_j_sw_c_bar(){
  return _j_sw_c_bar;
}

Eigen::Matrix<double,6,12> QUADRUPED::get_j_sw_j_bar(){
  return _j_sw_j_bar;
}

Eigen::Matrix<double,6,1> QUADRUPED::get_j_sw_dot(){
  return _j_sw_dot;
}

/*void QUADRUPED::set_world(raisim::World data){
  _world=data;
}*/

void  QUADRUPED::set_init_pos(Eigen::Matrix<double,6,1> data){
  _init_pos=data;
}

void QUADRUPED::set_init_vel(Eigen::Matrix<double,6,1> data){
  _init_vel=data;
}

void QUADRUPED::set_duration(double data){
  _duration=data; 
}

Eigen::MatrixXd QUADRUPED::getBRvel(){
    
  iDynTree::Twist br_vel;
	br_vel=kinDynComp.getFrameVel(13); //linear actuator 13 al posto di 14
	return toEigen(br_vel.getLinearVec3() );

}

Eigen::MatrixXd QUADRUPED::getBLvel(){ 

	iDynTree::Twist bl_vel;
	bl_vel=kinDynComp.getFrameVel(19); //linear actuator 19 al posto di 20
	return toEigen(bl_vel.getLinearVec3() );

}

Eigen::MatrixXd QUADRUPED::getFLvel(){

  iDynTree::Twist fl_vel;
	fl_vel=kinDynComp.getFrameVel(16); //linear actuator 16 al posto di 17
	return toEigen(fl_vel.getLinearVec3() );

}

Eigen::MatrixXd QUADRUPED::getFRvel(){

	iDynTree::Twist fr_vel;
	fr_vel=kinDynComp.getFrameVel(10); //linear actuator 10 al posto di 11
	return toEigen(fr_vel.getLinearVec3() );

}

bool QUADRUPED::get_raisim_ready(){
  return _raisim_ready;
}

Eigen::Matrix<double,6,12> QUADRUPED::get_j_st_j_bar_1(){
  return _j_st_j_bar_1;
}

Eigen::Matrix<double,18,18> QUADRUPED::get_eig_mass_matrix(){
  return _eig_mass_matrix;
}

void QUADRUPED::pd_controller(){

  _jointNominalConfig << -0.002, 0, 0.35, 0.999, 0.0, 0.04361, 0.0, 0.03, 0.6, -1.2, -0.03, 0.6, -1.2, 0.03, 0.6, -0.9, -0.03, 0.6, -0.9, 0.0; //( in caso di linear actuator aggiungere 0.0 alla fine)

  _jointVelocityTarget << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; //(in caso di linear actuator aggiungere 0 alla fine)


  _jointPgain.tail(13).setConstant(2000.0); //(in caso di linear actuator sostituire 12 con 13)
  _jointDgain.tail(13).setConstant(20.0); //(in caso di linear actuator sostituire 12 con 13)

  _anymalC->setGeneralizedCoordinate(_jointNominalConfig);
  _anymalC->setGeneralizedForce(Eigen::VectorXd::Zero(_anymalC->getDOF()));
  _anymalC->setPdGains(_jointPgain, _jointDgain);
  _anymalC->setPdTarget(_jointNominalConfig, _jointVelocityTarget);
  _anymalC->setName("AnymalC");

}

void QUADRUPED::pd_controller_current(){

  raisim::VecDyn vec_sample=_anymalC->getGeneralizedCoordinate();
  Vector<double, 20> vec_sample_eig_pos=vec_sample.e(); //(in caso di linear actuator sostituire 19 con 20)
  for(int i=0; i<20; i++){ //(in caso di linear actuator sostituire 19 con 20)
    _jointNominalConfig[i]=vec_sample_eig_pos[i];
  }

  //_jointVelocityTarget.setZero();
  vec_sample=_anymalC->getGeneralizedVelocity();
  Vector<double, 19> vec_sample_eig_vel=vec_sample.e(); //(in caso di linear actuator sostituire 18 con 19)
  for(int i=0; i<19; i++){ //(in caso di linear actuator sostituire 18 con 19)
    _jointVelocityTarget[i]=vec_sample_eig_vel[i];
  }

  _jointPgain.tail(13).setConstant(2000.0); //(in caso di linear actuator sostituire 12 con 13)
  _jointDgain.tail(13).setConstant(20.0); //(in caso di linear actuator sostituire 12 con 13)

  //_anymalC->setGeneralizedCoordinate(_jointNominalConfig);
  //Vector<double,13> _only_prismatic;
  //_only_prismatic<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0;
  _anymalC->setGeneralizedForce(Eigen::VectorXd::Zero(_anymalC->getDOF())); //Eigen::VectorXd::Zero(_anymalC->getDOF())

  _anymalC->setPdGains(_jointPgain, _jointDgain);
  _anymalC->setPdTarget(_jointNominalConfig, _jointVelocityTarget);

}

raisim::ArticulatedSystem* QUADRUPED::get_anymal(){
  return _anymalC;
}

/*void QUADRUPED::set_world(raisim::World data){
  _world=data;
}*/

raisim::World* QUADRUPED::get_world(){
  return &_world;
}

raisim::RaisimServer* QUADRUPED::get_server(){
  return &_server;
}

Eigen::Vector3d QUADRUPED::get_com_raisim(){
  return vec_sample_com_eig;
}

void QUADRUPED::compute_estimation(int flag){

  //metodo alternativo per ricavare l'orientamento di un piede (restituisce esattamente lo stesso risultato di getWorldTransform)
  //auto footFrameIndex = _anymalC->getFrameIdxByName("FR_foot_fixed"); // the URDF has a joint named "foot_joint"
  //raisim::Mat<3,3> footOrientation;
  //_anymalC->getFrameOrientation(footFrameIndex, footOrientation);

  Tbl=kinDynComp.getWorldTransform(19);
  _Tbl=toEigen(Tbl.getRotation());
  //std::cerr<<"contact frame LH raisim: "<<std::endl;
  //std::cerr<<_Tbl<<std::endl; 


  Tbr=kinDynComp.getWorldTransform(13);
  _Tbr=toEigen(Tbr.getRotation());
  //std::cerr<<"contact frame RH raisim: "<<std::endl;
  //std::cerr<<_Tbr<<std::endl; 


  Tfl=kinDynComp.getWorldTransform(16);
  _Tfl=toEigen(Tfl.getRotation());
  //std::cerr<<"contact frame LF raisim: "<<std::endl;
  //std::cerr<<_Tfl<<std::endl; 


  Tfr=kinDynComp.getWorldTransform(10);
  _Tfr=toEigen(Tfr.getRotation());
  //std::cerr<<"contact frame RF raisim: "<<std::endl;
  //std::cerr<<_Tfr<<std::endl; 

  if(flag==0){
    _Fgrf<< _f_gr.block(0,0,3,1),
            _f_gr.block(3,0,3,1),
            _f_gr.block(6,0,3,1),
            _f_gr.block(9,0,3,1); 
  }else if(flag==1){
    _Fgrf<< _f_gr.block(0,0,3,1),
            Eigen::Matrix<double,3,1>::Zero(),
            Eigen::Matrix<double,3,1>::Zero(),
            _f_gr.block(9,0,3,1); 
  }else if(flag==2){
    _Fgrf<< Eigen::Matrix<double,3,1>::Zero(),
            _f_gr.block(3,0,3,1),
            _f_gr.block(6,0,3,1),
            Eigen::Matrix<double,3,1>::Zero();
  }

  _Ccom=_t_bar.transpose().inverse()*_c*_t_bar.inverse()+_t_bar.transpose().inverse()*_eig_mass_matrix*_t_inv_der;

	_Ctq=_Ccom.transpose()*_v_c;

  _d=_Ctq.block(6,0,12,1) + _jnt_torques_star + _j_st_j_bar_0.transpose()*_Fgrf;

  _rho=_m22_bar*_v_c.block(6,0,12,1); 

  _yd[0]=_yd_prev[0]+(_d*_T); //integrale di alpha
  
  _w[0]=_k_1*(_rho-_yw_prev[0]-_yd[0]); //yw_prev[0] é l'integrale di f cappello (istante 0)
  
  _yw[0]=_yw_prev[0]+_w[0]*_T; //yw[0] é l'integrale di f cappello (che varrá al ciclo successivo)

  _yw[1]=_yw_prev[1]+_w[0]*_T; 
  _w[1] = _k_2*(-_ygamma_prev[1]+_yw[1]);
  _ygamma[1]=_ygamma_prev[1]+(_w[1]*_T); 

  _yw[2]=_yw_prev[2]+_w[1]*_T; 
  _w[2] = _k_3*(-_ygamma_prev[2]+_yw[2]);
  _ygamma[2]=_ygamma_prev[2]+(_w[2]*_T);
  
  
  _f_ext=(_B*_j_bar).block(0,6,12,12).transpose().inverse()*_w[2];

  f_ext_file<<_f_ext(0,0)<<" "<<_f_ext(1,0)<<" "<<_f_ext(2,0)<<" "<<_f_ext(3,0)<<" "<<_f_ext(4,0)<<" "<<_f_ext(5,0)<<" "<<_f_ext(6,0)<<" "<<_f_ext(7,0)<<" "<<_f_ext(8,0)<<" "<<_f_ext(9,0)<<" "<<_f_ext(10,0)<<" "<<_f_ext(11,0)<<" "<<_world.getWorldTime()<<"\n";
  f_ext_file.flush();
  f_gr_file<<_Fgrf(0,0)<<" "<<_Fgrf(1,0)<<" "<<_Fgrf(2,0)<<" "<<_Fgrf(3,0)<<" "<<_Fgrf(4,0)<<" "<<_Fgrf(5,0)<<" "<<_Fgrf(6,0)<<" "<<_Fgrf(7,0)<<" "<<_Fgrf(8,0)<<" "<<_Fgrf(9,0)<<" "<<_Fgrf(10,0)<<" "<<_Fgrf(11,0)<<" "<<_world.getWorldTime()<<"\n";
  f_gr_file.flush();

  _yd_prev=_yd;
  _yw_prev=_yw;
  _w_prev=_w;
  _ygamma_prev=_ygamma;


}

Eigen::Matrix<double,12,1> QUADRUPED::get_f_ext(){
  return _f_ext;
}

int QUADRUPED::get_stl1(){
  return _stl1;
}

int QUADRUPED::get_stl2(){
  return _stl2;
}

int QUADRUPED::get_swl1(){
  return _swl1;
}

int QUADRUPED::get_swl2(){
  return _swl2;
}

Eigen::Matrix<double,24,18> QUADRUPED::get_j_bar(){
  return _j_bar;
}

void QUADRUPED::drilling(){

  //commentare questo if statement in caso di undicesima prova
  //if(_world.getWorldTime() >= 4.0){
  //    plan->set_step_1(false);
  //    _pitch_ok=true;
  //}

  //////////////////////////////////////////////
  //durante il planning, stoppo raisim    
  _start_integrate=false;
  boost::this_thread::sleep_for(boost::chrono::microseconds(1000));
  _integration_started=false;

  //eseguo il planning
  plan->compute_planning(2);

  std::cerr<<"sono uscito dal planning 0"<<std::endl;

  
  //riprendo raisim
  _start_integrate=true;
  while(!_integration_started){
    usleep(0.1*1e6);
  }
  ///////////////////////////////////////////// 

  ///////////////////STAND////////////////////////////

     
  //_duration=plan->get_formulation().params_.ee_phase_durations_.at(0)[0];  

  _duration = 0.2;

  _begin=_world.getWorldTime(); 

  _anymalC->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

  //_start_pushing=true;

  _do_replanning = false; 

  stand_phase(); 

  if(!_force_found){
    _01_done = false;
  }

  ////////////////////////////////////
  //verifica sulla media ( prima_6, prima_7)
  
  //if( _impact ){
  //  for(int i=0; i<_index_for_array; i++){
  //  if( _tip_force_array(i,0) < 200 && _tip_force_array(i,0) >= 0.0){
  //    _force_mean_sum += _tip_force_array(i,0);
  //  }else{
  //    _force_mean_sum += 0.0;
  //  }
  //    _force_mean = _force_mean_sum/_index_for_array;
  //    mean_file<<_force_mean<<" "<<_world.getWorldTime()<<" "<<"\n";
  //    mean_file.flush();
  //  }
  //  if(_force_mean < 70.0 && _force_found){
  //    _gain+=10.0;
  //  }
  //}

  //////////////////////////////////
  //salvataggio media (prima_1, prima_2, prima_3, prima_4, prima_5)

  //if( _force_found ){
  //  for(int i=0; i<_index_for_array; i++){
  //  if( _tip_force_array(i,0) < 200 && _tip_force_array(i,0) >= 0.0){
  //    _force_mean_sum += _tip_force_array(i,0);
  //  }else{
  //    _force_mean_sum += 0.0;
  //  }
  //    _force_mean = _force_mean_sum/_index_for_array;
  //    mean_file<<_force_mean<<" "<<_world.getWorldTime()<<" "<<"\n";
  //    mean_file.flush();
  //  }
  //}

  //////////////////////////////////

  ////////////////////////////////////
  //verifica sulla media ( prima_8, prima_9, prima_10, prima_11 )
  
  if( _impact ){

    //ciclo for usato per "pulire" il vettore delle forze in punta dai doppioni
    for(int j=0; j<_index_for_array-1; j++){
      if( _tip_force_array(j, 1) == _tip_force_array(j+1, 1)  ){

        if( _tip_force_array(j, 0) < _tip_force_array(j+1, 0)){
          _tip_force_array_clean(j,0) = _tip_force_array(j+1, 0);
          _index_for_array_new++;
        }else{
          _tip_force_array_clean(j,0) = _tip_force_array(j, 0);
          _index_for_array_new++;
        }

      }else{
        _tip_force_array_clean(j,0) = _tip_force_array(j, 0);
        _index_for_array_new++;
      }
    }

    //in caso di pitch diverso da 0 vado a dividere la forza per il coseno(pitch)

    for(int i=0; i<_index_for_array_new; i++){

    if( _tip_force_array_clean(i,0) < 200 && _tip_force_array_clean(i,0) >= 0.0){
      _force_mean_sum += _tip_force_array_clean(i,0)/cos(_pitch);
    }else{
      _force_mean_sum += 0.0;
    }
      _force_mean = _force_mean_sum/_index_for_array;
      mean_file<<_force_mean<<" "<<_world.getWorldTime()<<" "<<"\n";
      mean_file.flush();
    }
    if(_force_mean < 70.0 && !_gain_found){
      _gain+=10.0;
    }else{
      _gain_found = true;
    }

    
  }

  //////////////////////////////////

  
  

  /////////////////////////////////
  _index_for_array = 0;

  _index_for_array_new = 0;

  _force_mean_sum = 0.0;

  _force_mean = 0.0;

  _M = 0;

  

  _first_cycle=true;

  _replanning_done=true;

  pd_controller_current();

  //return 0;
}

void QUADRUPED::walking(){
  //////////////////////////////////////////////
  //durante il planning, stoppo raisim    
  _start_integrate=false;
  boost::this_thread::sleep_for(boost::chrono::microseconds(1000));
  _integration_started=false;

  //eseguo il planning
  plan->compute_planning(0);

  std::cerr<<"sono uscito dal planning 0"<<std::endl;

  
  //riprendo raisim
  _start_integrate=true;
  while(!_integration_started){
    usleep(0.1*1e6);
  }
  ///////////////////////////////////////////// 

  ///////////////////STAND////////////////////////////

  _duration=plan->get_formulation().params_.ee_phase_durations_.at(3)[0];  
    
  _begin=_world.getWorldTime(); 

  _anymalC->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

  stand_phase(); 
  
  //////////////////////////////////////////////////////

  //////////////////////SWING///////////////////////////

  _duration=_duration+plan->get_formulation().params_.ee_phase_durations_.at(3)[1];

  swing_phase();
  
  ////////////////////////////////////////////////////

  ///////////////////STAND///////////////////////////

  _duration=_duration+plan->get_formulation().params_.ee_phase_durations_.at(3)[2];

  stand_phase(); 
    
  //////////////////////////////////////////////////

  get_raisim(0);

  update(0);

  //////////////////////////////////////////////////
  
  ////////////REPLANNING////////////////////////////
    
  //_anymalC->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE); //questa istruzione vien giá fatta quando setto i guadagni del controllore pd (intrinseca di SetGain), ricordati di commentarla
  pd_controller_current();

  _replanning_done=true;
    
  /////////////////////////////////////
  //durante il planning, stoppo raisim
  _start_integrate=false;
  boost::this_thread::sleep_for(boost::chrono::microseconds(1000));
  _integration_started=false;

  //eseguo il planning
  plan->compute_planning(1);

  //riprendo raisim
  _start_integrate=true;
  while(!_integration_started){
    usleep(0.1*1e6);
  }

  ///////////////////////////////////////////////////

  ///////////////////STAND///////////////////////////

  _duration=plan->get_formulation().params_.ee_phase_durations_.at(1)[0];

  _begin=_world.getWorldTime();

  _anymalC->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE); 

  stand_phase(); 

  //////////////////////////////////////////////////

  //////////////////SWING 2/////////////////////////

  _duration=_duration+plan->get_formulation().params_.ee_phase_durations_.at(1)[1];

  swing_phase2();

  //////////////////////////////////////////////////

  ///////////////////STAND//////////////////////////    
    
  _duration=_duration + plan->get_formulation().params_.ee_phase_durations_.at(1)[2];

  stand_phase();
    
  //////////////////////////////////////////////////
  
  //_anymalC->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE); //ricordati di verificare se funziona tutto una volta commentata questa istruzione
  pd_controller_current();

  /////////////////////////////////////////////////

  _first_cycle=true;

  _replanning_done=true;

  //return 0;

}

void QUADRUPED::compute_robustness(){

  for(int i=0; i<4; i++){

    _fc_i << _f_gr(_index_rob, 0), _f_gr(_index_rob+1, 0), _f_gr(_index_rob+2, 0);

    _product = _z_i(0)*( _fc_i(0)/_fc_i.norm() ) + _z_i(1)*( _fc_i(1)/_fc_i.norm() ) + _z_i(2)*( _fc_i(2)/_fc_i.norm() );

    _alpha_i = acos(_product);

    _sum = _sum + 1/( (_theta_i - _alpha_i)*(_theta_i + _alpha_i) );

    _index_rob += 3;


  }

  _index_rob = 0;

  _h_rob = (1/_lambda)*_sum;

  _sum = 0;

  _robustness = 1/_h_rob;

  _robustness_array[_index_rob_array] = _robustness;

  _index_rob_array++;

  
}

Eigen::Matrix<double,3,1> QUADRUPED::get_tip_force(){
  return _tip_force;
}

bool QUADRUPED::get_do_replanning(){
  return _do_replanning;
}

double QUADRUPED::get_pitch(){
  return _pitch;
}

double QUADRUPED::get_actuator_torque(){
  return _torque_array[18];
}

void QUADRUPED::low_pass_filter(double *x, double *y, int M){
  
  y[0] = -(_K/(_a - 1))*x[0] + ( 1/(_a - 1) )*y[1] - (_K/( _a - 1))*x[1];

  //y[0] = 0.0;

  for (int n=1; n < M ; n++) {
    //y[n] =  ((_a - 1)/_a)*y[n -1]  + (_K/_a)*x[n-1]; //eulero in avanti

    y[n] =  ((_a - 1)/(_a + 1))*y[n -1]  + (_K/(_a + 1))*x[n-1] + (_K/(_a + 1))*x[n]; //tustin
  }

}

double QUADRUPED::get_tip_force_filt(){
  return _tip_force_array(_index_for_array, 0);
}

double QUADRUPED::get_yy(){
  return _y_y[_M - 1];
}

double QUADRUPED::get_yz(){
  return _y_z[_M - 1];
}

bool QUADRUPED::get_force_reduced(){
  return _force_reduced;
}

void QUADRUPED::set_force_reduced(bool data){
  _force_reduced = data;
}

bool QUADRUPED::get_replanning_2_acceleration(){
  return _replanning_2_acceleration;
}

double QUADRUPED::get_gain(){
  return _gain;
}

bool QUADRUPED::get_impact(){
  return _impact;
}

bool QUADRUPED::get_drill(){
  return _drill;
}

bool QUADRUPED::get_force_found(){
  return _force_found;
}



int main(int argc, char* argv[]) {

  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");

  quad = new QUADRUPED();

  plan = new PLANNING(*quad);

  opti = new OPTIMAL(*quad);

  
#if WIN32
    timeBeginPeriod(1); // for sleep_for function. windows default clock speed is 1/64 second. This sets it to 1ms.
#endif

  //create raisim world
  //raisim::World world;
  (quad->get_world())->setTimeStep(0.001);

  //create objects
  (quad->get_world())->addGround(0, "gnd");

  auto balla=(quad->get_world())->addCylinder(1.2, 0.885, 200.0, "fieno");

  balla->setPosition(1.2,0,1.2); //1.22 (1,2,3,4,5,8,9,10) //1.25 (6) //1.3 (7)

  balla->setOrientation(0.707,0,0.707,0);

  balla->setBodyType(raisim::BodyType::STATIC);

  auto anymalC = (quad->get_world())->addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\a1\\urdf\\a1.urdf");

  quad->set_anymalC(anymalC);

  quad->set_balla(balla);

  //friction example. uncomment it to see the effect
  (quad->get_anymal())->getCollisionBody("FL_foot/0").setMaterial("FL_foot");
  (quad->get_world())->setMaterialPairProp("gnd", "FL_foot", 0.6, 0.0, 0.0);
  (quad->get_anymal())->getCollisionBody("RL_foot/0").setMaterial("RL_foot");
  (quad->get_world())->setMaterialPairProp("gnd", "RL_foot", 0.6, 0.0, 0.0);
  (quad->get_anymal())->getCollisionBody("FR_foot/0").setMaterial("FR_foot");
  (quad->get_world())->setMaterialPairProp("gnd", "FR_foot", 0.6, 0.0, 0.0);
  (quad->get_anymal())->getCollisionBody("RR_foot/0").setMaterial("RR_foot");
  (quad->get_world())->setMaterialPairProp("gnd", "RR_foot", 0.6, 0.0, 0.0);

  (quad->get_world())->setMaterialPairProp("gnd", "fieno", 0.6, 0.0, 0.0);

  (quad->get_anymal())->getCollisionBody("hole_saw/0").setMaterial("hole_saw");

  (quad->get_world())->setMaterialPairProp("fieno", "hole_saw", 0.4, 0.0, 0.0); //0.4

  //auto wire = (quad->get_world())->addStiffWire(balla, 0, {-0.4425,0,0.0}, anymalC, 0, {0.6,0,0}, 0.05);

  //wire->setStretchType(raisim::LengthConstraint::StretchType::BOTH);

  //anymalC joint PD controller
  quad->pd_controller();
  
  //inizializzazione di idyntree
  quad->init_model();

  //launch raisim server
  //raisim::RaisimServer server(&world);

  (quad->get_server())->launchServer();

  (quad->get_server())->focusOn(quad->get_anymal());


  quad->run();

  (quad->get_server())->killServer();

  
  
}
