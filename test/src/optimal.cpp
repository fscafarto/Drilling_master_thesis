#include "optimal.h"

std::ofstream w_com_des_file("w_com_des_file.txt");

std::ofstream diff_file("diff_file.txt");

OPTIMAL::OPTIMAL(QUADRUPED &quadruped){
    dogbot = &quadruped;

    initialization();

}

void OPTIMAL::compute_w_com_des(){

    Eigen::Matrix<double,6,1> w_com_des_temp=Eigen::MatrixXd::Zero(6,1);

    _k_p=900*Eigen::MatrixXd::Identity(6,6); //3500 //350 //1590 //1000 || //900

    _k_d=120*Eigen::MatrixXd::Identity(6,6); //50 //50 //500 //200 || //120

    Eigen::Matrix<double,6,1> temp_matrix=Eigen::MatrixXd::Zero(6,1);

    temp_matrix=dogbot->get_r_c() - dogbot->get_r_c_ref();

    //////////////////////////////////////////////
    //prima_6, prima_7 (nel caso di prima_7 la soglia sul wrench desiderato non é 70, ma é 90)
    
    //if( ( (dogbot->get_r_c_ref()(0,0) - dogbot->get_r_c()(0,0)) > 0.001 ) && ( w_com_des_temp(0,0) < 70 ) && ( dogbot->get_force_found() )){
    //    _k_p(0,0) = ( dogbot->get_gain() - dogbot->get_mass_matrix()(0,0)*dogbot->get_r_c_ref_dot_dot()(0,0) + _k_d(0,0)*( dogbot->get_r_c_dot()(0,0) - dogbot->get_r_c_ref_dot()(0,0) ))/( dogbot->get_r_c_ref()(0,0) -  dogbot->get_r_c()(0,0));
    //    if( _k_p(0,0) < 0.0){
    //        _k_p(0,0) = 900;
    //    }else{
    //        w_com_des_temp= -_k_p*temp_matrix -_k_d*(dogbot->get_r_c_dot() - dogbot->get_r_c_ref_dot()) + dogbot->get_m()*g + dogbot->get_mass_matrix().block(0,0,6,6)*dogbot->get_r_c_ref_dot_dot();
    //    }
    //}

    //////////////////////////////////////////////

    //////////////////////////////////////////////
    //prima_8, prima_9, terza_1

    _k_p(0,0) = ( dogbot->get_gain() - dogbot->get_mass_matrix()(0,0)*dogbot->get_r_c_ref_dot_dot()(0,0) + _k_d(0,0)*( dogbot->get_r_c_dot()(0,0) - dogbot->get_r_c_ref_dot()(0,0) ) )/( dogbot->get_r_c_ref()(0,0) -  dogbot->get_r_c()(0,0));

    w_com_des_temp= -_k_p*temp_matrix -_k_d*(dogbot->get_r_c_dot() - dogbot->get_r_c_ref_dot()) + dogbot->get_m()*g + dogbot->get_mass_matrix().block(0,0,6,6)*dogbot->get_r_c_ref_dot_dot();

    /////////////////////////////////////////////

    /////////////////////////////////////////////
    //prima_10

    //if( (dogbot->get_r_c_ref()(0,0) - dogbot->get_r_c()(0,0)) > 0.001 ){
    //    _k_p(0,0) = ( dogbot->get_gain() - dogbot->get_mass_matrix()(0,0)*dogbot->get_r_c_ref_dot_dot()(0,0) + _k_d(0,0)*( dogbot->get_r_c_dot()(0,0) - dogbot->get_r_c_ref_dot()(0,0) ))/( dogbot->get_r_c_ref()(0,0) -  dogbot->get_r_c()(0,0));
    //    w_com_des_temp= -_k_p*temp_matrix -_k_d*(dogbot->get_r_c_dot() - dogbot->get_r_c_ref_dot()) + dogbot->get_m()*g + dogbot->get_mass_matrix().block(0,0,6,6)*dogbot->get_r_c_ref_dot_dot();
    //}
    
    ////////////////////////////////////////////

    ///////////////////////////////////////////
    //prima_11

    //_k_p(0,0) = ( dogbot->get_gain() - dogbot->get_mass_matrix()(0,0)*dogbot->get_r_c_ref_dot_dot()(0,0) + _k_d(0,0)*( dogbot->get_r_c_dot()(0,0) - dogbot->get_r_c_ref_dot()(0,0) ))/( dogbot->get_r_c_ref()(0,0) -  dogbot->get_r_c()(0,0));
    //if( _k_p(0,0) > 10^(5) ){
    //    _k_p(0,0) = 10^(5);
    //}
    //w_com_des_temp= -_k_p*temp_matrix -_k_d*(dogbot->get_r_c_dot() - dogbot->get_r_c_ref_dot()) + dogbot->get_m()*g + dogbot->get_mass_matrix().block(0,0,6,6)*dogbot->get_r_c_ref_dot_dot();

    //////////////////////////////////////////

    dogbot->set_w_com_des(w_com_des_temp);

    w_com_des_file<<w_com_des_temp(0,0)<<" "<<w_com_des_temp(1,0)<<" "<<w_com_des_temp(2,0)<<" "<<w_com_des_temp(3,0)<<" "<<w_com_des_temp(4,0)<<" "<<w_com_des_temp(5,0)<<" "<<(dogbot->get_world())->getWorldTime()<<" "<<_k_p(0,0)<<"\n";
    w_com_des_file.flush();

}

void OPTIMAL::compute_d_fr(int flag){

    if(flag==0){

       for(int i=0; i<4; i++){
            _d_fr_0.block<5,3>(i*5,i*3) << (_t1 - dogbot->get_mu()*_n).transpose(),
                                          -(_t1 + dogbot->get_mu()*_n).transpose(),
                                           (_t2 - dogbot->get_mu()*_n).transpose(),
                                          -(_t2 + dogbot->get_mu()*_n).transpose(),
                                          -_n.transpose();
        }
    }

    if(flag==1 || flag==2){
        for(int i=0; i<2; i++){
            _d_fr_1.block<5,3>(i*5,i*3) << (_t1 - dogbot->get_mu()*_n).transpose(),
                                          -(_t1 + dogbot->get_mu()*_n).transpose(),
                                           (_t2 - dogbot->get_mu()*_n).transpose(),
                                          -(_t2 + dogbot->get_mu()*_n).transpose(),
                                          -_n.transpose();
        }
    }

}

void OPTIMAL::compute_optimization(int flag){
    //initialization();
  
    compute_w_com_des();

    compute_d_fr(flag);   

    if(flag==1 || flag==2){ //introdurre fext_lambda qui
       
        _k_d_sw=50*Eigen::MatrixXd::Identity(6,6); //50 //100 //100 //30 //50 //600 || //50

        _k_p_sw=500*Eigen::MatrixXd::Identity(6,6); //250 //1200 //1250 //110 //2500 //300 || //500
    
        _x_sw_cmd_dot_dot= dogbot->get_accd() + _k_d_sw*dogbot->get_veldelta() + _k_p_sw*dogbot->get_posdelta();// -_fext_lambda; //- dogbot->get_j_sw()*_m_c.inverse()*_n_c*dogbot->get_j_sw().transpose()*dogbot->get_f_sw_hat();
        
    }
       
    
    if(flag==0){ //qui manca un termine aggiuntivo sulla funzione di costo
    
        _A_0= _sigma_0.transpose()*dogbot->get_j_st_c_bar_0()*weight_1*dogbot->get_j_st_c_bar_0().transpose()*_sigma_0 + Eigen::MatrixXd::Identity(30,30);
        
        //STIMATORE (termine aggiutivo in b2_0)
    
        _b2_0= -_sigma_0.transpose()*dogbot->get_j_st_c_bar_0()*weight_1.transpose()*dogbot->get_w_com_des() + _sigma_0.transpose()*dogbot->get_j_st_c_bar_0()*weight_1.transpose()*dogbot->get_j_st_c_bar_0().transpose()*dogbot->get_f_ext();
            
        //cost function
        
        A.setlength(_A_0.rows(),_A_0.cols());
        
        for(int i=0;i<_A_0.rows();i++){
            for(int j=0; j<_A_0.cols();j++){
                A(i,j)=_A_0(i,j);
            }
        }

        b.setlength(_b2_0.rows());
        for(int i=0; i<_b2_0.rows(); i++){
            b(i)=_b2_0(i,0);//_b1_0(0,i);//+_b2_0(i,0);
        }

        ///////////////////////////////////////////////////////////////
    
        //////////////////////////////////////////////////////////////
        //CONSTRAINTS('con' stands for the matrix which are passed to alglib script)
    
        _A_con_0.block(0,0,6,6)=dogbot->get_mass_matrix().block(0,0,6,6);
        //_A_con_0.block(0,6,6,12)=Eigen::MatrixXd::Zero(6,12);
        _A_con_0.block(0,18,6,12)=-dogbot->get_j_st_c_bar_0().transpose();
        //_A_con_0.block(0,6+12+12,6,3*dogbot->get_n_sw())=Eigen::MatrixXd::Zero(6,3*dogbot->get_n_sw());
    
        _A_con_0.block(6,0,12,6)=dogbot->get_j_st_c_bar_0();
        _A_con_0.block(6,6,12,12)=dogbot->get_j_st_j_bar_0();
        //_A_con_0.block(6,6+12,12,12)=Eigen::MatrixXd::Zero(12, 12);
        //_A_con.block(6,6+12+12,12,3*dogbot->get_n_sw())=Eigen::MatrixXd::Zero(3*dogbot->get_n_st(),3*dogbot->get_n_sw());    
        
        /////////////////////////////////////////////////
        //STIMATORE (calcolo termine in b_con_0)
        _JacCOM_lin=_B_opt*dogbot->get_j_bar(); //12x24x24x18

        _Jstcom=_JacCOM_lin.block(0,0,12,6);
        
        _b_con_0.block(0,0,6,1)=-dogbot->get_bias_com().block(0,0,6,1) + _Jstcom.transpose()*dogbot->get_f_ext();
        
        ///////////////////////////////////////////////

        _b_con_0.block(6,0,12,1)=-dogbot->get_j_st_dot_0();
    
        //put A and b together
       
        A_b_con.setlength(_A_con_0.rows() + _D_con_0.rows(), _A_con_0.cols()+1);
    
        for(int i=0;i<_A_con_0.rows();i++){
            for(int j=0; j< _A_con_0.cols();j++){
                A_b_con(i,j)=_A_con_0(i,j);
            }
        }
        
    
        //_D_con_0.block(0,0,4*4,6)=Eigen::MatrixXd::Zero(4*4,6);
        //_D_con_0.block(0,6,4*4,12)=Eigen::MatrixXd::Zero(4*4, 12);

        ///_D_con_0.block(0,6+12,4*4, 3*4)=_d_fr_0;
        _D_con_0.block(0,6+12,4*5, 3*4)=_d_fr_0;

        //_D_con.block(0,6+12+3*4,4*4,3*dogbot->get_n_sw())=Eigen::MatrixXd::Zero(4*4, 3*dogbot->get_n_sw());
    
        //_D_con_0.block(4*4,0,12,6)=Eigen::MatrixXd::Zero(12,6);
        _D_con_0.block(4*5,6,12,12)=dogbot->get_mass_matrix().block(6,6,12,12);
        _D_con_0.block(4*5,6+12,12,3*4)=-dogbot->get_j_st_j_bar_0().transpose();
        //_D_con.block(4*4,6+12+3*4,12, 3*dogbot->get_n_sw())=Eigen::MatrixXd::Zero(12, 3*dogbot->get_n_sw());

        //_D_con_0.block(4*4+12,0,12,6)=Eigen::MatrixXd::Zero(12,6);
        _D_con_0.block(4*5+12,6,12,12)=-dogbot->get_mass_matrix().block(6,6,12,12);
        _D_con_0.block(4*5+12,6+12,12,3*4)=dogbot->get_j_st_j_bar_0().transpose();
        //_D_con.block(4*4+12,6+12+3*4,12, 3*dogbot->get_n_sw())=Eigen::MatrixXd::Zero(12, 3*dogbot->get_n_sw());

        //_D_con.block(4*4+2*12,0, 3*dogbot->get_n_sw(), 6)=dogbot->get_j_sw_c_bar();
        //_D_con.block(4*4+2*12,6, 3*dogbot->get_n_sw(), 12)=dogbot->get_j_sw_j_bar();
        //_D_con.block(4*4+2*12,6+12,3*dogbot->get_n_sw(),3*4)=Eigen::MatrixXd::Zero(3*dogbot->get_n_sw(), 3*4);
        //_D_con.block(4*4+2*12,6+12+3*4,3*dogbot->get_n_sw(),3*dogbot->get_n_sw())=Eigen::MatrixXd::Identity(3*dogbot->get_n_sw(),3*dogbot->get_n_sw());
    
        //_D_con.block(4*4+2*12+3*dogbot->get_n_sw(),0, 3*dogbot->get_n_sw(), 6)=-dogbot->get_j_sw_c_bar();
        //_D_con.block(4*4+2*12+3*dogbot->get_n_sw(),6, 3*dogbot->get_n_sw(), 12)=-dogbot->get_j_sw_j_bar();
        //_D_con.block(4*4+2*12+3*dogbot->get_n_sw(),6+12,3*dogbot->get_n_sw(),3*4)=Eigen::MatrixXd::Zero(3*dogbot->get_n_sw(), 3*4);
        //_D_con.block(4*4+2*12+3*dogbot->get_n_sw(),6+12+3*4,3*dogbot->get_n_sw(),3*dogbot->get_n_sw())=Eigen::MatrixXd::Identity(3*dogbot->get_n_sw(),3*dogbot->get_n_sw());
    

        _D_con_0.block(44,6,12,12)=Eigen::MatrixXd::Identity(12,12);
        _D_con_0.block(56,6,12,12)=-Eigen::MatrixXd::Identity(12,12);


        ///////////////aggiunta//////////////////////

        for(int i=0; i<4; i++){
		    _c_con_0.block(4+i*5,0,1,1)<<-20;  //-20
	    }

        /////////////////////////////////////////////

        //or anon uso più la matrice C che si presume sia stata calcolata in maniera errata
        _c_con_0.block(20,0,12,1)=_tau_max - dogbot->get_bias_com().block(6,0,12,1);
        _c_con_0.block(20+12,0,12,1)=-_tau_min + dogbot->get_bias_com().block(6,0,12,1);

        //////////////////////////////////////////////////////
        //questa sezione nel dogbot non era commentata
        
        double deltat=0.01;
        Eigen::Matrix<double,12,1> eigenq=dogbot->get_joint_pos();
	    Eigen::Matrix<double,12,1> eigendq=dogbot->get_v_c().block(6,0,12,1);
	    Eigen::Matrix<double,12,1> ddqmin=(2/pow(deltat,2))*(_qmin-eigenq-deltat*eigendq);
	    Eigen::Matrix<double,12,1> ddqmax=(2/pow(deltat,2))*(_qmax-eigenq-deltat*eigendq);

        _c_con_0.block(44,0,12,1)=ddqmax;
        _c_con_0.block(56,0,12,1)=-ddqmin;

        ///////////////////////////////////////////////////
    
        for(int i=_A_con_0.rows();i<_A_con_0.rows() + _D_con_0.rows();i++){
            for(int j=0; j<_A_con_0.cols(); j++){
                A_b_con(i,j)=_D_con_0(i-_A_con_0.rows(),j);
            }
        }
    
        //insert scalar coefficients for A/b constraint
        for(int i=0;i<_A_con_0.rows(); i++){
            A_b_con(i,_A_con_0.cols())=_b_con_0(i,0);
        }
      
        //insert scalar coefficients for C/d constraint
        for(int i=_A_con_0.rows();i<_A_con_0.rows() + _D_con_0.rows(); i++){
            A_b_con(i,_A_con_0.cols())=_c_con_0(i-_A_con_0.rows(),0);
        }
    
    
        ////////////////////////////////////////////////////////////////////////////////////

        
        ct.setlength(_A_con_0.rows() + _D_con_0.rows());
    
        for(int i=0; i<_A_con_0.rows(); i++){
            ct(i)=0.0;
        }
    
        for(int i=_A_con_0.rows(); i<_A_con_0.rows() + _D_con_0.rows(); i++){
            ct(i)=-1.0;
        }


        alglib::real_1d_array x;
        alglib::minqpstate state;
        alglib::minqpreport rep;
    
        // create solver, set quadratic/linear terms
                    
        alglib::minqpcreate(30, state);
        alglib::minqpsetquadraticterm(state, A);
        alglib::minqpsetlinearterm(state, b);
        alglib::minqpsetlc(state, A_b_con, ct);    
        alglib::minqpsetscaleautodiag(state); 
            
        alglib::minqpsetalgodenseaul(state, 1.0e-2, 1.0e+4, 5); //2 4 5

        alglib::minqpoptimize(state);
        alglib::minqpresults(state, x, rep);
        //printf("%s\n", x.tostring(1).c_str()); 

        _q_dot_dot_des_temp= Eigen::MatrixXd::Zero(12,1);        
        _f_gr_star_temp= Eigen::MatrixXd::Zero(12,1);
    
        for(int i=6; i<18; i++){
            _q_dot_dot_des_temp(i-6,0)=x[i];
        }
    
        for(int i=18;i<30;i++){
            _f_gr_star_temp(i-18,0)=x[i];
        }

        dogbot->set_q_dot_dot_des(_q_dot_dot_des_temp);

        dogbot->set_f_gr_star(_f_gr_star_temp);
    }

    if(flag==1 || flag==2){ 
        
        _A_1= _sigma_1.transpose()*dogbot->get_j_st_c_bar_1()*weight_1*dogbot->get_j_st_c_bar_1().transpose()*_sigma_1 + weight_2;
        
        //STIMATORE (termine aggiuntivo in b2_1)

        _f_ext_sw.block(0,0,3,1)=dogbot->get_f_ext().block(dogbot->get_stl1(),0,3,1);
        _f_ext_sw.block(3,0,3,1)=dogbot->get_f_ext().block(dogbot->get_stl2(),0,3,1);
    
        _b2_1= -_sigma_1.transpose()*dogbot->get_j_st_c_bar_1()*weight_1.transpose()*dogbot->get_w_com_des() + _sigma_1.transpose()*dogbot->get_j_st_c_bar_1()*weight_1.transpose()*dogbot->get_j_st_c_bar_1().transpose()*_f_ext_sw;
            
        //cost function
        
        A.setlength(_A_1.rows(),_A_1.cols());
        
    
        for(int i=0;i<_A_1.rows();i++){
            for(int j=0; j<_A_1.cols();j++){
                A(i,j)=_A_1(i,j);
            }
        }


        b.setlength(_b2_1.rows());
        for(int i=0; i<_b2_1.rows(); i++){
            b(i)=_b2_1(i,0);//_b1_1(0,i);//+_b2_1(i,0);
        }
      
    
        ///////////////////////////////////////////////////////////////
    
        //////////////////////////////////////////////////////////////
        //CONSTRAINTS('con' stands for the matrix which are passed to alglib script)
    
        _A_con_1.block(0,0,6,6)=dogbot->get_mass_matrix().block(0,0,6,6);
        //_A_con_1.block(0,6,6,12)=Eigen::MatrixXd::Zero(6,12);
        _A_con_1.block(0,18,6,3*2)=-dogbot->get_j_st_c_bar_1().transpose();
        //_A_con_1.block(0,6+12+3*2,6,3*2)=Eigen::MatrixXd::Zero(6,3*2);
    
        _A_con_1.block(6,0,3*2,6)=dogbot->get_j_st_c_bar_1();
        _A_con_1.block(6,6,3*2,12)=dogbot->get_j_st_j_bar_1();
        //_A_con_1.block(6,6+12,3*2,3*2)=Eigen::MatrixXd::Zero(3*2, 3*2);
        //_A_con_1.block(6,6+12+3*2,3*2,3*2)=Eigen::MatrixXd::Zero(3*2,3*2);
    
        
        ////////////////////////////////////////////////////////////////
        //STIMATORE (introduco fext_st)

        _JacCOM_lin=_B_opt*dogbot->get_j_bar();

        _Jstcom_sw.block(0,0,3,6)= _JacCOM_lin.block(dogbot->get_stl1(),0,3,6);
	    _Jstcom_sw.block(3,0,3,6)= _JacCOM_lin.block(dogbot->get_stl1(),0,3,6);
        
	    
        _b_con_1.block(0,0,6,1)=-dogbot->get_bias_com().block(0,0,6,1) + _Jstcom_sw.transpose()*_f_ext_sw;

        //std::cerr<<"b_con_1: "<<std::endl;
        //std::cerr<<_Jstcom_sw.transpose()*_f_ext_sw<<std::endl;

        ///////////////////////////////////////////////////////////////

        //_b_con_1.block(0,0,6,1)=-dogbot->get_mass_matrix()(0,0)*g;
    
        _b_con_1.block(6,0,6,1)=-dogbot->get_j_st_dot_1();

        //_b_con_1.block(6,0,6,1)=-dogbot->get_j_st_dot_1().block(0,0,6,6)*dogbot->get_v_c().block(0,0,6,1) - dogbot->get_j_st_dot_1().block(0,6,6,12)*dogbot->get_v_c().block(6,0,12,1);
        
        //put A and b together
        
        A_b_con.setlength(_A_con_1.rows() + _D_con_1.rows(), _A_con_1.cols()+1);
    
        for(int i=0;i<_A_con_1.rows();i++){
            for(int j=0; j<_A_con_1.cols();j++){
                A_b_con(i,j)=_A_con_1(i,j);
            }
        }  
    

        //_D_con_1.block(0,0,4*2,6)=Eigen::MatrixXd::Zero(4*2,6);
        //_D_con_1.block(0,6,4*2,12)=Eigen::MatrixXd::Zero(4*2, 12);
        _D_con_1.block(0,6+12,5*2, 3*2)=_d_fr_1;
        //_D_con_1.block(0,6+12+3*2,4*2,3*2)=Eigen::MatrixXd::Zero(4*2, 3*2);
    
        //_D_con_1.block(4*2,0,12,6)=Eigen::MatrixXd::Zero(12,6);
        _D_con_1.block(5*2,6,12,12)=dogbot->get_mass_matrix().block(6,6,12,12);
        _D_con_1.block(5*2,6+12,12,3*2)=-dogbot->get_j_st_j_bar_1().transpose();
        //_D_con_1.block(4*2,6+12+3*2,12, 3*2)=Eigen::MatrixXd::Zero(12, 3*2);
    
        //_D_con_1.block(4*2+12,0,12,6)=Eigen::MatrixXd::Zero(12,6);
        _D_con_1.block(5*2+12,6,12,12)=-dogbot->get_mass_matrix().block(6,6,12,12);
        _D_con_1.block(5*2+12,6+12,12,3*2)=dogbot->get_j_st_j_bar_1().transpose();
        //_D_con_1.block(4*2+12,6+12+3*2,12, 3*2)=Eigen::MatrixXd::Zero(12, 3*2);
    
        _D_con_1.block(5*2+2*12,0, 3*2, 6)=dogbot->get_j_sw_c_bar();
        _D_con_1.block(5*2+2*12,6, 3*2, 12)=dogbot->get_j_sw_j_bar();
        //_D_con_1.block(4*2+2*12,6+12,3*2,3*2)=Eigen::MatrixXd::Zero(3*2, 3*2);
        _D_con_1.block(5*2+2*12,6+12+3*2,3*2,3*2)=Eigen::MatrixXd::Identity(3*2,3*2);
    
        _D_con_1.block(5*2+2*12+3*2,0, 3*2, 6)=-dogbot->get_j_sw_c_bar();
        _D_con_1.block(5*2+2*12+3*2,6, 3*2, 12)=-dogbot->get_j_sw_j_bar();
        //_D_con_1.block(4*2+2*12+3*2,6+12,3*2,3*2)=Eigen::MatrixXd::Zero(3*2, 3*2);
        _D_con_1.block(5*2+2*12+3*2,6+12+3*2,3*2,3*2)=-Eigen::MatrixXd::Identity(3*2,3*2);   

        _D_con_1.block(46,6,12,12)=Eigen::MatrixXd::Identity(12,12);
        _D_con_1.block(58,6,12,12)=-Eigen::MatrixXd::Identity(12,12);

        //AGGIUNTA (non serve)

        //for(int i=0; i<2; i++){
		//    _c_con_1.block(4+i*5,0,1,1)<<-20;
	    //}

        //////////////////////////////////////////////////////////////////
        //STIMATORE (introduco fext_lambda)

        _Jst.block(0,0,3,18)=_JacCOM_lin.block(dogbot->get_stl1(),0,3,18);
        _Jst.block(3,0,3,18)=_JacCOM_lin.block(dogbot->get_stl2(),0,3,18);

        _P=Eigen::Matrix<double,18,18>::Identity()-_Jst.transpose()*(_Jst*dogbot->get_mass_matrix().inverse()*_Jst.transpose()).inverse()*_Jst*dogbot->get_mass_matrix().inverse();
    
	    _fext_lambda<<_JacCOM_lin.block(dogbot->get_swl1(),0,3,18)*dogbot->get_mass_matrix().inverse()*_Si.transpose()*_P*_JacCOM_lin.transpose()*dogbot->get_f_ext(),
	                  _JacCOM_lin.block(dogbot->get_swl2(),0,3,18)*dogbot->get_mass_matrix().inverse()*_Si.transpose()*_P*_JacCOM_lin.transpose()*dogbot->get_f_ext();

        //_c_con_1.block(0,0,4*2,1)=Eigen::MatrixXd::Zero(4*2,1);
        _c_con_1.block(5*2,0,12,1)=_tau_max - dogbot->get_bias_com().block(6,0,12,1);
        _c_con_1.block(5*2+12,0,12,1)=-_tau_min + dogbot->get_bias_com().block(6,0,12,1);
        _c_con_1.block(5*2+2*12,0,3*2,1)=_x_sw_cmd_dot_dot - dogbot->get_j_sw_dot() - _fext_lambda;  //.block(0,0,6,6)*dogbot->get_v_c().block(0,0,6,1) - dogbot->get_j_sw_dot().block(0,6,6,12)*dogbot->get_v_c().block(6,0,12,1);
        _c_con_1.block(5*2+2*12+3*2,0,3*2,1)=-_x_sw_cmd_dot_dot + dogbot->get_j_sw_dot() + _fext_lambda; //.block(0,0,6,6)*dogbot->get_v_c().block(0,0,6,1) + dogbot->get_j_sw_dot().block(0,6,6,12)*dogbot->get_v_c().block(6,0,12,1);
    
        //std::cerr<<"f_ext_lambda: "<<std::endl;
        //std::cerr<<_fext_lambda<<std::endl;
        
        //////////////////////////////////////////////////
        //questa sezione nel dogbot non era commentata (vincoli sull'accelerazione)

        double deltat=0.01;
        Eigen::Matrix<double,12,1> eigenq=dogbot->get_joint_pos();
	    Eigen::Matrix<double,12,1> eigendq=dogbot->get_v_c().block(6,0,12,1);
	    
	    Eigen::Matrix<double,12,1> ddqmin=(2/pow(deltat,2))*(_qmin-eigenq-deltat*eigendq);
	    Eigen::Matrix<double,12,1> ddqmax=(2/pow(deltat,2))*(_qmax-eigenq-deltat*eigendq);

        _c_con_1.block(46,0,12,1)=ddqmax;
        _c_con_1.block(58,0,12,1)=-ddqmin;

        //////////////////////////////////////////////////////
        
        
        
        for(int i=_A_con_1.rows();i<_A_con_1.rows() + _D_con_1.rows();i++){
            for(int j=0; j<_A_con_1.cols();j++){
                A_b_con(i,j)=_D_con_1(i-_A_con_1.rows(),j);
            }
        }
    
        //insert scalar coefficients for A/b constraint
        for(int i=0;i<_A_con_1.rows(); i++){
            A_b_con(i,_A_con_1.cols())=_b_con_1(i,0);
        }
      
        //insert scalar coefficients for C/d constraint
        for(int i=_A_con_1.rows();i<_A_con_1.rows() + _D_con_1.rows(); i++){
            A_b_con(i,_A_con_1.cols())=_c_con_1(i-_A_con_1.rows(),0);
        }
    
        ////////////////////////////////////////////////////////////////////////////////////
       
        
        ct.setlength(_A_con_1.rows() + _D_con_1.rows());
    
        for(int i=0; i<_A_con_1.rows(); i++){
            ct(i)=0.0;
        }
    
    
    
        for(int i=_A_con_1.rows(); i<_A_con_1.rows() + _D_con_1.rows(); i++){
            ct(i)=-1.0;
        }

        alglib::real_1d_array x;
        alglib::minqpstate state;
        alglib::minqpreport rep;

        // create solver, set quadratic/linear terms
        alglib::minqpcreate(30, state);
        alglib::minqpsetquadraticterm(state, A);
        alglib::minqpsetlinearterm(state, b);
        alglib::minqpsetlc(state, A_b_con, ct);
        alglib::minqpsetscaleautodiag(state);
                
        alglib::minqpsetalgodenseaul(state, 1.0e-2, 1.0e+4, 5); 

        alglib::minqpoptimize(state);
        alglib::minqpresults(state, x, rep);
        //printf("%s\n", x.tostring(1).c_str());        
    
    
        
        _q_dot_dot_des_temp= Eigen::MatrixXd::Zero(12,1);
    
        for(int i=6; i<18; i++){
            _q_dot_dot_des_temp(i-6,0)=x[i];
        }

        dogbot->set_q_dot_dot_des(_q_dot_dot_des_temp);

        //prendo solo front right e back left (piedi a contatto con il suolo)
        if(flag==1){
            
            _f_gr_star_temp= Eigen::MatrixXd::Zero(12,1);

            //front right
            _f_gr_star_temp(0,0)=x[18];
            _f_gr_star_temp(1,0)=x[19];
            _f_gr_star_temp(2,0)=x[20];     
        
            //back left
            _f_gr_star_temp(9,0)=x[21];
            _f_gr_star_temp(10,0)=x[22];
            _f_gr_star_temp(11,0)=x[23];

            dogbot->set_f_gr_star(_f_gr_star_temp);
            
            //std::cerr<<"ottimo slack: "<<std::endl;
    
            //std::cerr<<x[24]<<std::endl;
            //std::cerr<<x[25]<<std::endl;
            //std::cerr<<x[26]<<std::endl;
            //std::cerr<<x[27]<<std::endl;
            //std::cerr<<x[28]<<std::endl;
            //std::cerr<<x[29]<<std::endl;
            
        }

        if(flag==2){
            
            _f_gr_star_temp= Eigen::MatrixXd::Zero(12,1); 

            //back right
            _f_gr_star_temp(3,0)=x[18];
            _f_gr_star_temp(4,0)=x[19];
            _f_gr_star_temp(5,0)=x[20];     
        
            //front left
            _f_gr_star_temp(6,0)=x[21];
            _f_gr_star_temp(7,0)=x[22];
            _f_gr_star_temp(8,0)=x[23];

            dogbot->set_f_gr_star(_f_gr_star_temp);
            
        }
    
        
    }
   
    
}    




void OPTIMAL::initialization(){

    _d_fr_0=Eigen::MatrixXd::Zero(20,12);

    _d_fr_1=Eigen::MatrixXd::Zero(10,6);

    _sigma_0.block(0,0,12,18)=Eigen::MatrixXd::Zero(12,18);
    _sigma_0.block(0,18,12,12)=Eigen::MatrixXd::Identity(12,12);

    _sigma_1.block(0,0,6,30)=Eigen::MatrixXd::Zero(6,30);
    _sigma_1.block(0,18,6,6)=Eigen::MatrixXd::Identity(6,6);

    _A_0=Eigen::MatrixXd::Zero(30,30);

    _A_1=Eigen::MatrixXd::Zero(30,30);

    _b1_0=Eigen::MatrixXd::Zero(1,30);

    _b1_1=Eigen::MatrixXd::Zero(1,30);

    _b2_0=Eigen::MatrixXd::Zero(30,1);

    _b2_1=Eigen::MatrixXd::Zero(30,1);

    _A_con_0=Eigen::MatrixXd::Zero(18,30);
    

    _A_con_1=Eigen::MatrixXd::Zero(12,30);

    _b_con_0=Eigen::MatrixXd::Zero(18,1);

    _b_con_1=Eigen::MatrixXd::Zero(12,1);

    _D_con_0=Eigen::MatrixXd::Zero(68,30);

    _D_con_1=Eigen::MatrixXd::Zero(70,30);

    _c_con_0=Eigen::MatrixXd::Zero(68,1);

    _c_con_1=Eigen::MatrixXd::Zero(70,1);

    for(int i=0; i<4; i++){
        _tau_max(i,0)=20.0;
        _tau_min(i,0)=-20.0;
    }

    for(int i=4; i<12; i++){
        _tau_max(i,0)=55.0;
        _tau_min(i,0)=-55.0;
    }

    _x_sw_cmd_dot_dot=Eigen::MatrixXd::Zero(6,1);

    _t1 << 1, 0, 0;
    _t2 << 0, 1, 0;
    _n  << 0, 0, 1;

    g=Eigen::MatrixXd::Zero(6,1);
    g(2,0)=9.81;

    weight_1=50*Eigen::MatrixXd::Identity(6,6); //50

    weight_2=Eigen::MatrixXd::Identity(30,30);
    weight_2.block(24,24,6,6)=20*Eigen::MatrixXd::Identity(6,6); //20

    weight_3=50*Eigen::MatrixXd::Identity(6,6);

    _qmin<<-0.802, -0.802, -0.802, -0.802, -1.047,  -2.696, -1.047,  -2.696, -1.047,  -2.696, -1.047,  -2.696; //qui ho messo 3.14 al posto di 9.42
    
    _qmax<< 0.802,  0.802,  0.802,  0.802,  4.188,  -0.916,  4.188,  -0.916,  4.188,  -0.916,  4.188,  -0.916; //qui ho messo 3.14 al posto di 9.42

    _B_opt<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
             Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
    	     Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
    	     Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);

    _fext_lambda = Eigen::Matrix<double,6,1>::Zero();

    _Si<<Eigen::Matrix<double,6,18>::Zero(),
	     Eigen::Matrix<double,12,6>::Zero(),Eigen::Matrix<double,12,12>::Identity();

    _Jst= Eigen::Matrix<double,6,18>::Zero();

    _Jstcom_sw= Eigen::Matrix<double,6,6>::Zero();

    

}


