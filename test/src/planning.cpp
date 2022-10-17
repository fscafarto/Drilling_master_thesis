#include "planning.h"

using namespace towr;
using namespace std;



PLANNING::PLANNING(QUADRUPED &quadruped)

{
    dogbot = &quadruped;
}


void PLANNING::compute_planning(int flag){
    
    auto gategen = towr::GaitGenerator::MakeGaitGenerator(4);

    towr::NlpFormulation formulation;
    _formulation=formulation;

    _formulation.terrain_ = std::make_shared<towr::FlatGround>(0.0); 
    
    _formulation.model_ = towr::RobotModel(towr::RobotModel::Anymal);  

    auto stance = _formulation.model_.kinematic_model_->GetNominalStanceInBase();
    _formulation.initial_ee_W_ = stance; 

      
    _formulation.initial_base_.lin.at(towr::kPos) << dogbot->get_init_pos()(0,0), dogbot->get_init_pos()(1,0), dogbot->get_init_pos()(2,0);
    _formulation.initial_base_.ang.at(towr::kPos) << dogbot->get_init_pos()(3,0), dogbot->get_init_pos()(4,0), dogbot->get_init_pos()(5,0);
    _formulation.initial_base_.lin.at(towr::kVel) << dogbot->get_init_vel()(0,0), dogbot->get_init_vel()(1,0), dogbot->get_init_vel()(2,0); 
    _formulation.initial_base_.ang.at(towr::kVel) << dogbot->get_init_vel()(3,0), dogbot->get_init_vel()(4,0), dogbot->get_init_vel()(5,0); 


    //questa istruzione mette a zero la componente z di formulation initial
    std::for_each(_formulation.initial_ee_W_.begin(), _formulation.initial_ee_W_.end(),
                  [&](Eigen::Vector3d& p){ p.z() = 0.0; });

    
    if(!_first_cycle_plan){
        _com_x=_formulation.initial_base_.lin.at(towr::kPos)[0];
        _com_y=_formulation.initial_base_.lin.at(towr::kPos)[1];
        _com_z=_formulation.initial_base_.lin.at(towr::kPos)[2];

        _com_pitch=_formulation.initial_base_.ang.at(towr::kPos)[1];

        _first_cycle_plan=true;
    }

    //walking (IMPORTANTE! NON ELIMINARE)
    //_formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0]+0.02, 0.0, _com_z;
    //_formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, 0.0;

    ///////////////////////////////////////////////
    //prima_1, prima_2, prima_3, prima_4, prima_5, prima_6, prima_7

    //if( dogbot->get_do_replanning() ){
    //    _offset += 0.005;
    //    _formulation.final_base_.lin.at(towr::kPos) << _com_x+_offset,  _com_y, _com_z;
    //    _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, 0.0; //0.0 (prima prova) //0.03 (seconda prova) //0.08726 (terza prova) //0.0 (quarta prova) //qui cambio il pitch 
    //    std::cerr<<_offset<<std::endl;
    //}else{
    //    _formulation.final_base_.lin.at(towr::kPos) << _com_x+_offset,  _com_y, _com_z;
    //    _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, 0.0; 
    //}

    //////////////////////////////////////////
    //prima_8, prima_9, prima_10, prima_11, seconda_1, terza_1, terza_1_lunga, quarta_1_wall, quinta_1_wall

    //_formulation.final_base_.lin.at(towr::kPos) << _com_x+_offset,  _com_y, _com_z;
    //_formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, 0.0;

    //////////////////////////////////////////////

    //////////////////////////////////////////////
    //terza_1_wall (sarebbe la terza_1 definitiva), combo_1_wall, ottava_1_wall, nona_1_wall

    if( _formulation.initial_base_.ang.at(towr::kPos)[1] > 0.08726){
        _offset_pitch += 0.01745;
    }else{
        _offset_pitch = 0.0;
    }

    _formulation.final_base_.lin.at(towr::kPos) << _com_x+_offset,  _com_y, _com_z;
    _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.08726 - _offset_pitch, 0.0;

    ///////////////////////////////////////////

    //if(_step_2){
    //    _formulation.final_base_.lin.at(towr::kPos) << _formulation.initial_base_.lin.at(towr::kPos)[0],_formulation.initial_base_.lin.at(towr::kPos)[1], _formulation.initial_base_.lin.at(towr::kPos)[2];
    //    _formulation.final_base_.ang.at(towr::kPos) << 0.0, 0.0, _formulation.initial_base_.ang.at(towr::kPos)[2]+(1.0*M_PI)/180;
    //}

    //qui ci vogliono le istruzioni stance

    
    //questo ciclo for assegna le componenti x e y di getpos alle rispettive componenti x e y di formulation initial
    Eigen::Vector3d pos_ee;
    for (int ee=0; ee<4; ee++){

        switch(ee){
            case 0: pos_ee=dogbot->getFLpos(); //FL
            break;
            case 1: pos_ee=dogbot->getFRpos(); //FR
            break;
            case 2: pos_ee=dogbot->getBLpos(); //BL
            break;
            case 3: pos_ee=dogbot->getBRpos(); //BR
            break;
        }

        _formulation.initial_ee_W_.at(ee)[0]=pos_ee[0];
        _formulation.initial_ee_W_.at(ee)[1]=pos_ee[1];
    }

    

    if(flag==0){
        auto gait_type = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C5);
        gategen->SetCombo(gait_type);
    }
    else if(flag==1){
        auto gait_type = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C6);
        gategen->SetCombo(gait_type);
    }
    else if(flag==2){
        auto gait_type = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C7);
        gategen->SetCombo(gait_type);
    }
       

    //gategen->SetGaits({_stand, _run}); // Stand + Run1

    _formulation.params_.ee_phase_durations_.clear();
    for(int i=0;i<4;++i){
        _formulation.params_.ee_phase_durations_.push_back(gategen->GetPhaseDurations(0.4,i)); 
  	    _formulation.params_.ee_in_contact_at_start_.push_back(gategen->IsInContactAtStart(i));
    }


    ifopt::Problem nlp;

    towr::SplineHolder solution;

    _solution=solution;
    
    //Init nonlinear programming with vars, constraints and costs
    for(auto c: _formulation.GetVariableSets(_solution))
    nlp.AddVariableSet(c);

    for(auto c: _formulation.GetConstraints(_solution))
    nlp.AddConstraintSet(c);

    for(auto c: _formulation.GetCosts())
    nlp.AddCostSet(c);


    auto solver = std::make_shared<ifopt::IpoptSolver>();

    //solver->SetOption("derivative_test", "first-order");
    solver -> SetOption("jacobian_approximation","exact");
    //solver -> SetOption("linear_solver", "mumps");
    solver -> SetOption("max_cpu_time",20.0); 
    //solver -> SetOption("max_iter", 3000);
    //solver -> SetOption("print_level", 5);
    solver -> Solve(nlp);
	
}

towr::NlpFormulation PLANNING::get_formulation(){
    return _formulation;
}

towr::SplineHolder PLANNING::get_solution(){
    return _solution;
}



