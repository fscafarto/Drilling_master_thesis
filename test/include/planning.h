#ifndef PLANNING_H
#define PLANNING_H


//#include "ros/ros.h"
#include "boost/thread.hpp"
#include <ifopt/ipopt_solver.h>
#include <towr/initialization/gait_generator.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>

#include "anymals_like.h"



class QUADRUPED;


class PLANNING{
    public:

    PLANNING(QUADRUPED &quadruped);

    void compute_planning(int flag);

    towr::SplineHolder get_solution();

    towr::NlpFormulation get_formulation();

   


    private:

    QUADRUPED*  dogbot;

    towr::SplineHolder _solution;

    towr::NlpFormulation _formulation;

    double _x_temp=0.0;
    double _y_temp=0.0;
    double _z_temp=0.0;
    double _yaw_temp=0.0;

    bool _first_cycle_plan=false;

    double _com_x, _com_y, _com_z, _com_pitch;

    //double _offset = 0.0; //prima_1, prima_2, prima_3, prima_4, prima_5, prima_6, prima_7

    //double _offset = 0.03; //prima_8

    //double _offset = 0.05; //prima_9, prima_10, prima_11, quarta_1_wall, quinta_1_wall

    double _offset = 0.01; //terza_1, terza_1_lunga, terza_1_wall, combo_1_wall, ottava_1_wall, nona_1_wall

    double _offset_pitch = 0.0;

    double _offset_z = 0.0;




};

#endif