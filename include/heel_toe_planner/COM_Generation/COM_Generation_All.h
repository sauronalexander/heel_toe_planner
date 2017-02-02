#ifndef COM_GENERATION_ALL_H
#define COM_GENERATION_ALL_H

#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <heel_toe_planner/COM_Generation/rtwtypes.h>
#include <heel_toe_planner/COM_Generation/rt_defines.h>
#include <heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_types.h>
#include <heel_toe_planner/COM_Generation/COM_Boundary_Condition.h>
#include <heel_toe_planner/COM_Generation/COM_Generation.h>
#include <heel_toe_planner/COM_Generation/Cosine_Cosine_Interpolation.h>
#include <heel_toe_planner/COM_Generation/fileManager.h>
#include <heel_toe_planner/COM_Generation/Forward_Kinematic.h>
#include <heel_toe_planner/COM_Generation/fprintf.h>
#include <heel_toe_planner/COM_Generation/Gait_Generation.h>
#include <heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_emxAPI.h>
#include <heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_emxutil.h>
#include <heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_initialize.h>
#include <heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_rtwutil.h>
#include <heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_terminate.h>
#include <heel_toe_planner/COM_Generation/mldivide.h>
#include <heel_toe_planner/COM_Generation/Quadratic_Cosine_Interpolation.h>
#include <heel_toe_planner/COM_Generation/rtGetInf.h>
#include <heel_toe_planner/COM_Generation/rtGetNaN.h>
#include <heel_toe_planner/COM_Generation/rt_nonfinite.h>
#include <heel_toe_planner/COM_Generation/X_DS_Differential_Equa_Solver.h>
#include <heel_toe_planner/COM_Generation/X_SS_Differential_Equa_Solver.h>
#include <heel_toe_planner/COM_Generation/Y_Correction.h>
#include <heel_toe_planner/COM_Generation/Y_DS_Differential_Equa_Solver.h>
#include <heel_toe_planner/COM_Generation/Y_End.h>
#include <heel_toe_planner/COM_Generation/Y_SS_Differential_Equa_Solver.h>
#include <heel_toe_planner/COM_Generation/ZMP_Boundary_Condition.h>
#include <heel_toe_planner/COM_Generation/ZMP_END_FITTING.h>
#include <heel_toe_planner/COM_Generation/ZMP_Generation.h>

class COM_Generation
{
	private:
		emxArray_real_T * leftgait_x;
		emxArray_real_T * rightgait_x;
		emxArray_real_T * leftgait_y;
		emxArray_real_T * rightgait_y;
		emxArray_real_T * zmpUB_x;
		emxArray_real_T * zmpLB_x;
		emxArray_real_T * zmpUB_y;
		emxArray_real_T * zmpLB_y;
		emxArray_real_T * zmpLB_x_t;
		emxArray_real_T * zmpUB_x_t;
        emxArray_real_T * delta_y;
        emxArray_real_T * coef_x;
        emxArray_real_T * coef_y;

        double * x_init_pframe;
        double * x_init_heel_pframe;
        double * x_end_pframe;
        double * x_end_heel_pframe;

        double totaltime;
        double tinit;
        double tend;
        double tstep;
        double ratio;

        double wfoot;

        double htar;
        double lstep;
        double lfoot;
        double lthigh;
        double lshank;


        //Mass
        double Mtot;
        double m1;
        double m2;
        double m3;
        double g;

        //Error Estimation
        double zi_p;
        double ze_p;

    protected:
        emxArray_real_T * x_left;
        emxArray_real_T * x_right;
        emxArray_real_T * x_trunk;
        emxArray_real_T * y_trunk;
        emxArray_real_T * z_left;
        emxArray_real_T * z_right;
        emxArray_real_T * theta_left;
        emxArray_real_T * theta_right;
        emxArray_real_T * zmp_x;
        emxArray_real_T * zmp_y;
        double z2;
        double zi;
        double ze;
        double theta_start;
        double theta_end;
        double * theta_i;
        double * theta_e;
        double w;
        double Period;

	public:

		COM_Generation();
        void Set_Parameters(double totaltime, double tinit, double tend, double tstep,
                            double ratio, double Period, double lstep, double z2);
        void Set_Robot_Parameters(double lthigh, double lshank, double htar, double lfoot,
                                  double w, double wfoot);
        virtual void Set_Mass(double Mtot, double m1, double m2, double m3, double g);
        void Generate_COM();
        virtual void Error();
        virtual ~COM_Generation();
};

#endif // COM_GENERATION_ALL_H
