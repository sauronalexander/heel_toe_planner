#include <heel_toe_planner/COM_Generation/COM_Generation_All.h>

COM_Generation::COM_Generation()
{
    Heel_Toe_Planner_Part1_initialize();

    leftgait_x = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    rightgait_x = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    leftgait_y = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    rightgait_y = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));

    zmpUB_x = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    zmpLB_x = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    zmpUB_y = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    zmpLB_y = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    zmpLB_x_t = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    zmpUB_x_t = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));

    zmp_x = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    zmp_y = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    delta_y = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    coef_x = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    coef_y = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));

    theta_i = new double[3];
    theta_e = new double[4];
    x_init_pframe = new double[2];
    x_init_heel_pframe = new double[2];
    x_end_pframe = new double[2];
    x_end_heel_pframe = new double[2];

    x_left = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    x_right = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    x_trunk = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    y_trunk = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    z_left = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    z_right = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    theta_left = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    theta_right = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));

    Initial_emxArry(leftgait_x);
    Initial_emxArry(rightgait_x);
    Initial_emxArry(leftgait_y);
    Initial_emxArry(rightgait_y);

    Initial_emxArry(zmpUB_x);
    Initial_emxArry(zmpLB_x);
    Initial_emxArry(zmpUB_y);
    Initial_emxArry(zmpLB_y);
    Initial_emxArry(zmpLB_x_t);
    Initial_emxArry(zmpUB_x_t);

    Initial_emxArry(zmp_x);
    Initial_emxArry(zmp_y);
    Initial_emxArry(delta_y);
    Initial_emxArry(coef_x);
    Initial_emxArry(coef_y);

    Initial_emxArry(x_left);
    Initial_emxArry(x_right);
    Initial_emxArry(x_trunk);
    Initial_emxArry(y_trunk);
    Initial_emxArry(z_left);
    Initial_emxArry(z_right);
    Initial_emxArry(theta_left);
    Initial_emxArry(theta_right);
}

void COM_Generation::Set_Parameters(double totaltime, double tinit, double tend, double tstep,
                                    double ratio, double Period, double lstep, double z2)
{
    this->totaltime = totaltime;
    this->tinit = tinit;
    this->tend = tend;
    this->tstep = tstep;
    this->ratio = ratio;
    this->Period = Period;
    this->lstep = lstep;
    this->z2 = z2;
}

void COM_Generation::Set_Robot_Parameters(double lthigh, double lshank, double htar,
                                          double lfoot, double w, double wfoot)
{
    this->lthigh = lthigh;
    this->lshank = lshank;
    this->w = w;
    this->wfoot = wfoot;
    this->htar = htar;
    this->lfoot = lfoot;

}

void COM_Generation::Set_Mass(double Mtot, double m1, double m2, double m3, double g)
{
    this->Mtot = Mtot;
    this->m1 = m1;
    this->m2 = m2;
    this->m3 = m3;
    this->g = g;
}

void COM_Generation::Generate_COM()
{
    //First Generate Gaits
    Gait_Generation(totaltime, tinit, tend, tstep, ratio, Period, lstep, 0.0, w, 0.0,
                    leftgait_x, rightgait_x, leftgait_y, rightgait_y);
    ROS_INFO("Gait Generation Complete");

    ZMP_Boundary_Condition(totaltime, tinit, tend, tstep, ratio, Period, 0.0, lfoot, w, wfoot, leftgait_x,
                           rightgait_x, leftgait_y, rightgait_y, zmpUB_x, zmpLB_x, zmpUB_y, zmpLB_y,
                           zmpUB_x_t, zmpLB_x_t);
    ROS_INFO("ZMP Boundary Generation Complete");

    ZMP_Generation(zmpLB_x_t, zmpUB_x_t, totaltime, tinit, tend, tstep, ratio, Period, lstep, lfoot,
                   w, wfoot, htar, zmp_x, zmp_y, delta_y, coef_x, coef_y);
    ROS_INFO("ZMP Generation Complete");

    COM_Boundary_Condition(lthigh, lshank, lfoot, htar, lstep, z2, x_init_pframe, x_init_heel_pframe,
                           x_end_pframe, x_end_heel_pframe, theta_i, theta_e);
    ROS_INFO("COM Boundary Generation Complete");

    ze_p = z2 + x_end_pframe[1];
    zi_p = z2 + x_init_pframe[1];
    ze = 0.5*lfoot*sin(theta_e[0]+theta_e[1]-theta_e[2]);
    zi = -0.5*lfoot*sin(theta_i[0]+theta_i[1]);
    theta_start = -1.0*(theta_e[0]+theta_e[1]-theta_e[2]);
    theta_end = -1.0*(theta_i[0]+theta_i[1]);


    ROS_INFO_STREAM("ze: "<<ze<<"; zi: "<<zi);

    b_COM_Generation(totaltime, tinit, tend, tstep, ratio, Period, lstep, lfoot, w, wfoot, htar, Mtot,
                   m1, m2, m3, g, zmp_x, zmp_y, delta_y, coef_x, coef_y, x_init_pframe, x_init_heel_pframe,
                   x_end_pframe, x_end_heel_pframe, theta_i, theta_e, zmpUB_x_t, zmpLB_x_t,
                   zmpUB_x, zmpLB_x, zmpUB_y, zmpLB_y, leftgait_x, rightgait_x, leftgait_y, rightgait_y,
                   z2, ze, zi, x_left, x_right, x_trunk, y_trunk, z_left, z_right, theta_left, theta_right);

}

void COM_Generation::Error()
{
    ROS_WARN_STREAM("ERROR TERMS:");
    ROS_WARN_STREAM("zi error: "<<zi_p - zi<<"; ze error: "<<ze_p - ze);
}

COM_Generation::~COM_Generation()
{
    Delete_emxArry(leftgait_x);
    Delete_emxArry(rightgait_x);
    Delete_emxArry(leftgait_y);
    Delete_emxArry(rightgait_y);

    Delete_emxArry(zmpUB_x);
    Delete_emxArry(zmpLB_x);
    Delete_emxArry(zmpUB_y);
    Delete_emxArry(zmpLB_y);
    Delete_emxArry(zmpLB_x_t);
    Delete_emxArry(zmpUB_x_t);

    Delete_emxArry(zmp_x);
    Delete_emxArry(zmp_y);
    Delete_emxArry(delta_y);
    Delete_emxArry(coef_x);
    Delete_emxArry(coef_y);

    delete theta_i;
    delete theta_e;
    delete x_init_pframe;
    delete x_init_heel_pframe;
    delete x_end_pframe;
    delete x_end_heel_pframe;

    Delete_emxArry(x_left);
    Delete_emxArry(x_right);
    Delete_emxArry(x_trunk);
    Delete_emxArry(y_trunk);
    Delete_emxArry(z_left);
    Delete_emxArry(z_right);
    Delete_emxArry(theta_left);
    Delete_emxArry(theta_right);

    leftgait_x = NULL;
    rightgait_x = NULL;
    leftgait_y = NULL;
    rightgait_y = NULL;

    zmpUB_x = NULL;
    zmpLB_x = NULL;
    zmpUB_y = NULL;
    zmpLB_y = NULL;
    zmpLB_x_t = NULL;
    zmpUB_x_t = NULL;

    zmp_x = NULL;
    zmp_y = NULL;
    delta_y = NULL;
    coef_x = NULL;
    coef_y = NULL;

    theta_i = NULL;
    theta_e = NULL;
    x_init_pframe = NULL;
    x_init_heel_pframe = NULL;
    x_end_pframe = NULL;
    x_end_heel_pframe = NULL;

    x_left = NULL;
    x_right = NULL;
    x_trunk = NULL;
    y_trunk = NULL;
    z_left = NULL;
    z_right = NULL;
    theta_left = NULL;
    theta_right = NULL;
}
