#include <iostream>
#include <ros/ros.h>
#include <heel_toe_planner/COM_Generation/COM_Generation_All.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "Test_Com_Gen");
    ros::NodeHandle nh;

    double * temp;

    //Gait Generation
    double totaltime = 11.5, tinit = 0.7, tend = 0.8, tstep = 1.0, ratio = 0.1, w = 0.4, wfoot = 0.1;
    double Period = 0.01, htar = 0.2, lstep = 0.5, lfoot = 0.2, lthigh = 0.35, lshank = 0.35, z2 = 0.69;
    double Mtot = 80.0;
    double m1 = Mtot/5.0;
    double m3 = Mtot/5.0;
    double m2 = Mtot - m1 - m3;
    double g = 9.8;

    Heel_Toe_Planner_Part1_initialize();

    emxArray_real_T * leftgait_x = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * rightgait_x = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * leftgait_y = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * rightgait_y = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));

    Initial_emxArry(leftgait_x);
    Initial_emxArry(rightgait_x);
    Initial_emxArry(leftgait_y);
    Initial_emxArry(rightgait_y);

    test_gait();
    Gait_Generation(totaltime, tinit, tend, tstep, ratio, Period, lstep, 0.0, w, 0.0,
                    leftgait_x, rightgait_x, leftgait_y, rightgait_y);

//    std::cout<<"Left_X: "<<(leftgait_x->size[1])<<std::endl;
//    double * temp = leftgait_x->data;
//    for(int i=0; i<(leftgait_x->size[1]); i++)
//    {
//        std::cout<<(*temp)<<", ";
//        temp++;
//        if(i%10 == 0)
//            std::cout<<std::endl;
//    }

//    std::cout<<"Left_Y: "<<(leftgait_y->size[1])<<std::endl;
//    temp = leftgait_y->data;
//    for(int i=0; i<(leftgait_y->size[1]); i++)
//    {
//        std::cout<<(*temp)<<", ";
//        temp++;
//        if(i%10 == 0)
//            std::cout<<std::endl;
//    }
    std::cout<<"Gait Generation Test Done"<<std::endl;

    //ZMP Boundary Condition
    emxArray_real_T * zmpUB_x = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * zmpLB_x = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * zmpUB_y = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * zmpLB_y = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * zmpLB_x_t = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * zmpUB_x_t = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    Initial_emxArry(zmpUB_x);
    Initial_emxArry(zmpLB_x);
    Initial_emxArry(zmpUB_y);
    Initial_emxArry(zmpLB_y);
    Initial_emxArry(zmpLB_x_t);
    Initial_emxArry(zmpUB_x_t);

    test_zmp_boundary();
    ZMP_Boundary_Condition(totaltime, tinit, tend, tstep, ratio, Period, 0.0, lfoot, w, wfoot, leftgait_x,
                           rightgait_x, leftgait_y, rightgait_y, zmpUB_x, zmpLB_x, zmpUB_y, zmpLB_y,
                           zmpUB_x_t, zmpLB_x_t);

//    std::cout<<"UB_y: "<<(zmpUB_y->size[1])<<std::endl;
//    temp = zmpUB_y->data;
//    for(int i=0; i<(zmpUB_y->size[1]); i++)
//    {
//        std::cout<<(*temp)<<", ";
//        temp++;
//        if(i%10 == 0)
//            std::cout<<std::endl;
//    }

//    std::cout<<"UB_X_t: "<<(zmpUB_x_t->size[1])<<std::endl;
//    temp = zmpUB_x_t->data;
//    for(int i=0; i<(zmpUB_x_t->size[1]); i++)
//    {
//        std::cout<<(*temp)<<", ";
//        temp++;
//        if(i%10 == 0)
//            std::cout<<std::endl;
//    }

    std::cout<<"ZMP Boundary Condition Test Done"<<std::endl;

    //ZMP_Generation

    test_ZMP_Generation();

    emxArray_real_T * zmp_x = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * zmp_y = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * delta_y = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * coef_x = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * coef_y = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    Initial_emxArry(zmp_x);
    Initial_emxArry(zmp_y);
    Initial_emxArry(delta_y);
    Initial_emxArry(coef_x);
    Initial_emxArry(coef_y);

    ZMP_Generation(zmpLB_x_t, zmpUB_x_t, totaltime, tinit, tend, tstep, ratio, Period, lstep, lfoot,
                   w, wfoot, htar, zmp_x, zmp_y, delta_y, coef_x, coef_y);

//    std::cout<<"zmp_x: "<<(zmp_x->size[1])<<std::endl;
//    temp = zmp_x->data;
//    for(int i=0; i<(zmp_x->size[1]); i++)
//    {
//        std::cout<<(*temp)<<", ";
//        temp++;
//        if(i%10 == 0)
//            std::cout<<std::endl;
//    }

//    std::cout<<"zmp_y: "<<(zmp_y->size[1])<<std::endl;
//    temp = zmp_y->data;
//    for(int i=0; i<(zmp_y->size[1]); i++)
//    {
//        std::cout<<(*temp)<<", ";
//        temp++;
//        if(i%10 == 0)
//            std::cout<<std::endl;
//    }

    std::cout<<"ZMP Generation Test Done"<<std::endl;

    //COM Boundary Condition
    test_COM_Boundary_Condition();
    double * theta_i = new double[3];
    double * theta_e = new double[4];
    double * x_init_pframe = new double[2];
    double * x_init_heel_pframe = new double[2];
    double * x_end_pframe = new double[2];
    double * x_end_heel_pframe = new double[2];

    COM_Boundary_Condition(lthigh, lshank, lfoot, htar, lstep, z2, x_init_pframe, x_init_heel_pframe,
                           x_end_pframe, x_end_heel_pframe, theta_i, theta_e);

    std::cout<<"theta_iss:"<<std::endl;
    for(int i=0; i<3; i++)
        std::cout<<theta_i[i]<<", "<<std::endl;
    std::cout<<std::endl;

    std::cout<<"theta_ess:"<<std::endl;
    for(int i=0; i<4; i++)
        std::cout<<theta_e[i]<<", "<<std::endl;
    std::cout<<std::endl;

    std::cout<<"x_init_pframe:"<<std::endl;
    for(int i=0; i<2; i++)
        std::cout<<x_init_pframe[i]<<", "<<std::endl;
    std::cout<<std::endl;

    std::cout<<"x_init_heel_pframe:"<<std::endl;
    for(int i=0; i<2; i++)
        std::cout<<x_init_heel_pframe[i]<<", "<<std::endl;
    std::cout<<std::endl;

    std::cout<<"x_end_pframe:"<<std::endl;
    for(int i=0; i<2; i++)
        std::cout<<x_end_pframe[i]<<", "<<std::endl;
    std::cout<<std::endl;

    std::cout<<"x_end_heel_pframe:"<<std::endl;
    for(int i=0; i<2; i++)
        std::cout<<x_end_heel_pframe[i]<<", "<<std::endl;
    std::cout<<std::endl;

    std::cout<<"ZMP Generation Test Done"<<std::endl;

    //COM Generation
    double ze_p = z2 + x_end_pframe[1];
    double zi_p = z2 + x_init_pframe[1];
    double ze = 0.5*lfoot*sin(theta_e[0]+theta_e[1]-theta_e[2]);
    double zi = -0.5*lfoot*sin(theta_i[0]+theta_i[1]);
    std::cout<<ze<<", "<<zi<<std::endl;
    std::cerr<<"zi error: "<<zi_p - zi<<std::endl;
    std::cerr<<"ze error: "<<ze_p - ze<<std::endl;
    test_COM_Generation();

    emxArray_real_T * x_left = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * x_right = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * x_trunk = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * y_trunk = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * z_left = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    emxArray_real_T * z_right = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    Initial_emxArry(x_left);
    Initial_emxArry(x_right);
    Initial_emxArry(x_trunk);
    Initial_emxArry(y_trunk);
    Initial_emxArry(z_left);
    Initial_emxArry(z_right);

    b_COM_Generation(totaltime, tinit, tend, tstep, ratio, Period, lstep, lfoot, w, wfoot, htar, Mtot,
                   m1, m2, m3, g, zmp_x, zmp_y, delta_y, coef_x, coef_y, x_init_pframe, x_init_heel_pframe,
                   x_end_pframe, x_end_heel_pframe, theta_i, theta_e, zmpUB_x_t, zmpLB_x_t,
                   zmpUB_x, zmpLB_x, zmpUB_y, zmpLB_y, leftgait_x, rightgait_x, leftgait_y, rightgait_y,
                   z2, ze, zi, x_left, x_right, x_trunk, y_trunk, z_left, z_right);

//    std::cout<<"x_left: "<<(x_left->size[1])<<std::endl;
//    temp = x_left->data;
//    for(int i=0; i<100; i++)
//    {
//        std::cout<<(*temp)<<", ";
//        temp++;
//        if(i%10 == 0)
//            std::cout<<std::endl;
//    }

//    std::cout<<"x_trunk: "<<(x_trunk->size[1])<<std::endl;
//    temp = x_trunk->data;
//    for(int i=0; i<100; i++)
//    {
//        std::cout<<(*temp)<<", ";
//        temp++;
//        if(i%10 == 0)
//            std::cout<<std::endl;
//    }

//    std::cout<<"y_trunk: "<<(y_trunk->size[1])<<std::endl;
//    temp = y_trunk->data;
//    for(int i=0; i<100; i++)
//    {
//        std::cout<<(*temp)<<", ";
//        temp++;
//        if(i%10 == 0)
//            std::cout<<std::endl;
//    }

//    std::cout<<"z_left: "<<(zmp_y->size[1])<<std::endl;
//    temp = z_left->data;
//    for(int i=0; i<100; i++)
//    {
//        std::cout<<(*temp)<<", ";
//        temp++;
//        if(i%10 == 0)
//            std::cout<<std::endl;
//    }






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

    return 0;
}
