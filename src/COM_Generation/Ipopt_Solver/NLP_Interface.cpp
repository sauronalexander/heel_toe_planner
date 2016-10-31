#include <heel_toe_planner/COM_Generation/Ipopt_Solver/NLP_Interface.hpp>

double NLP1_Solver(double yzmp_1, double delta13, double yzmp_1_prime, double yzmp_3)
{
    Ipopt::SmartPtr<Ipopt::TNLP> mynlp = new NLP1(yzmp_1, delta13, yzmp_1_prime, yzmp_3);
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    app->Options()->SetNumericValue("tol", 1e-9);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetIntegerValue("print_level", 0);


    Ipopt::ApplicationReturnStatus status;
    status = app->Initialize();
    if(status != Ipopt::Solve_Succeeded)
    {
        ROS_ERROR_STREAM("*** Error during initialization!");
        return (int) status;
    }
    status = app->OptimizeTNLP(mynlp);
    if (status != Ipopt::Solve_Succeeded) {
        // Retrieve some statistics about the solve
//        Ipopt::Number final_obj = app->Statistics()->FinalObjective();
//        std::cout << std::endl << std::endl << "*** The final value of the w3 is " << final_obj << '.' << std::endl;
        ROS_ERROR_STREAM("w3 cannot be solved!");
        return 0;
    }
    else
        return app->Statistics()->FinalObjective();


}

bool NLP2_Solver(double lthigh, double lshank, double lfoot, double lstep, double z2, double htar, double * theta_i)
{
    Ipopt::SmartPtr<NLP2> NLP_pt = new NLP2(lthigh, lshank, lfoot, lstep, z2, htar);
    Ipopt::SmartPtr<Ipopt::TNLP> mynlp =  NLP_pt;
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app;// = IpoptApplicationFactory();

    for(int i=0; i<3; i++)
    {
        app = IpoptApplicationFactory();
        app->Options()->SetNumericValue("tol", 1e-2);
        app->Options()->SetStringValue("mu_strategy", "adaptive");
        app->Options()->SetIntegerValue("print_level", 0);
        app->Options()->SetNumericValue("constr_viol_tol", 1e-2);
        app->Options()->SetNumericValue("acceptable_tol", 1e-2);
        Ipopt::ApplicationReturnStatus status;
        status = app->Initialize();
        if(status != Ipopt::Solve_Succeeded)
        {
            ROS_ERROR_STREAM("*** Error during initialization!");
            return (int) status;
        }
        status = app->OptimizeTNLP(mynlp);
        if (status != Ipopt::Solve_Succeeded) {
            // Retrieve some statistics about the solve
    //        Ipopt::Number final_obj = app->Statistics()->FinalObjective();
    //        std::cout << std::endl << std::endl << "*** The final value of the w3 is " << final_obj << '.' << std::endl;
            ROS_ERROR_STREAM("theta_i cannot be solved!");
            return false;
        }
        else
            theta_i[i] = app->Statistics()->FinalObjective();
        NLP_pt->Index_Increment();

    }

    return true;
}

bool NLP3_Solver(double lthigh, double lshank, double lfoot, double lstep, double z2, double htar, double zi, double * theta_e)
{
    Ipopt::SmartPtr<NLP3> NLP_pt = new NLP3(lthigh, lshank, lfoot, lstep, z2, zi, htar);
    Ipopt::SmartPtr<Ipopt::TNLP> mynlp =  NLP_pt;
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app;// = IpoptApplicationFactory();

    for(int i=0; i<4; i++)
    {
        app = IpoptApplicationFactory();
        app->Options()->SetNumericValue("tol", 5e-3);
        app->Options()->SetStringValue("mu_strategy", "adaptive");
        app->Options()->SetIntegerValue("print_level", 0);
        app->Options()->SetNumericValue("constr_viol_tol", 5e-3);
//        app->Options()->SetNumericValue("acceptable_tol", 1e-2);

        Ipopt::ApplicationReturnStatus status;
        status = app->Initialize();
        if(status != Ipopt::Solve_Succeeded)
        {
            ROS_ERROR_STREAM("*** Error during initialization!");
            return (int) status;
        }
        status = app->OptimizeTNLP(mynlp);
        if (status != Ipopt::Solve_Succeeded) {
            // Retrieve some statistics about the solve
    //        Ipopt::Number final_obj = app->Statistics()->FinalObjective();
    //        std::cout << std::endl << std::endl << "*** The final value of the w3 is " << final_obj << '.' << std::endl;
            ROS_ERROR_STREAM("theta_e cannot be solved!");
            return false;
        }
        else
            theta_e[i] = app->Statistics()->FinalObjective();
        NLP_pt->Index_Increment();

    }
    return true;
}
