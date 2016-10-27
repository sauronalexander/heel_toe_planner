#include <heel_toe_planner/COM_Generation/Ipopt_Solver/Nonlinear_Eqn_Solver2.hpp>

NLP2::NLP2()
{
    this->htar = 0;
    this->lfoot = 0;
    this->lthigh = 0;
    this->z2 = 0;
    this->lshank = 0;
    this->lstep = 0;
    this->solve_index = 0; //index of the variable needs to be solved in x
}

void NLP2::Index_Increment()
{
    assert(this->solve_index < 3);
    this->solve_index++;
}

NLP2::NLP2(Ipopt::Number lthigh, Ipopt::Number lshank, Ipopt::Number lfoot, Ipopt::Number lstep,
           Ipopt::Number z2, Ipopt::Number htar)
{
    this->lthigh = lthigh;
    this->lshank = lshank;
    this->lfoot = lfoot;
    this->lstep = lstep;
    this->z2 = z2;
    this->htar = htar;
    this->solve_index = 0; //index of the variable needs to be solved in x
}

NLP2::~NLP2()
{}

bool NLP2::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                        Ipopt::Index &nnz_h_lag, IndexStyleEnum &Index_style)
{
    n = 3;
    m = 3;
    nnz_jac_g = 6;
    nnz_h_lag =4;
    Index_style = FORTRAN_STYLE;
    return true;
}

bool NLP2::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u, Ipopt::Index m,
                           Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    assert (n==3);
    assert (m==3);
    x_l[0] = -1.0*M_PI/6;
    x_u[0] = 0.0;
    x_l[1] = -1.0*M_PI/5;
    x_u[1] = 0.0;
    x_l[2] = 0;
    x_u[2] = M_PI/2;
    g_l[0] = g_l[1] = g_l[2] = g_u[0] = g_u[1] = g_u[2] = 0.0;
    return true;
}

bool NLP2::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x, bool init_z, Ipopt::Number *z_L,
                              Ipopt::Number *z_U, Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
{
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);
    //First Solved In Matlab
    x[0] = -0.2833;
    x[1] = -0.6665;
    x[2] = 0.3929;
    return true;
}

bool NLP2::eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number &obj_value)
{
    obj_value = x[this->solve_index];
    return true;
}

bool NLP2::eval_grad_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number *grad_f)
{
    for(int i=0; i< (int) n; i++)
        grad_f[i] = 0.0;
    grad_f[this->solve_index] = 1.0;
    //std::cout<<"eval_grad_f: "<<grad_f[0]<<", "<<grad_f[1]<<", "<<grad_f[2]<<std::endl;
    return true;
}

bool NLP2::eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Number *g)
{
    g[0] = -1.0*lthigh*sin(x[0]) - lshank*sin(x[0]+x[1]) - (1-htar)*lfoot*cos(x[0]+x[1]) + (lthigh+lshank)*sin(x[2])
            -(lstep-lfoot+htar*lfoot);
    g[1] = lthigh*cos(x[0]) + lshank*cos(x[0]+x[1]) - (1-htar)*lfoot*sin(x[0]+x[1]) - z2;
    g[2] = (lthigh+lshank)*cos(x[2])-z2;
//    std::cout<<"eval_g: "<<g[0]<<", "<<g[1]<<", "<<g[2]<<std::endl;
//    std::cout<<x[0]<<", "<<x[1]<<", "<<x[2]<<std::endl;
    return true;
}

bool NLP2::eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac,
                      Ipopt::Index *iRow, Ipopt::Index *jCol, Ipopt::Number *values)
{
    if(values == NULL)
    {
        iRow[0] = 1;
        jCol[0] = 1;

        iRow[1] = 1;
        jCol[1] = 2;

        iRow[2] = 1;
        jCol[2] = 3;

        iRow[3] = 2;
        jCol[3] = 1;

        iRow[4] = 2;
        jCol[4] = 2;

        iRow[5] = 3;
        jCol[5] = 3;
    }
    else
    {
        values[0] = -lthigh*cos(x[0]) - lshank*cos(x[0]+x[1]) + (1-htar)*lfoot*sin(x[0]+x[1]);
        values[1] = -1.0*lshank*cos(x[0]+x[1]) + (1-htar)*lfoot*sin(x[0]+x[1]);
        values[2] = (lthigh+lshank)*cos(x[2]);
        values[3] = -1.0*lthigh*sin(x[0]) - lshank*sin(x[0]+x[1]) - (1-htar)*lfoot*cos(x[0]+x[1]);
        values[4] = -1.0*lshank*sin(x[0]+x[1]) - (1-htar)*lfoot*cos(x[0]+x[1]);
        values[5] = -(lthigh+lshank)*sin(x[2]);
        //std::cout<<"eval_jac_g: "<<values[0]<<", "<<values[1]<<", "<<values[2]<<", "<<values[3]<<", "<<
        //           values[4]<<", "<<values[5]<<std::endl;
    }
    return true;
}

bool NLP2::eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number obj_factor,
                  Ipopt::Index m, const Ipopt::Number *lambda, bool new_lambda, Ipopt::Index nele_hess,
                  Ipopt::Index *iRow, Ipopt::Index *jCol, Ipopt::Number *values)
{
    if(values == NULL)
    {
        iRow[0] = 1;
        jCol[0] = 1;

        iRow[1] = 2;
        jCol[1] = 1;

        iRow[2] = 2;
        jCol[2] = 2;

        iRow[3] = 3;
        jCol[3] = 3;
    }
    else
    {
        //g1--lmbda[0]
        values[0] = lambda[0]*(lthigh*sin(x[0]) + lshank*sin(x[0]+x[1] + (1-htar)*lfoot*cos(x[0]+x[1])));
        values[1] = lambda[0]*(lshank*sin(x[0]+x[1]) + (1-htar)*lfoot*cos(x[0]+x[1]));
        values[2] = lambda[0]*(lshank*sin(x[0]+x[1]) + (1-htar)*lfoot*cos(x[0]+x[1]));
        values[3] = lambda[0]*(-1.0*(lthigh+lshank)*sin(x[2]));

        //g2--lmbda[1]
        values[0] += lambda[1]*(-1.0*lthigh*cos(x[0]) - lshank*cos(x[0]+x[1]) + (1-htar)*lfoot*sin(x[0]+x[1]));
        values[1] += lambda[1]*(-1.0*lshank*cos(x[0]+x[1]) + (1-htar)*lfoot*sin(x[0]+x[1]));
        values[2] += lambda[1]*(-1.0*lshank*cos(x[0]+x[1]) + (1-htar)*lfoot*sin(x[0]+x[1]));

        //g3--lambda[3]
        values[3] += lambda[2]*(-1.0*(lthigh+lshank)*cos(x[2]));
        //std::cout<<"eval_hessian: "<<values[0]<<", "<<values[1]<<", "<<values[2]<<", "<<values[3]<<", "<<
        //           values[4]<<std::endl;
    }
    return true;
}

void NLP2::finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number *x,
                             const Ipopt::Number *z_L, const Ipopt::Number *z_U, Ipopt::Index m,
                             const Ipopt::Number *g, const Ipopt::Number *lambda,
                             Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                             Ipopt::IpoptCalculatedQuantities *ip_cq)
{

}
