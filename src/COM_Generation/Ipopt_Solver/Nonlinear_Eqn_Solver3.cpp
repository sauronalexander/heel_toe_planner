#include <heel_toe_planner/COM_Generation/Ipopt_Solver/Nonlinear_Eqn_Solver3.hpp>

NLP3::NLP3()
{
    this->htar = 0;
    this->lfoot = 0;
    this->lthigh = 0;
    this->z2 = 0;
    this->lshank = 0;
    this->lstep = 0;
    this->zi = 0;
    this->solve_index = 0; //index of the variable needs to be solved in x
}

void NLP3::Index_Increment()
{
    assert(this->solve_index < 4);
    this->solve_index++;
}

NLP3::NLP3(Ipopt::Number lthigh, Ipopt::Number lshank, Ipopt::Number lfoot, Ipopt::Number lstep,
           Ipopt::Number z2, Ipopt::Number zi, Ipopt::Number htar)
{
    this->lthigh = lthigh;
    this->lshank = lshank;
    this->lfoot = lfoot;
    this->lstep = lstep;
    this->z2 = z2;
    this->htar = htar;
    this->zi = zi;
    this->solve_index = 0; //index of the variable needs to be solved in x
//    std::cout<<this->lthigh<<", "<<this->lshank<<", "<<this->lfoot<<", "<<this->lstep<<", "<<this->z2<<", "<<this->htar<<", "<<this->zi<<std::endl;

}

NLP3::~NLP3()
{}

bool NLP3::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                        Ipopt::Index &nnz_h_lag, IndexStyleEnum &Index_style)
{
    n = 4;
    m = 4;
    nnz_jac_g = 12;
    nnz_h_lag = 7;
    Index_style = FORTRAN_STYLE;
    return true;
}

bool NLP3::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u, Ipopt::Index m,
                           Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    assert (n==4);
    assert (m==4);
    x_l[0] = 0.0;
    x_u[0] = 1.0*M_PI/2;
    x_l[1] = -1.0*M_PI/2;
    x_u[1] = 0.0;
    x_l[2] = -1.0*M_PI/2;
    x_u[2] = 0.0;
    x_l[3] = -1.0*M_PI/2;
    x_u[3] = 0.0;
    g_l[0] = g_l[1] = g_l[2] = g_l[3] = g_u[0] = g_u[1] = g_u[2] = g_u[3] = 0.0;
    return true;
}

bool NLP3::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x, bool init_z, Ipopt::Number *z_L,
                              Ipopt::Number *z_U, Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
{
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);
    x[0] = 0.0;
    x[1] = -1.0*M_PI/2;
    x[2] = -1.0*M_PI/2;
    x[3] = -1.0*M_PI/2;
    return true;
}

bool NLP3::eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number &obj_value)
{
    obj_value = x[this->solve_index];
    return true;
}

bool NLP3::eval_grad_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number *grad_f)
{
    for(int i=0; i< (int) n; i++)
        grad_f[i] = 0.0;
    grad_f[this->solve_index] = 1.0;
    //std::cout<<"eval_grad_f: "<<grad_f[0]<<", "<<grad_f[1]<<", "<<grad_f[2]<<std::endl;
    return true;
}

bool NLP3::eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Number *g)
{
    g[0] = lthigh*sin(x[0]) + lshank*sin(x[0]+x[1]) - htar*lfoot*cos(x[0]+x[1]-x[2])
            - (lthigh+lshank)*sin(x[3]) - htar*lfoot + lfoot - lstep;
    g[1] = lthigh*cos(x[0]) + lshank*cos(x[0]+x[1]) + htar*lfoot*sin(x[0]+x[1]-x[2]) - (lthigh+lshank)*cos(x[3]);
    g[2] = (lthigh+lshank)*cos(x[3]) - z2;
    g[3] = 0.5*lfoot*sin(x[0]+x[1]-x[2]) - zi;
//    std::cout<<"eval_g: "<<g[0]<<", "<<g[1]<<", "<<g[2]<<", "<<g[3]<<std::endl;
    return true;
}

bool NLP3::eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac,
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

        iRow[3] = 1;
        jCol[3] = 4;

        iRow[4] = 2;
        jCol[4] = 1;

        iRow[5] = 2;
        jCol[5] = 2;

        iRow[6] = 2;
        jCol[6] = 3;

        iRow[7] = 2;
        jCol[7] = 4;

        iRow[8] = 3;
        jCol[8] = 4;

        iRow[9] = 4;
        jCol[9] = 1;

        iRow[10] = 4;
        jCol[10] = 2;

        iRow[11] = 4;
        jCol[11] = 3;
    }
    else
    {
        values[0] = lthigh*cos(x[0]) + lshank*cos(x[0]+x[1]) + htar*lfoot*sin(x[0]+x[1]-x[2]);
        values[1] = lshank*cos(x[0]+x[1]) + htar*lfoot*sin(x[0]+x[1]-x[2]);
        values[2] = -1.0*htar*lfoot*sin(x[0]+x[1]-x[2]);
        values[3] = -1.0*(lthigh+lshank)*cos(x[3]);
        values[4] = -1.0*lthigh*sin(x[0]) - lshank*sin(x[0]+x[1]) + htar*lfoot*cos(x[0]+x[1]-x[2]);
        values[5] = -1.0*lshank*sin(x[0]+x[1]) + htar*lfoot*cos(x[0]+x[1]-x[2]);
        values[6] = -1.0*htar*lfoot*cos(x[0]+x[1]-x[2]);
        values[7] = (lthigh+lshank)*sin(x[3]);
        values[8] = -1.0*(lthigh+lshank)*sin(x[3]);
        values[9] = 0.5*lfoot*cos(x[0]+x[1]-x[2]);
        values[10] = 0.5*lfoot*cos(x[0]+x[1]-x[2]);
        values[11] = -0.5*lfoot*cos(x[0]+x[1]-x[2]);
//        std::cout<<"eval_jac_g: "<<lfoot<<": "<<x[0]<<", "<<x[1]<<", "<<x[2]<<", "<<x[0]+x[1]-x[2]<<", "<<lfoot*cos(x[0]+x[1]-x[2])<<";   "<<values[9]<<", "<<values[10]<<", "<<values[11]<<std::endl;
    }
    return true;
}

bool NLP3::eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number obj_factor,
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
        jCol[3] = 1;

        iRow[4] = 3;
        jCol[4] = 2;

        iRow[5] = 3;
        jCol[5] = 3;

        iRow[6] = 4;
        jCol[6] = 4;
    }
    else
    {
        values[0] = lambda[0] * (-1.0*lthigh*sin(x[0]) - lshank*sin(x[0]+x[1]) + htar*lfoot*cos(x[0]+x[1]-x[2]));
        values[1] = lambda[0] * (-1.0*lshank*sin(x[0]+x[1]) + htar*lfoot*cos(x[0]+x[1]-x[2]));
        values[2] = lambda[0] * (-1.0*lshank*sin(x[0]+x[1]) + htar*lfoot*cos(x[0]+x[1]-x[2]));
        values[3] = lambda[0] * (-1.0*htar*lfoot*cos(x[0]+x[1]-x[2]));
        values[4] = lambda[0] * (-1.0*htar*lfoot*cos(x[0]+x[1]-x[2]));
        values[5] = lambda[0] * (htar*lfoot*cos(x[0]+x[1]-x[2]));
        values[6] = lambda[0] * ((lthigh+lshank)*sin(x[3]));

        values[0] += lambda[1] * (-1.0*lthigh*cos(x[0]) - lshank*cos(x[0]+x[1]) - htar*lfoot*sin(x[0]+x[1]-x[2]));
        values[1] += lambda[1] * (-1.0*lshank*cos(x[0]+x[1]) - htar*lfoot*sin(x[0]+x[1]-x[2]));
        values[2] += lambda[1] * (-1.0*lshank*cos(x[0]+x[1]) - htar*lfoot*sin(x[0]+x[1]-x[2]));
        values[3] += lambda[1] * (htar*lfoot*sin(x[0]+x[1]-x[2]));
        values[4] += lambda[1] * (htar*lfoot*sin(x[0]+x[1]-x[2]));
        values[5] += lambda[1] * (-1.0*htar*lfoot*sin(x[0]+x[1]-x[2]));
        values[6] += lambda[1] * ((lthigh+lshank)*cos(x[3]));

        values[6] += lambda[2] * (-1.0*(lthigh+lshank)*cos(x[3]));

        values[0] += lambda[3] * (-0.5*lfoot*sin(x[0]+x[1]-x[2]));
        values[1] += lambda[3] * (-0.5*lfoot*sin(x[0]+x[1]-x[2]));
        values[2] += lambda[3] * (-0.5*lfoot*sin(x[0]+x[1]-x[2]));
        values[3] += lambda[3] * (0.5*lfoot*sin(x[0]+x[1]-x[2]));
        values[4] += lambda[3] * (0.5*lfoot*sin(x[0]+x[1]-x[2]));
        values[5] += lambda[3] * (-0.5*lfoot*sin(x[0]+x[1]-x[2]));
//        std::cout<<"eval_hessian: "<<values[0]<<", "<<values[1]<<", "<<values[2]<<", "<<values[3]<<", "<<
//            values[4]<<", "<<values[5]<<std::endl;
    }
    return true;
}

void NLP3::finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number *x,
                             const Ipopt::Number *z_L, const Ipopt::Number *z_U, Ipopt::Index m,
                             const Ipopt::Number *g, const Ipopt::Number *lambda,
                             Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                             Ipopt::IpoptCalculatedQuantities *ip_cq)
{}
