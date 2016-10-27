#include <heel_toe_planner/COM_Generation/Ipopt_Solver/Nonlinear_Eqn_Solver1.hpp>

NLP1::NLP1()
{
    this->yzmp_1 = 0;
    this->yzmp_1_prime = 0;
    this->yzmp_3 = 0;
    this->delta13 = 0;
}

NLP1::NLP1(Ipopt::Number yzmp_1, Ipopt::Number delta13, Ipopt::Number yzmp_1_prime, Ipopt::Number yzmp_3)
{
    this->yzmp_1 = yzmp_1;
    this->delta13 = delta13;
    this->yzmp_1_prime = yzmp_1_prime;
    this->yzmp_3 = yzmp_3;
}

NLP1::~NLP1()
{}

bool NLP1::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g, Ipopt::Index &nnz_h_lag,
                        IndexStyleEnum &index_style)
{
    n=1;
    m=1;
    nnz_jac_g = 1;
    nnz_h_lag = 1;
    index_style = FORTRAN_STYLE;
    return true;
}

bool NLP1::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u, Ipopt::Index m,
                           Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    assert (n==1);
    assert (m==1);
    x_l[0] = 25;
    x_u[0] = 35;
    g_l[0] = g_u[0]=0.0;
    return true;
}

bool NLP1::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x, bool init_z, Ipopt::Number *z_L,
                              Ipopt::Number *z_U, Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
{
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);
    x[0] = 26.0;
    return true;
}

bool NLP1::eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number & obj_value)
{
    Ipopt::Number w3 = x[0];
    obj_value = w3;
    return true;
}
bool NLP1::eval_grad_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number * grad_f)
{
    grad_f[0] = 1.0;
    return true;
}

bool NLP1::eval_g(Ipopt::Index n, const Ipopt::Number * x, bool new_x, Ipopt::Index m, Ipopt::Number *g)
{
    Ipopt::Number w3 = x[0];
    g[0] = yzmp_1*cos(delta13*w3) + (yzmp_1_prime/w3)*sin(delta13*w3) - yzmp_3;
    return true;
}

bool NLP1::eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac,
                      Ipopt::Index *iRow, Ipopt::Index *jCol, Ipopt::Number *values)
{
    if(values == NULL)
    {
        iRow[0] = 1;
        jCol[0] = 1;
    }
    else
    {
        Ipopt::Number w3 = x[0];
        values[0] = -yzmp_1*delta13*sin(delta13*w3) - (yzmp_1_prime/(w3*w3))*sin(delta13*w3) +
                (yzmp_1_prime*delta13/w3)*cos(delta13*w3);
    }
    return true;
}

bool NLP1::eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number obj_factor,
                  Ipopt::Index m, const Ipopt::Number *lambda, bool new_lambda, Ipopt::Index nele_hess,
                  Ipopt::Index *iRow, Ipopt::Index *jCol, Ipopt::Number *values)
{
    if(values == NULL)
    {
        iRow[0] = 1;
        jCol[0] = 1;
    }
    else
    {
        Ipopt::Number w3 = x[0];
        values[0] = -yzmp_1*delta13*delta13*cos(delta13*w3) + (2*yzmp_1_prime/(w3*w3*w3))*sin(delta13*w3)
                - (2*yzmp_1_prime*delta13/(w3*w3))*cos(delta13*w3)
                - (yzmp_1_prime*delta13*delta13/w3)*sin(delta13*w3);
        values[0] *= lambda[0];
    }
    return true;
}

void NLP1::finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x,
                             const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m,
                             const Ipopt::Number* g, const Ipopt::Number* lambda, Ipopt::Number obj_value,
                             const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq)
{
    // here is where we would store the solution to variables, or write to a file, etc
    // so we could use the solution. Since the solution is displayed to the console,
    // we currently do nothing here.

    return;
}
