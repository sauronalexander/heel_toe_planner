#ifndef NONLINEAR_EQN_SOLVER3_HPP
#define NONLINEAR_EQN_SOLVER3_HPP

#include <iostream>
#include <cassert>
#include <cmath>
#include <coin/IpTNLP.hpp>
#include <coin/IpTypes.hpp>

class NLP3: public Ipopt::TNLP
{
    public:
        NLP3();

        NLP3(Ipopt::Number lthigh, Ipopt::Number lshank, Ipopt::Number lfoot, Ipopt::Number lstep,
             Ipopt::Number z2, Ipopt::Number htar, Ipopt::Number zi);

        void Index_Increment();

        virtual ~NLP3();

        virtual bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                                  Ipopt::Index &nnz_h_lag, IndexStyleEnum &Index_style);

        virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                                     Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u);

        virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x, bool init_z,
                                        Ipopt::Number *z_L, Ipopt::Number *z_U, Ipopt::Index m,
                                        bool init_lambda, Ipopt::Number *lambda);

        virtual bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number & obj_value);

        virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number * grad_f);

        virtual bool eval_g(Ipopt::Index n, const Ipopt::Number * x, bool new_x, Ipopt::Index m, Ipopt::Number *g);

        virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m,
                                Ipopt::Index nele_jac, Ipopt::Index *iRow, Ipopt::Index *jCol,
                                Ipopt::Number *values);

        virtual bool eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number obj_factor,
                            Ipopt::Index m, const Ipopt::Number *lambda, bool new_lambda, Ipopt::Index nele_hess,
                            Ipopt::Index *iRow, Ipopt::Index *jCol, Ipopt::Number *values);

        virtual void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x,
                                       const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m,
                                       const Ipopt::Number* g, const Ipopt::Number* lambda,
                                       Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data,
                                       Ipopt::IpoptCalculatedQuantities* ip_cq);

    private:
        Ipopt::Number lthigh;
        Ipopt::Number lshank;
        Ipopt::Number htar;
        Ipopt::Number lfoot;
        Ipopt::Number lstep;
        Ipopt::Number z2;
        Ipopt::Number zi;
        int solve_index;
        NLP3(const NLP3&);
        NLP3& operator=(const NLP3&);

};
#endif // NONLINEAR_EQN_SOLVER3_HPP
