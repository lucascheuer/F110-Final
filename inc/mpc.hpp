#ifndef MPC_H
#define MPC_H
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>

#include "constraints.hpp"
#include "state.hpp"
#include "model.hpp"
#include "cost.hpp"

class MPC
{
    public:
        MPC();
        MPC(int horizon);
        virtual ~MPC();
        void Init(Model model, Cost cost, Constraints constraints);
        void Update(State current_state, State desired_state, Input last_input);
        Input solved_input();
    private:
        int horizon_;
        int input_size_;
        int state_size_;
        int num_states_;
        int num_inputs_;
        int num_variables_;
        int num_constraints_;
        bool solver_init_ = false;
        Eigen::SparseMatrix<double> hessian_;
        Eigen::VectorXd gradient_;
        Eigen::SparseMatrix<double> linear_matrix_;
        Eigen::VectorXd lower_bound_;
        Eigen::VectorXd upper_bound_;
        Constraints constraints_;
        Model model_;
        State current_state_;
        State desired_state_;
        Cost cost_;
        Input solved_input_;
        OsqpEigen::Solver solver_;

        // functions
        void CreateHessianMatrix();
        void CreateGradientVector();
        void CreateLinearConstraintMatrix();
        void CreateLowerBound();
        void CreateUpperBound();
        void DoMPC();
        void SparseBlockSet(Eigen::SparseMatrix<double> &modify, const Eigen::MatrixXd &block, int row_start, int col_start);
        void SparseBlockEye(Eigen::SparseMatrix<double> &modify, int size, int row_start, int col_start);
        
        // mode
};
#endif