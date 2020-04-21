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
        MPC(int horizon);
        virtual ~MPC();
        void Update();
        void Update(State &current_state, State &desired_state, Input &last_input, Model &model, Cost &cost);

    private:
        int horizon_;
        int input_size_;
        int state_size_;
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
        // functions
        void CreateHessianMatrix();
        void CreateGradientVector();
        void CreateLinearConstraintMatrix();
        void CreateLowerBound();
        void CreateUpperBound();
        void SparseBlockSet(Eigen::SparseMatrix<double> &modify, const Eigen::MatrixXd &block, int row_start, int col_start);
        void SparseBlockEye(Eigen::SparseMatrix<double> &modify, int size, int row_start, int col_start);

        // mode
};
#endif