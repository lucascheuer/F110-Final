#include "mpc.hpp"
MPC::MPC(int horizon): horizon_(horizon), state_size_(3), input_size_(2)
{
    ROS_INFO("mpc created");
}

MPC::~MPC()
{
    ROS_INFO("killing the mpc");
}

void MPC::Update()
{
    CreateHessianMatrix();
}

void MPC::Update(State &current_state, State &desired_state, Input &last_input, Model &model, Cost &cost)
{

    // constraints_ = constraints;
    // model_ = model;
    // current_state_ = current_state;
    cost_ = cost;
    desired_state_ = desired_state;
    model_ = model;
    model_.linearize(current_state, last_input, 0.1);
    CreateHessianMatrix();
    CreateGradientVector();
    CreateLinearConstraintMatrix();
}

void MPC::CreateHessianMatrix()
{   
    hessian_.resize(state_size_ * (horizon_ + 1) + input_size_ * horizon_, state_size_ * (horizon_ + 1) + input_size_ * horizon_);
    for (int ii = 0; ii < horizon_ + 1; ++ii) // change for terminal cost
    {   
        SparseBlockSet(hessian_, cost_.q(), ii * state_size_, ii * state_size_);
    }
    for (int ii = 0; ii < horizon_; ++ii) // change for terminal cost
    {
        SparseBlockSet(hessian_, cost_.r(), state_size_ * (horizon_ + 1) + ii * input_size_, state_size_ * (horizon_ + 1) + ii * input_size_);
    }
}

void MPC::CreateGradientVector()
{
    gradient_.resize(state_size_ * (horizon_ + 1) + input_size_ * horizon_);
    Eigen::VectorXd horizon_block = (-1 * cost_.q() * desired_state_.ToVector()).replicate(horizon_ + 1, 1); // change for terminal cost
    gradient_.head(horizon_block.size()) = horizon_block;
}

void MPC::CreateLinearConstraintMatrix()
{
    linear_matrix_.setZero(); // here for steering angle vs throttle
    linear_matrix_.resize(state_size_ * (horizon_ + 1) + (horizon_ + 1) * state_size_ + horizon_ * input_size_, state_size_ * (horizon_ + 1) + input_size_ * horizon_);
    int input_col_start = state_size_ * (horizon_+ 1);
    
    Eigen::MatrixXd a_eye(state_size_, 2 * state_size_);
    a_eye << model_.a() , Eigen::MatrixXd::Identity(state_size_, state_size_);

    // prediction equality constraint
    SparseBlockSet(linear_matrix_, Eigen::MatrixXd::Identity(state_size_, state_size_), 0, 0);
    for (int ii = 1; ii < horizon_ + 1; ++ii)
    {
        // SparseBlockSet()
        SparseBlockSet(linear_matrix_, a_eye, ii*state_size_, (ii - 1) * state_size_);
        SparseBlockSet(linear_matrix_, model_.b(), ii*state_size_, input_col_start + (ii - 1) * input_size_);
    }

    // state and input upper and lower bounds
    SparseBlockEye(linear_matrix_, linear_matrix_.cols(), (horizon_ + 1) * state_size_, 0);
}

void MPC::CreateLowerBound()
{

}

void MPC::CreateUpperBound()
{

}

void MPC::SparseBlockSet(Eigen::SparseMatrix<double> &modify, const Eigen::MatrixXd &block, int row_start, int col_start)
{
    int row_size = block.rows();
    int col_size = block.cols();
    for (int row = 0; row < row_size; ++row)
    {
        for (int col = 0; col < col_size; ++col)
        {
            if (block(row, col) != 0)
            {
                modify.insert(row_start + row, col_start + col) = block(row, col);
            }
        }
    }
}

void MPC::SparseBlockEye(Eigen::SparseMatrix<double> &modify, int size, int row_start, int col_start)
{
    for (int row = 0; row < size; ++row)
    {
        modify.insert(row_start + row, col_start + row) = 1;
    }
}