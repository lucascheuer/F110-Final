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
}

void MPC::CreateHessianMatrix()
{
    hessian_.resize(state_size_ * (horizon_ + 1) + input_size_ * horizon_, state_size_ * (horizon_ + 1) + input_size_ * horizon_);
    for (int ii = 0; ii < horizon_ + 1; ++ii) // change for terminal cost
    {
        for (int row = 0; row < state_size_; ++row)
        {
            for (int col = 0; col < state_size_; ++col)
            {
                hessian_.insert(ii * state_size_ + row, ii * state_size_ + col) = cost_.q()(row, col);
            }
        }
    }
    for (int ii = 0; ii < horizon_; ++ii) // change for terminal cost
    {
        for (int row = 0; row < input_size_; ++row)
        {
            for (int col = 0; col < input_size_; ++col)
            {
                hessian_.insert(state_size_ * (horizon_ + 1) + ii * input_size_ + row, state_size_ * (horizon_ + 1) + ii * input_size_ + col) = cost_.r()(row, col);
            }
        }
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

}

void MPC::CreateLowerBound()
{

}

void MPC::CreateUpperBound()
{

}