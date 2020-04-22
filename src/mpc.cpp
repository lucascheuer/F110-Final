#include "mpc.hpp"

MPC::MPC()
{

}

MPC::MPC(int horizon):
    horizon_(horizon),
    state_size_(3),
    input_size_(2),
    num_states_(state_size_ * (horizon_ + 1)),
    num_inputs_(input_size_ * horizon_),
    num_variables_(num_states_ + num_inputs_),
    num_constraints_(2 * num_states_ + num_inputs_)
{
    
    ROS_INFO("mpc created");
}

MPC::~MPC()
{
    ROS_INFO("killing the mpc");
}

void MPC::Init(Model model, Cost cost, Constraints constraints)
{
    model_ = model;
    cost_ = cost;
    constraints_ = constraints;
}

void MPC::Update(State current_state, State desired_state, Input last_input)
{


    current_state_ = current_state;
    desired_state_ = desired_state;
    model_.linearize(current_state, last_input, 0.1);
    constraints_.set_state(current_state_);
    CreateHessianMatrix();
    CreateGradientVector();
    CreateLinearConstraintMatrix();
    CreateLowerBound();
    CreateUpperBound();
    Eigen::VectorXd QPSolution;

    if (solver_init_)
    {
        solver_.updateHessianMatrix(hessian_);
        solver_.updateGradient(gradient_);
        solver_.updateLinearConstraintsMatrix(linear_matrix_);
        solver_.updateBounds(lower_bound_, upper_bound_);
    } else
    {
        solver_.settings()->setVerbosity(false);
        solver_.data()->setNumberOfVariables(num_variables_);
        solver_.data()->setNumberOfConstraints(num_constraints_);
        if (!solver_.data()->setHessianMatrix(hessian_)) std::cout << "hessian failed" << std::endl;
        if (!solver_.data()->setGradient(gradient_)) std::cout << "gradient failed" << std::endl;
        if (!solver_.data()->setLinearConstraintsMatrix(linear_matrix_)) std::cout << "lienar failed" << std::endl;
        if (!solver_.data()->setLowerBound(lower_bound_)) std::cout << "lower failed" << std::endl;
        if (!solver_.data()->setUpperBound(upper_bound_)) std::cout << "upper failed" << std::endl;
        if (!solver_.initSolver()) std::cout << "solver failed" << std::endl;
        solver_init_ = true;
    }
    if (!solver_.solve())
    {
        std::cout << "solve failed" << std::endl;
    } else
    {
        QPSolution = solver_.getSolution();
        solved_input_.SetV(QPSolution(num_states_));
        solved_input_.SetSteerAng(QPSolution(num_states_ + 1));
        
    }
}

void MPC::CreateHessianMatrix()
{   
    hessian_.resize(num_variables_, num_variables_);
    for (int ii = 0; ii < horizon_ + 1; ++ii) // change for terminal cost
    {   
        SparseBlockSet(hessian_, cost_.q(), ii * state_size_, ii * state_size_);
    }
    for (int ii = 0; ii < horizon_; ++ii) // change for terminal cost
    {
        SparseBlockSet(hessian_, cost_.r(), num_states_ + ii * input_size_, num_states_ + ii * input_size_);
    }
}

void MPC::CreateGradientVector()
{
    gradient_.resize(num_variables_);
    Eigen::VectorXd horizon_block = (-1 * cost_.q() * desired_state_.ToVector()).replicate(horizon_ + 1, 1); // change for terminal cost
    gradient_.head(horizon_block.size()) = horizon_block;
}

void MPC::CreateLinearConstraintMatrix()
{
    // here for steering angle vs throttle
    linear_matrix_.resize(num_constraints_, num_variables_);
    
    Eigen::MatrixXd a_eye(state_size_, 2 * state_size_);
    a_eye << model_.a() , Eigen::MatrixXd::Identity(state_size_, state_size_);

    // prediction equality constraint
    SparseBlockSet(linear_matrix_, Eigen::MatrixXd::Identity(state_size_, state_size_), 0, 0);
    for (int ii = 1; ii < horizon_ + 1; ++ii)
    {
        // SparseBlockSet()
        SparseBlockSet(linear_matrix_, a_eye, ii*state_size_, (ii - 1) * state_size_);
        SparseBlockSet(linear_matrix_, model_.b(), ii*state_size_, num_states_ + (ii - 1) * input_size_);
    }

    // state and input upper and lower bounds
    SparseBlockEye(linear_matrix_, linear_matrix_.cols(), (horizon_ + 1) * state_size_, 0);
}

void MPC::CreateLowerBound()
{
    lower_bound_.resize(num_constraints_);
    lower_bound_ << current_state_.ToVector(), Eigen::VectorXd::Zero(horizon_ * state_size_), constraints_.MovedXMin().replicate(horizon_ + 1, 1), constraints_.u_min().replicate(horizon_, 1);
}

void MPC::CreateUpperBound()
{
    upper_bound_.resize(num_constraints_);
    upper_bound_ << current_state_.ToVector(), Eigen::VectorXd::Zero(horizon_ * state_size_), constraints_.MovedXMax().replicate(horizon_ + 1, 1), constraints_.u_max().replicate(horizon_, 1);
}

void DoMPC()
{
    // solver
}

Input MPC::solved_input()
{
    return solved_input_;
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