#include "mpc.hpp"

// MPC::MPC()constraints_()
// {

// }

MPC::MPC(ros::NodeHandle &nh):
    nh_(nh),
    state_size_(3),
    input_size_(2),
    constraints_(nh)
{
    nh.getParam("/horizon", horizon_);
    nh.getParam("/dt", dt_);
    num_inputs_=(input_size_ * horizon_);
    num_states_=(state_size_ * (horizon_ + 1));
    num_variables_=(num_states_ + num_inputs_);
    num_constraints_=(num_states_ + 2 * (horizon_ + 1) + num_inputs_);
    
    hessian_.resize(num_variables_, num_variables_);
    gradient_.resize(num_variables_);
    linear_matrix_.resize(num_constraints_, num_variables_);
    lower_bound_.resize(num_constraints_);
    upper_bound_.resize(num_constraints_);
    
    full_solution_ = Eigen::VectorXd::Zero(num_variables_);
    mpc_pub_ = nh.advertise<visualization_msgs::Marker>("mpc", 1);
    prev_time_ = ros::Time::now();
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
    desired_input_.SetV(4.5);
    desired_input_.SetSteerAng(0);
    CreateHessianMatrix();
    CreateUpperBound();
    CreateLowerBound();
}

void MPC::update_scan(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    scan_msg_ = *scan_msg;
}

void MPC::Update(State &current_state, std::vector<State> &desired_state_trajectory)
{

    ROS_INFO("updating MPC");
    current_state_ = current_state;
    desired_state_trajectory_ = desired_state_trajectory;
    ros::Time curr_time = ros::Time::now();
    // Input temp(solved_input_.v(), 0);
    // model_.linearize(current_state, solved_input_, (curr_time - prev_time_).toSec());
    Input input(solved_input().v(), solved_input().steer_ang());
    model_.linearize(current_state_, input, dt_);
    // std::cout<< (curr_time - prev_time_).toSec()<<std::endl;
    prev_time_ = curr_time;
    constraints_.set_state(current_state_);
    constraints_.find_half_spaces(current_state_,scan_msg_);
    
    CreateGradientVector();
    CreateLinearConstraintMatrix();
    UpdateLowerBound();
    UpdateUpperBound();
    // // std::cout << gradient_ << std::endl << std::endl;
    if (solver_init_)
    {
        // if (!solver_.updateHessianMatrix(hessian_)) std::cout << "hessian failed" << std::endl;
        if (!solver_.updateGradient(gradient_)) std::cout << "gradient failed" << std::endl;
        if (!solver_.updateLinearConstraintsMatrix(linear_matrix_)) std::cout << "linear failed" << std::endl;
        if (!solver_.updateBounds(lower_bound_, upper_bound_)) std::cout << "bounds failed" << std::endl;
    } else
    {
        solver_.settings()->setVerbosity(false);
        solver_.data()->setNumberOfVariables(num_variables_);
        solver_.data()->setNumberOfConstraints(num_constraints_);
        if (!solver_.data()->setHessianMatrix(hessian_)) std::cout << "hessian failed" << std::endl;
        if (!solver_.data()->setGradient(gradient_)) std::cout << "gradient failed" << std::endl;
        if (!solver_.data()->setLinearConstraintsMatrix(linear_matrix_)) std::cout << "linear failed" << std::endl;
        if (!solver_.data()->setLowerBound(lower_bound_)) std::cout << "lower failed" << std::endl;
        if (!solver_.data()->setUpperBound(upper_bound_)) std::cout << "upper failed" << std::endl;
        if (!solver_.initSolver()) std::cout << "solver failed" << std::endl;
        solver_init_ = true;
    }
    if (!solver_.solve()) {
        std::cout << "solve failed" << std::endl;
    } else {
        full_solution_ = solver_.getSolution();
        updateSolvedTrajectory();
    }
}

void MPC::updateSolvedTrajectory()
{
    solved_path_mutex.lock();
 
    trajectory_idx_ = 0;
    solved_trajectory_.clear(); //full_solution_.size()-1
    for (int i = num_states_; i < 5; i+=2) {
        double v = std::isnan(full_solution_(i)) ? 0.5 : full_solution_(i);
        double angle = std::isnan(full_solution_(i+1)) ? 0.001 : full_solution_(i+1);
        Input input(v, angle);
        solved_trajectory_.push_back(input);
    }
 
    solved_path_mutex.unlock();
}

void MPC::Visualize()
{
    

    // std::cout << "a" << std::endl << model_.a() << std::endl << std::endl << "b" << std::endl << model_.b() << std::endl <<std::endl;

    State predicted_state;
    Input predicted_input;
    for (int ii = 0; ii < horizon_; ++ii)
    {
        
        predicted_state.SetX(full_solution_(ii * state_size_));
        predicted_state.SetY(full_solution_(ii * state_size_ + 1));
        predicted_state.SetOri(full_solution_(ii * state_size_ + 2));

        predicted_input.SetV(full_solution_(num_states_ + ii * input_size_));
        predicted_input.SetSteerAng(full_solution_(num_states_ + ii * input_size_ + 1));
        // std::cout << "state" << std::endl <<predicted_state.ToVector() << std::endl << std::endl << "input" << std::endl << predicted_input.ToVector() <<std::endl << std::endl;
        DrawCar(predicted_state, predicted_input);
        
    }
    
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;

    for (int ii = 0; ii < horizon_-1; ++ii)
    {
        point.x = desired_state_trajectory_[ii].x();
        point.y = desired_state_trajectory_[ii].y();
        point.z = 0.2;
        points_.push_back(point);
        point.x = desired_state_trajectory_[ii].x() + 0.4 * cos(desired_state_trajectory_[ii].ori());
        point.y = desired_state_trajectory_[ii].y() + 0.4 * sin(desired_state_trajectory_[ii].ori());
        point.z = 0.2;
        points_.push_back(point);
        color.r = (float)ii / horizon_;
        color.g = 0;
        color.b = 0;
        color.a = 1;
        colors_.push_back(color);
        color.r = 0;
        color.g = (float)ii / horizon_;
        color.b = 0;
        color.a = 1;
        colors_.push_back(color);
        
    }


    mpc_pub_.publish(Visualizer::GenerateList(points_, colors_, visualization_msgs::Marker::LINE_LIST, 0.05, 0.0, 0.0));
    points_.clear();
    colors_.clear();
}

void MPC::CreateHessianMatrix()
{   
    // hessian_.resize(num_variables_, num_variables_);
    hessian_.setZero();
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
    // Eigen::VectorXd state_costs;
    // state_costs.setZero(num_states_, 1);
    for (int ii  = 0; ii < horizon_; ++ii)
    {
        gradient_.block(ii * state_size_, 0, state_size_, 1) = -1 * cost_.q() * desired_state_trajectory_[ii].ToVector();
        // gradient_.block(num_states_ + ii * input_size_, 0, input_size_, 1) = -1 * cost_.r() * desired_input_.ToVector();
    }
    gradient_.block(horizon_ * state_size_, 0, 3, 1) = -1 * cost_.q() * desired_state_trajectory_[horizon_ - 1].ToVector();
    gradient_.tail(horizon_ * input_size_) = Eigen::VectorXd::Zero(horizon_ * input_size_);
    // gradient_ << state_costs, Eigen::VectorXd::Zero(horizon_ * input_size_);
    // std::cout << gradient_ << std::endl << std::endl;
    // gradient_ << (-1 * cost_.q() * desired_state_.ToVector()).replicate(horizon_ + 1, 1), Eigen::VectorXd::Zero(horizon_ * input_size_);
}

void MPC::CreateLinearConstraintMatrix()
{
    // here for steering angle vs throttle
    linear_matrix_.setZero();
    
    Eigen::MatrixXd a_eye(state_size_, 2 * state_size_);
    a_eye << model_.a() , -Eigen::MatrixXd::Identity(state_size_, state_size_);
    Eigen::MatrixXd gap_con(2, state_size_);
    gap_con(0, 0) = constraints_.l1()(0);
    gap_con(0, 1) = constraints_.l1()(1);
    gap_con(0, 2) = 0;
    gap_con(1, 0) = constraints_.l2()(0);
    gap_con(1, 1) = constraints_.l2()(1);
    gap_con(1, 2) = 0;
    
    // std::cout << gap_con << std::endl << std::endl;
    // prediction equality constraint
    // SparseBlockSet(linear_matrix_, Eigen::MatrixXd::Identity(state_size_, state_size_), 0, 0);
    SparseBlockSet(linear_matrix_, gap_con, num_states_, 0);
    // // here for steering angle vs throttle
    // linear_matrix_.resize(num_constraints_, num_variables_);
    
    // Eigen::MatrixXd a_eye(state_size_, 2 * state_size_);
    // a_eye << model_.a() , -Eigen::MatrixXd::Identity(state_size_, state_size_);

    // prediction equality constraint
    SparseBlockEye(linear_matrix_, num_states_, 0, 0, -1);
    for (int ii = 1; ii < horizon_ + 1; ++ii)
    {
        // SparseBlockSet()
        SparseBlockSet(linear_matrix_, model_.a(), ii*state_size_, (ii - 1) * state_size_);
        // SparseBlockSet(linear_matrix_, Eigen::MatrixXd::Identity(state_size_, state_size_), ii*state_size_, ii * state_size_);
        SparseBlockSet(linear_matrix_, model_.b(), ii*state_size_, num_states_ + (ii - 1) * input_size_);
        SparseBlockSet(linear_matrix_, gap_con, num_states_ + 2 * (ii), (ii)*state_size_);
        // std::cout << linear_matrix_ << std::endl << std::endl;
    }
    
    // l1 A B C
    // state and input upper and lower bounds
    SparseBlockEye(linear_matrix_, num_inputs_, num_states_ + 2 * (horizon_ + 1), num_states_, 1);
    // std::cout << linear_matrix_ << std::endl << std::endl;
    
    // SparseBlockEye(linear_matrix_, linear_matrix_.cols(), (horizon_ + 1) * state_size_, 0, 1);
    // std::cout << linear_matrix_ << std::endl <<std::endl;
    // std::cout << model_.a() << std::endl <<std::endl;
    // std::cout << model_.b() << std::endl <<std::endl;
}

void MPC::CreateLowerBound()
{
    lower_bound_.resize(num_constraints_);
    Eigen::VectorXd gap_con(2);
    gap_con(0) = -OsqpEigen::INFTY;
    gap_con(1) = -OsqpEigen::INFTY;

    lower_bound_ << Eigen::VectorXd::Zero((horizon_ + 1) * state_size_), gap_con.replicate(horizon_ + 1, 1), constraints_.u_min().replicate(horizon_, 1);
    // lower_bound_.resize(num_constraints_);
    // lower_bound_ << Eigen::VectorXd::Zero((horizon_ + 1) * state_size_), constraints_.MovedXMin().replicate(horizon_ + 1, 1), constraints_.u_min().replicate(horizon_, 1);
    // std::cout << lower_bound_ << std::endl << std::endl;
}

void MPC::CreateUpperBound()
{
    upper_bound_.resize(num_constraints_);
    Eigen::VectorXd gap_con(2);
    gap_con(0) = OsqpEigen::INFTY;
    gap_con(1) = OsqpEigen::INFTY;
    // std::cout << gap_con << std::endl << std::endl;
    upper_bound_ << Eigen::VectorXd::Zero((horizon_ + 1) * state_size_), gap_con.replicate(horizon_ + 1, 1), constraints_.u_max().replicate(horizon_, 1);
    // upper_bound_.resize(num_constraints_);
    // upper_bound_ << Eigen::VectorXd::Zero((horizon_ + 1) * state_size_), constraints_.MovedXMax().replicate(horizon_ + 1, 1), constraints_.u_max().replicate(horizon_, 1);
    // std::cout << upper_bound_ << std::endl << std::endl;
}

void MPC::UpdateLowerBound()
{   
    lower_bound_.resize(num_constraints_);
    Eigen::VectorXd gap_con(2);
    gap_con(0) = -constraints_.l1()(2);
    gap_con(1) = -constraints_.l2()(2);
    lower_bound_.head(num_states_) << -current_state_.ToVector(), -model_.c().replicate(horizon_, 1);
    lower_bound_.block(num_states_, 0, (horizon_ + 1) * 2, 1) = gap_con.replicate(horizon_ + 1, 1);
    // std::cout << lower_bound_ << std::endl << std::endl;
}

void MPC::UpdateUpperBound()
{
    upper_bound_.head(num_states_) << -current_state_.ToVector(), -model_.c().replicate(horizon_, 1);
    // std::cout << upper_bound_ << std::endl << std::endl;
}

Input MPC::solved_input()
{
    if (trajectory_idx_ >= solved_trajectory_.size()) {
        ROS_ERROR("Trajectory completed!");
        return Input(0.5,-0.1);
    }
    return solved_trajectory_[trajectory_idx_];
}

void MPC::increment_solved_path()
{
    solved_path_mutex.lock();
    trajectory_idx_++;
    solved_path_mutex.unlock();
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

void MPC::SparseBlockEye(Eigen::SparseMatrix<double> &modify, int size, int row_start, int col_start, int number)
{
    for (int row = 0; row < size; ++row)
    {
        modify.insert(row_start + row, col_start + row) = number;
    }
}

void MPC::DrawCar(State &state, Input &input)
{
    float L = 0.3302f;
    float wheel_size = 0.2;
    float arrow_width = 0.05;
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;

    point.x = state.x();
    point.y = state.y();
    point.z = 0.1;
    points_.push_back(point);

    point.x = state.x() + L * cos(state.ori());
    point.y = state.y() + L * sin(state.ori());
    point.z = 0.1;
    points_.push_back(point);
    
    point.x = state.x() + L * cos(state.ori()) - 0.5 * wheel_size * cos(state.ori() + input.steer_ang());
    point.y = state.y() + L * sin(state.ori()) - 0.5 * wheel_size * sin(state.ori() + input.steer_ang());
    point.z = 0.125;
    points_.push_back(point);

    point.x = state.x() + L * cos(state.ori()) + 0.5 * wheel_size * cos(state.ori() + input.steer_ang());
    point.y = state.y() + L * sin(state.ori()) + 0.5 * wheel_size * sin(state.ori() + input.steer_ang());
    
    point.z = 0.125;
    points_.push_back(point);

    point.x = state.x() + L * 0.5 * cos(state.ori());
    point.y = state.y() + L * 0.5 * sin(state.ori());
    point.z = 0.15;
    points_.push_back(point);
    point.x = state.x() + L * 0.5 * cos(state.ori()) + L * 0.5 * (input.v() / constraints_.u_max()(0)) * cos(state.ori());
    point.y = state.y() + L * 0.5 * sin(state.ori()) + L * 0.5 * (input.v() / constraints_.u_max()(0)) * sin(state.ori());
    point.z = 0.15;
    points_.push_back(point);
    

    
    color.r = 0;
    color.g = 0;
    color.b = 1;
    color.a = 1;
    colors_.push_back(color);
    colors_.push_back(color);

    color.r = 0;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    colors_.push_back(color);
    colors_.push_back(color);

    color.r = 1;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    colors_.push_back(color);
    colors_.push_back(color);
}