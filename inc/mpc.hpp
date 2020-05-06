#ifndef MPC_H
#define MPC_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>


#include "constraints.hpp"
#include "state.hpp"
#include "model.hpp"
#include "cost.hpp"
#include "visualizer.hpp"

// Implements Model Predictive Control by minimizing a quadratic programming
// problem given constraints. Uses OSQP for optimization.

class MPC
{
    public:
        MPC();
        MPC(ros::NodeHandle &nh);
        virtual ~MPC();
        // Initializes MPC for its first iteration
        void Init(Model model, Cost cost);
        // Runs one iteration of MPC given the latest state from callback and last MPC input
        // Uses desired_state_trajectory for tracking reference
        void Update(State current_state, Input input, std::vector<State> &desired_state_trajectory);
        // Generates pretty lines
        void Visualize();
        // Updates scan_msg content
        void update_scan(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

        // accessor functions
        Constraints constraints();
        float get_dt();
        int get_horizon();
        std::vector<Input> get_solved_trajectory();
    private:
        int horizon_;
        int input_size_;
        int state_size_;
        int num_states_;
        int num_inputs_;
        int num_variables_;
        int num_constraints_;
        bool solver_init_ = false;
        float dt_;

        Eigen::SparseMatrix<double> hessian_;
        Eigen::VectorXd gradient_;
        Eigen::SparseMatrix<double> linear_matrix_;
        Eigen::VectorXd lower_bound_;
        Eigen::VectorXd upper_bound_;
        Constraints constraints_;
        Model model_;
        State current_state_;
        Input desired_input_;
        std::vector<State> desired_state_trajectory_;
        Cost cost_;
        sensor_msgs::LaserScan scan_msg_;
        Eigen::VectorXd full_solution_;
        OsqpEigen::Solver solver_;
        ros::Time prev_time_;
        std::vector<Input> solved_trajectory_;
        int trajectory_idx_;

        // ros vis stuff
        ros::NodeHandle nh_;
        ros::Publisher mpc_pub_;
        std::vector<geometry_msgs::Point> points_;
        std::vector<std_msgs::ColorRGBA> colors_;

        void CreateHessianMatrix();
        void CreateGradientVector();
        void CreateLinearConstraintMatrix();
        void UpdateLinearConstraintMatrix();
        void CreateLowerBound();
        void CreateUpperBound();
        void UpdateLowerBound();
        void UpdateUpperBound();
        void SparseBlockInit(Eigen::SparseMatrix<double> &modify, const Eigen::MatrixXd &block, int row_start, int col_start);
        void SparseBlockSet(Eigen::SparseMatrix<double> &modify, const Eigen::MatrixXd &block, int row_start, int col_start);
        void SparseBlockEye(Eigen::SparseMatrix<double> &modify, int size, int row_start, int col_start, int number);
        void DrawCar(State &state, Input &input);
        void updateSolvedTrajectory();
};
#endif