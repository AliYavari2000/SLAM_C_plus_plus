/******************************************************************************
 * slam_ekf_eigen.cpp
 * 
 * A conceptual EKF-SLAM demonstration using Eigen for matrix operations.
 * 
 * Make sure you have Eigen installed (e.g., "sudo apt-get install libeigen3-dev"
 * on Ubuntu). Then compile with:
 *    g++ -I./eigen -std=c++14 slam_ekf_eigen.cpp -o ekf_slam
 ******************************************************************************/

 #include <iostream>
 #include <Eigen/Dense>
 #include <random>
 #include <cmath>
 #include <vector>
 #include <algorithm>
 #include <fstream>
 
 // Global constants
 static constexpr double EPSILON = 1e-6;
 static constexpr double PI = 3.14159265358979323846;
 
 /**
  * @brief Normalize angle to the range [-pi, pi].
  */
 double wrapToPi(double angle) {
     angle = std::fmod(angle + PI, 2.0 * PI);
     if (angle < 0)
         angle += 2.0 * PI;
     return angle - PI;
 }
 
 /**
  * @brief Generate landmarks in the square [-l, l] x [-l, l] or in some fixed pattern.
  */
 Eigen::Matrix2Xd landmarks_generate(int l, int n, bool random_landmark) {
     // Return a 2 x n matrix of landmark coordinates
     Eigen::Matrix2Xd L(2, n);
     static std::mt19937 gen(12345); // fixed seed
     std::uniform_real_distribution<double> dist(-l, l);
 
     if (random_landmark) {
         for (int i = 0; i < n; ++i) {
             L(0, i) = dist(gen);
             L(1, i) = dist(gen);
         }
     } else {
        L(0, 0) = -2.9216; L(1, 0) = 8.3048; // Landmark 1   
        L(0, 1) = -0.1186; L(1, 1) = 0.643; // Landmark 2
         // Provide some fixed pattern; expand as needed for more landmarks
         
     }
     return L;
 }
 
 /**
  * @brief Unicycle robot motion (one-step update).
  * 
  * X: (x, y, theta)
  * u: (v, w) 
  * dt: time step
  */
 Eigen::Vector3d robot_motion(const Eigen::Vector3d &X, double v, double w, double dt) {
     Eigen::Vector3d Xnew = X;
     double theta = X(2);
     if (std::fabs(w) > EPSILON) {
         Xnew(0) += -v / w * std::sin(theta) + (v / w) * std::sin(theta + w * dt);
         Xnew(1) +=  v / w * std::cos(theta) - (v / w) * std::cos(theta + w * dt);
     } else {
         // Straight-line motion when w ~ 0
         Xnew(0) += v * std::cos(theta) * dt;
         Xnew(1) += v * std::sin(theta) * dt;
     }
     Xnew(2) = wrapToPi(theta + w * dt);
     return Xnew;
 }
 
 /**
  * @brief Jacobian of the robot motion model w.r.t. the robot state X = [x, y, theta].
  */
 Eigen::Matrix3d jacobian_robot_motion(const Eigen::Vector3d &X, double v, double w, double dt) {
     Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
     double theta = X(2);
     if (std::fabs(w) > EPSILON) {
         A(0,2) =  v/w * (std::cos(theta + w*dt) - std::cos(theta));
         A(1,2) =  v/w * (std::sin(theta + w*dt) - std::sin(theta));
     } else {
         A(0,2) = -v * dt * std::sin(theta);
         A(1,2) =  v * dt * std::cos(theta);
     }
     return A;
 }
 
 /**
  * @brief Inverse-landmark observation.
  * Given robot state Xr = [x, y, theta] and measurement z = [range, bearing],
  * compute the landmark's (x,y) and the corresponding Jacobians.
  */
 struct InverseObsResult {
     Eigen::Vector2d landmark; // [lx, ly]
     Eigen::Matrix<double,2,3> dR; 
     Eigen::Matrix2d dL;
 };
 InverseObsResult inverse_landmark_obsv(const Eigen::Vector3d &Xr, const Eigen::Vector2d &z) {
     double r = z(0);
     double phi = z(1);
     double phi0 = wrapToPi(phi + Xr(2));
     double lx = Xr(0) + r * std::cos(phi0);
     double ly = Xr(1) + r * std::sin(phi0);
     Eigen::Vector2d L;
     L << lx, ly;
 
     // Jacobian with respect to robot state
     Eigen::Matrix<double,2,3> dR;
     dR.setZero();
     dR(0,0) = 1.0;
     dR(1,1) = 1.0;
     dR(0,2) = -r * std::sin(phi0);
     dR(1,2) =  r * std::cos(phi0);
 
     // Jacobian with respect to measurement z
     Eigen::Matrix2d dL_2;
     dL_2(0,0) = std::cos(phi0);
     dL_2(0,1) = -r*std::sin(phi0);
     dL_2(1,0) = std::sin(phi0);
     dL_2(1,1) =  r*std::cos(phi0);
 
     return {L, dR, dL_2};
 }
 
 /**
  * @brief Landmark measurement model: from robot state Xr and landmark L, compute expected measurement h = [range, bearing].
  * Also returns the Jacobian C (2x5) with respect to [rx, ry, rtheta, lx, ly].
  */
 struct LandmarkEstimateResult {
     Eigen::Vector2d h;       // [range, bearing]
     Eigen::Matrix<double,2,5> C;  // Jacobian
 };
 LandmarkEstimateResult landmark_estimate(const Eigen::Vector3d &Xr, const Eigen::Vector2d &L) {
     double dx = L(0) - Xr(0);
     double dy = L(1) - Xr(1);
     double R  = std::sqrt(dx*dx + dy*dy);
     double phi = wrapToPi(std::atan2(dy, dx) - Xr(2));
     Eigen::Vector2d h;
     h << R, phi;
 
     double R_inv = (R < EPSILON) ? 0.0 : 1.0 / R;
     double dRdx  = -dx * R_inv;
     double dRdy  = -dy * R_inv;
     double dRdlx =  dx * R_inv;
     double dRdly =  dy * R_inv;
     double denom = (dx*dx + dy*dy);
     if (denom < EPSILON) denom = EPSILON;
     double dphidx  =  dy / denom;
     double dphidy  = -dx / denom;
     double dphidtheta = -1.0;
     double dphidlx = -dy / denom;
     double dphidly =  dx / denom;
 
     Eigen::Matrix<double,2,5> C;
     // Row for range
     C(0,0) = dRdx;
     C(0,1) = dRdy;
     C(0,2) = 0.0;
     C(0,3) = dRdlx;
     C(0,4) = dRdly;
     // Row for bearing
     C(1,0) = dphidx;
     C(1,1) = dphidy;
     C(1,2) = dphidtheta;
     C(1,3) = dphidlx;
     C(1,4) = dphidly;
 
     return {h, C};
 }
 
 /**
  * @brief Compute robot error compared to ground truth.
  */
 Eigen::Vector3d robot_error(const Eigen::Vector3d &X_est, const Eigen::Vector3d &X_true) {
     Eigen::Vector3d err;
     double dx = X_est(0) - X_true(0);
     double dy = X_est(1) - X_true(1);
     double dth = wrapToPi(X_est(2) - X_true(2));
     err << dx*dx, dy*dy, dth;
     return err;
 }
 
 /**
  * @brief Compute sum of squared errors in x and y for a set of landmarks.
  */
 Eigen::Vector2d landmark_sum_mse(const Eigen::Matrix2Xd &L_real, const Eigen::Matrix2Xd &L_est) {
     if (L_real.cols() != L_est.cols()) {
         return Eigen::Vector2d::Zero();
     }
     Eigen::Vector2d out(0.0, 0.0);
     for (int i = 0; i < L_real.cols(); i++) {
         double dx = L_real(0,i) - L_est(0,i);
         double dy = L_real(1,i) - L_est(1,i);
         out(0) += dx*dx;
         out(1) += dy*dy;
     }
     return out;
 }

 void debug_print_matrix(const Eigen::MatrixXd& M) {
    std::cout << "Matrix (" << M.rows() << "x" << M.cols() << "):\n" << M << "\n";
}

 
 // ----------------------------------------------------------------------------
 // Main demonstration of EKF-SLAM with Eigen (revised version)
 // ----------------------------------------------------------------------------
 int main() {
     // ------------------ Initialization ------------------
     int iteration = 3000;
     double dt = 0.1;
 
     int landmark_number = 20;
     int state_number = 3 + 2 * landmark_number; // 3 for robot, 2 per landmark
     
 
     // State vector X: [robot; landmarks]
     Eigen::VectorXd X = Eigen::VectorXd::Zero(state_number);
 
     // Covariance P:
     Eigen::MatrixXd P = Eigen::MatrixXd::Zero(state_number, state_number);
     // Robot covariance ~ 0.1
     P(0,0) = 0.1;
     P(1,1) = 0.1;
     P(2,2) = 0.1;
     // Landmarks: high initial uncertainty
     for (int i = 0; i < landmark_number; i++) {
         int Lx = 3 + 2 * i;
         int Ly = Lx + 1;
         P(Lx, Lx) = 10000.0;
         P(Ly, Ly) = 10000.0;
     }

 
     // Generate landmarks
     bool random_landmark = true;  // true for random, false for fixed pattern
     int map_length = 10;
     Eigen::Matrix2Xd L = landmarks_generate(map_length, landmark_number, random_landmark);

     // Storage for landmark estimates
     Eigen::Matrix2Xd L_est = Eigen::Matrix2Xd::Zero(2, landmark_number);
     Eigen::Matrix2Xd L_ekf_free = Eigen::Matrix2Xd::Zero(2, landmark_number);
 
     // Observed landmarks (by id)
     std::vector<int> landmarks_observed;
     

     //std::cout << L << "\n";
     // Measurement matrix Y: (2 x landmark_number)
     Eigen::Matrix2Xd Y(2, landmark_number);
     Y.setZero();
 
     // Noise standard deviations
     double qv = 3.0 * 0.01;       // process noise for v
     double qw = 3.0 * (PI/1000);   // process noise for w
     double vv = 0.1;              // measurement noise in range
     double vw = PI/180.0;         // measurement noise in bearing
 
     // Control inputs
     double u_v = 1.0;
     double u_w = PI/10.0;
 
     // Ground-truth (expected) robot state
     Eigen::Vector3d X_expect(0.0, 0.0, 0.0);
 
     // Logging vectors
     std::vector<Eigen::Vector3d> robot_error_log(iteration, Eigen::Vector3d::Zero());
     std::vector<Eigen::Vector2d> landmark_error_log(iteration, Eigen::Vector2d::Zero());
 
     // Generate observation sequence (shuffled landmark ids)
     std::vector<int> obsv_sequence(landmark_number);
     for (int i = 0; i < landmark_number; i++) {
         obsv_sequence[i] = i;
     }

     std::mt19937 rng(42);
     std::shuffle(obsv_sequence.begin(), obsv_sequence.end(), rng);

 
     std::normal_distribution<double> gauss(0.0, 1.0);
     int nextLandmarkIndexToObserve = 0;
 
     // Maintain list of indices in X that are “active” (robot always active)
     std::vector<int> states_observed = {0, 1, 2};
 
     // Precompute measurement noise covariance matrix R_meas
     Eigen::Matrix2d R_meas = Eigen::Matrix2d::Zero();
     R_meas(0,0) = vv * vv;
     R_meas(1,1) = vw * vw;
 
     // -------------------- Main EKF Loop --------------------
     for (int j = 0; j < iteration; j++) {
 
         // 1) Possibly initialize a new landmark
         if (nextLandmarkIndexToObserve < landmark_number) {
             int lm_id = obsv_sequence[nextLandmarkIndexToObserve];
             nextLandmarkIndexToObserve++;
 
             int Lx_idx = 3 + 2 * lm_id;
             int Ly_idx = Lx_idx + 1;
 
             // Add new landmark indices to active states
             states_observed.push_back(Lx_idx);
             states_observed.push_back(Ly_idx);
 
             // Simulate measurement from ground truth with noise
             double dx = L(0, lm_id) - X_expect(0);
             double dy = L(1, lm_id) - X_expect(1);
             double rng_meas = std::sqrt(dx * dx + dy * dy) + vv * gauss(rng);
             double brg_meas = wrapToPi(std::atan2(dy, dx) - X_expect(2) + vw * gauss(rng));
             Eigen::Vector2d z_new(rng_meas, brg_meas);
 
             // Inverse observation to initialize landmark position
             auto invRes = inverse_landmark_obsv(X_expect, z_new);
             X(Lx_idx) = invRes.landmark(0);
             X(Ly_idx) = invRes.landmark(1);
 
             // Update covariance for new landmark
             Eigen::Matrix<double,2,3> dR = invRes.dR;
             Eigen::Matrix2d dL_ = invRes.dL;
             Eigen::Matrix3d Prr = P.block<3,3>(0,0);
             Eigen::Matrix2d newLandCov = dR * Prr * dR.transpose() + dL_ * R_meas * dL_.transpose();
             P.block<2,2>(Lx_idx, Lx_idx) = newLandCov;

            int n_obs = states_observed.size() - 2;
            Eigen::MatrixXd P_xL(3, n_obs);

            for (int i = 0; i < n_obs; i++) {
                P_xL.col(i) = P.block(0, states_observed[i], 3, 1); }
            
            Eigen::MatrixXd newP_xL = dR * P_xL;

            for (int i = 0; i < n_obs; i++) {
                P.block(Lx_idx, states_observed[i], 2, 1) = newP_xL.col(i);
                P.block(states_observed[i], Lx_idx, 1, 2) = newP_xL.col(i).transpose();  
         }
        
        // Mark landmark as observed
         landmarks_observed.push_back(lm_id);

        }
        
         
         std::cout << P << "\n";
         // 2) Update ground-truth robot state
         X_expect = robot_motion(X_expect, u_v, u_w, dt);

    
         // Update covariance for robot state
 
         // 3) EKF Prediction Step:
         {
             // Generate noise once for consistency in state update and Jacobian computation
             double noise_v = qv * gauss(rng);
             double noise_w = qw * gauss(rng);
             double u_n_v = u_v + noise_v;
             double u_n_w = u_w + noise_w;
 
             // Predict robot state (first three elements)
             Eigen::Vector3d Xr = X.segment<3>(0);
             Xr = robot_motion(Xr, u_n_v, u_n_w, dt);
             X.segment<3>(0) = Xr;
 
             // Compute robot Jacobian
             Eigen::Matrix3d A = jacobian_robot_motion(Xr, u_n_v, u_n_w, dt);
 
             // Update robot covariance block P(0:2,0:2)
             Eigen::Matrix3d Prr = P.block<3,3>(0,0);
             P.block<3,3>(0,0) = A * Prr * A.transpose();
            
            int n_obs = states_observed.size() - 3;
            Eigen::MatrixXd P_rL(3, n_obs);
            for (int i = 0; i < n_obs; i++) {
                P_rL.col(i) = P.block(0, states_observed[i+3], 3, 1);
            }
            Eigen::MatrixXd newP_rL = A * P_rL;
            for (int i = 0; i < n_obs; i++) {
                P.block(0, states_observed[i+3], 3, 1) = newP_rL.col(i);
                P.block(states_observed[i+3], 0, 1, 3) = newP_rL.col(i).transpose();
                 }
             }
         

        //std::cout << P << "\n";

        
        // Insert simulated measurements for all observed landmarks here:
        for (int lm_id : landmarks_observed) {
            double dx = L(0, lm_id) - X_expect(0);
            double dy = L(1, lm_id) - X_expect(1);
            double rng_meas = std::sqrt(dx * dx + dy * dy) + vv * gauss(rng);
            double brg_meas = wrapToPi(std::atan2(dy, dx) - X_expect(2) + vw * gauss(rng));
            Y(0, lm_id) = rng_meas;
            Y(1, lm_id) = brg_meas;
        }


 
         // 4) EKF Measurement Update for each observed landmark:
         //     (Now using a full-state update over the indices in states_observed)
         for (auto &lm_id : landmarks_observed) {
             int Lx_idx = 3 + 2 * lm_id;
             int Ly_idx = Lx_idx + 1;
 
             // Get current robot state and landmark estimate from X
             Eigen::Vector3d Xr = X.segment<3>(0);
             Eigen::Vector2d Lm(X(Lx_idx), X(Ly_idx));
             auto measRes = landmark_estimate(Xr, Lm);
             Eigen::Vector2d h = measRes.h;
 
             // Measurement innovation: compare simulated measurement Y(:,lm_id) with prediction
             double y_range   = Y(0, lm_id) - h(0);
             double y_bearing = wrapToPi(Y(1, lm_id) - h(1));
             Eigen::Vector2d innovation(y_range, y_bearing);
 
             // Build full measurement Jacobian H_full (2 x state_number)
             // It is zero except in the columns corresponding to the robot (indices 0-2)
             // and the current landmark (Lx_idx and Ly_idx) where it equals measRes.C.
             Eigen::MatrixXd H_full = Eigen::MatrixXd::Zero(2, state_number);
             std::vector<int> idx_temp = {0, 1, 2, Lx_idx, Ly_idx};
             for (int j = 0; j < 5; j++) {
                 H_full.col(idx_temp[j]) = measRes.C.col(j);
             }
 
             // Standard EKF update (full state update)
             Eigen::Matrix2d S = H_full * P * H_full.transpose() + R_meas;
             Eigen::MatrixXd K = P * H_full.transpose() * S.inverse();
             X = X + K * innovation;
             X(2) = wrapToPi(X(2));  // ensure heading is normalized
             P = P - K * H_full * P;
 
             // Also update the EKF-free landmark estimate (using inverse observation)
             Eigen::Vector2d Y_i(Y(0, lm_id), Y(1, lm_id));
             auto invFree = inverse_landmark_obsv(X.segment<3>(0), Y_i);
             L_ekf_free.col(lm_id) = invFree.landmark;
             // Update the current landmark estimate from the state vector
             L_est(0, lm_id) = X(Lx_idx);
             L_est(1, lm_id) = X(Ly_idx);
         }
 
         // Logging of errors
         {
             Eigen::Vector3d err = robot_error(X.head<3>(), X_expect);
             robot_error_log[j] = err;
 
             if (!landmarks_observed.empty()) {
                 Eigen::Matrix2Xd Lobs_real(2, landmarks_observed.size());
                 Eigen::Matrix2Xd Lobs_est(2,  landmarks_observed.size());
                 for (int k = 0; k < (int)landmarks_observed.size(); k++) {
                     int id = landmarks_observed[k];
                     Lobs_real.col(k) = L.col(id);
                     Lobs_est.col(k)  = L_est.col(id);
                 }
                 Eigen::Vector2d lerr = landmark_sum_mse(Lobs_real, Lobs_est);
                 landmark_error_log[j] = lerr;
             } else {
                 landmark_error_log[j] = Eigen::Vector2d::Zero();
             }
         }
     }
 
     // Write logs to files
     {
         std::ofstream fr("robot_error.txt");
         for (int i = 0; i < iteration; i++) {
             fr << i << " " << robot_error_log[i](0) << " "
                << robot_error_log[i](1) << " " << robot_error_log[i](2) << "\n";
         }
         fr.close();
     }
     {
         std::ofstream fl("landmark_error.txt");
         for (int i = 0; i < iteration; i++) {
             fl << i << " " << landmark_error_log[i](0) << " "
                << landmark_error_log[i](1) << "\n";
         }
         fl.close();
     }
 
     std::cout << "EKF-SLAM (Eigen-based) finished.\n";
     std::cout << "Wrote robot_error.txt and landmark_error.txt.\n";
     return 0;
 }
 