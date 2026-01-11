#pragma once

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <Eigen/Core>

// Vertex and edge types used to formulate a simplified TEB-like optimization
// VertexPose: x, y, theta
class VertexPose : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexPose() {}
  virtual void setToOriginImpl() override { _estimate = Eigen::Vector3d::Zero(); }
  virtual void oplusImpl(const double* update) override {
    Eigen::Map<const Eigen::Vector3d> u(update);
    _estimate += u;
  }
  virtual bool read(std::istream& is) { (void)is; return false; }
  virtual bool write(std::ostream& os) const { (void)os; return false; }
};

// Unary edge that pulls a vertex to a target pose (path alignment)
class EdgePoseToPoint : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePoseToPoint() {}
  void computeError() override {
    const VertexPose* v = static_cast<const VertexPose*>(_vertices[0]);
    _error = v->estimate() - _measurement; // simple difference
  }
  virtual bool read(std::istream& is) { (void)is; return false; }
  virtual bool write(std::ostream& os) const { (void)os; return false; }
};

// Binary edge that enforces smoothness between consecutive poses
class EdgeSmoothness : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexPose, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSmoothness() {}
  void computeError() override {
    const VertexPose* v1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* v2 = static_cast<const VertexPose*>(_vertices[1]);
    _error = (v1->estimate() - v2->estimate());
  }
  virtual bool read(std::istream& is) { (void)is; return false; }
  virtual bool write(std::ostream& os) const { (void)os; return false; }
};

// Unary obstacle edge: measurement is obstacle point (x,y); penalize being close
class EdgeObstacle : public g2o::BaseUnaryEdge<3, Eigen::Vector2d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double safety_radius_{0.2};
  EdgeObstacle() {}
  void computeError() override {
    const VertexPose* v = static_cast<const VertexPose*>(_vertices[0]);
    Eigen::Vector2d pos(v->estimate().x(), v->estimate().y());
    Eigen::Vector2d obs = _measurement;
    Eigen::Vector2d diff = pos - obs;
    double d = diff.norm();
    if (d > safety_radius_) {
      _error.setZero();
    } else {
      // encourage moving away from the obstacle
      Eigen::Vector2d e = (safety_radius_ - d) * (diff.normalized());
      _error << e.x(), e.y(), 0.0; // only x,y components used
    }
  }
  virtual bool read(std::istream& is) { (void)is; return false; }
  virtual bool write(std::ostream& os) const { (void)os; return false; }
};

// Kinematic constraint edge for differential drive robots
class EdgeKinematics : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPose, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double max_vel_x_{1.0};
  double max_vel_theta_{1.0};
  double dt_{0.1}; // time step between poses
  
  EdgeKinematics() {}
  
  void computeError() override {
    const VertexPose* v1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* v2 = static_cast<const VertexPose*>(_vertices[1]);
    
    Eigen::Vector3d pose1 = v1->estimate();
    Eigen::Vector3d pose2 = v2->estimate();
    
    // Compute required velocities
    double dx = pose2.x() - pose1.x();
    double dy = pose2.y() - pose1.y();
    double dtheta = pose2.z() - pose1.z();
    
    // Normalize angular difference
    while (dtheta > M_PI) dtheta -= 2.0*M_PI;
    while (dtheta < -M_PI) dtheta += 2.0*M_PI;
    
    // Transform to robot frame
    double cos_theta = std::cos(pose1.z());
    double sin_theta = std::sin(pose1.z());
    double v_x = (dx * cos_theta + dy * sin_theta) / dt_;
    double v_theta = dtheta / dt_;
    
    // Penalize velocities that exceed limits
    _error.setZero();
    if (std::abs(v_x) > max_vel_x_) {
      _error[0] = std::abs(v_x) - max_vel_x_;
    }
    if (std::abs(v_theta) > max_vel_theta_) {
      _error[1] = std::abs(v_theta) - max_vel_theta_;
    }
  }
  
  virtual bool read(std::istream& is) { (void)is; return false; }
  virtual bool write(std::ostream& os) const { (void)os; return false; }
};