// Copyright (C) 2023 Alessandro Fornasier.
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <alessandro.fornasier@ieee.org>

#include <unsupported/Eigen/MatrixFunctions>

#include "msceqf/symmetry/symmetry.hpp"

namespace msceqf
{
const Matrix5 Symmetry::D = []() {
  Matrix5 D = Matrix5::Zero();
  D(3, 4) = 1.0;
  return D;
}();

// **EqF_Info**
/**
 * @brief Implements the group action φ(X, ξ) that transforms system states.
 *
 * @note **EqF KEY CONCEPT - GROUP ACTION:**
 * The group action defines how the symmetry group G acts on the state space M.
 * For each state component, we have specific transformation rules:
 *
 * - Extended Pose T: T_new = T_old · D        [Paper [2] Eq. (12)]
 * - Bias b: b_new = Ad_B^{-1} · (b_old - δ)   [Paper [2] Eq. (13)]
 * - Extrinsics S: S_new = C^{-1} · S · E      [Paper [2] Eq. (14)]
 * - Intrinsics K: K_new = K · L
 * - Features f: f_new = P·S · Q^{-1} · (P·S)^{-1} · f
 *
 * Standard EKF has no equivalent - states are just vectors.
 *
 * @see Paper [2] Section III-A (Group Action on Homogeneous Space)
 */
const SystemState Symmetry::phi(const MSCEqFState& X, const SystemState& xi)
{
  SE3 PS = xi.P();

  if (X.opts().num_persistent_features_ > 0)
  {
    PS.multiplyRight(xi.S());
  }

  SystemState result(xi);

  for (auto& [key, ptr] : result.state_)
  {
    assert(key.valueless_by_exception() == false);

    if (std::holds_alternative<SystemStateElementName>(key))
    {
      switch (std::get<SystemStateElementName>(key))
      {
        case SystemStateElementName::T:
          std::static_pointer_cast<ExtendedPoseState>(ptr)->T_.multiplyRight(X.D());
          break;
        case SystemStateElementName::b:
          std::static_pointer_cast<BiasState>(ptr)->b_ = X.B().invAdjoint() * (xi.b() - X.delta());
          break;
        case SystemStateElementName::S:
          std::static_pointer_cast<CameraExtrinsicState>(ptr)->S_.multiplyLeft(X.C().inv());
          std::static_pointer_cast<CameraExtrinsicState>(ptr)->S_.multiplyRight(X.E());
          break;
        case SystemStateElementName::K:
          std::static_pointer_cast<CameraIntrinsicState>(ptr)->K_.multiplyRight(X.L());
          break;
      }
    }
    else
    {
      std::static_pointer_cast<FeatureState>(ptr)->f_ =
          PS * (X.Q(std::get<uint>(key)).inv() * (PS.inv() * xi.f(std::get<uint>(key))));
    }
  }

  return result;
}

// **EqF_Info**
/**
 * @brief Implements the lift function λ(ξ, u) that maps inputs to the Lie algebra.
 *
 * @note **EqF KEY CONCEPT - LIFT FUNCTION:**
 * The lift converts IMU measurements (u) to elements of the symmetry group's Lie algebra.
 * This enables propagation on the group manifold rather than in Euclidean space.
 *
 * Lift equations for each component:
 * - λ_T (extended pose): [ω - b_ω, a - b_a + R'g, R'v]  [Paper [2] Eq. (20)-(22)]
 * - λ_b (bias): ad_{b} · λ_T[0:6]                       [Paper [2] Eq. (23)]
 * - λ_S (extrinsics): Ad_{S}^{-1} · [λ_T[0:3], λ_T[6:9]]
 * - λ_f (features): Computed from camera-frame feature position
 *
 * Standard EKF directly uses f(x, u); EqF uses the lift to preserve symmetry.
 *
 * @see Paper [2] Section IV-A (Lifted Dynamics), Equations (20)-(23)
 */
const SystemState::SystemStateAlgebraMap Symmetry::lift(const SystemState& xi, const Imu& u)
{
  SystemState::SystemStateAlgebraMap lambda(xi.state_.size());

  Vector9 lambda_T = Vector9::Zero();
  Vector6 lambda_S = Vector6::Zero();

  lambda_T.block<3, 1>(0, 0) = u.ang_ - xi.b().block<3, 1>(0, 0);
  lambda_T.block<3, 1>(3, 0) = u.acc_ - xi.b().block<3, 1>(3, 0) + xi.T().R().transpose() * xi.ge3();
  lambda_T.block<3, 1>(6, 0) = xi.T().R().transpose() * xi.T().v();

  lambda_S = xi.S().invAdjoint() * (Vector6() << lambda_T.block<3, 1>(0, 0), lambda_T.block<3, 1>(6, 0)).finished();

  SE3 PS_inv = xi.S().inv();

  if (xi.opts_.num_persistent_features_ > 0)
  {
    PS_inv.multiplyRight(xi.P().inv());
  }

  for (auto& [key, ptr] : xi.state_)
  {
    assert(key.valueless_by_exception() == false);

    if (std::holds_alternative<SystemStateElementName>(key))
    {
      switch (std::get<SystemStateElementName>(key))
      {
        case SystemStateElementName::T:
          lambda[key] = lambda_T;
          break;
        case SystemStateElementName::b:
          lambda[key] = SE3::adjoint(xi.b()) * lambda_T.block<6, 1>(0, 0);
          break;
        case SystemStateElementName::S:
          lambda[key] = lambda_S;
          break;
        case SystemStateElementName::K:
          lambda[key] = Vector4::Zero();
          break;
      }
    }
    else
    {
      // Feature expressed in camera frame and its squared depth
      Vector3 Cf = PS_inv * xi.f(std::get<uint>(key));
      fp Cf_depth2 = Cf.squaredNorm();

      // origin expressed in camera frame
      Vector3 Cnu = PS_inv * Vector3(Vector3::Zero());

      Vector4 lambda_f = Vector4::Zero();
      lambda_f.block<3, 1>(0, 0) = lambda_S.block<3, 1>(0, 0) + Cf.cross(lambda_S.block<3, 1>(3, 0) - Cnu) / Cf_depth2;
      lambda_f(3) = Cf.dot(lambda_S.block<3, 1>(3, 0) - Cnu) / Cf_depth2;
      lambda[key] = lambda_f;
    }
  }

  return lambda;
}

// **EqF_Info**
/**
 * @brief Computes the curvature correction matrix for the EqF update step.
 *
 * @note **EqF KEY CONCEPT - CURVATURE CORRECTION:**
 * This is UNIQUE to equivariant filters and has no equivalent in standard EKF.
 *
 * Standard EKF assumes a flat Euclidean state space. The EqF operates on curved
 * Lie group manifolds, and this curvature must be accounted for in the covariance
 * update. The correction is:
 *
 *     P_new = exp(-½Γ) · P · exp(-½Γ)ᵀ
 *
 * where Γ is constructed from adjoint representations of the innovation:
 *     Γ_block = ad(innovation_block)
 *
 * Without this correction, the covariance would be inconsistent with the true
 * uncertainty on the manifold.
 *
 * @see Paper [1] "Equivariant filter (EqF)", Section IV-B, Theorem 2, Eq. (32)-(33)
 * @see OVERVIEW.md Section "Curvature Correction"
 */
const MatrixX Symmetry::curvatureCorrection(const MSCEqFState& X, const VectorX& inn)
{
  MatrixX Gamma = MatrixX::Zero(inn.rows(), inn.rows());

  Gamma.block(X.index(MSCEqFStateElementName::Dd), X.index(MSCEqFStateElementName::Dd), 9, 9) =
      SE23::adjoint(inn.segment(X.index(MSCEqFStateElementName::Dd), 9));

  Gamma.block(X.index(MSCEqFStateElementName::Dd) + 9, X.index(MSCEqFStateElementName::Dd), 6, 6) =
      SE3::adjoint(inn.segment(X.index(MSCEqFStateElementName::Dd) + 9, 6));

  Gamma.block(X.index(MSCEqFStateElementName::Dd) + 9, X.index(MSCEqFStateElementName::Dd) + 9, 6, 6) =
      SE3::adjoint(inn.segment(X.index(MSCEqFStateElementName::Dd), 6));

  Gamma.block(X.index(MSCEqFStateElementName::E), X.index(MSCEqFStateElementName::E), 6, 6) =
      SE3::adjoint(inn.segment(X.index(MSCEqFStateElementName::E), X.dof(MSCEqFStateElementName::E)));

  if (X.opts().enable_camera_intrinsics_calibration_)
  {
    Gamma.block(X.index(MSCEqFStateElementName::L), X.index(MSCEqFStateElementName::L), 4, 4) =
        In::adjoint(inn.segment(X.index(MSCEqFStateElementName::L), X.dof(MSCEqFStateElementName::L)));
  }

  for (auto& [timestamp, clone] : X.clones_)
  {
    Gamma.block(clone->getIndex(), clone->getIndex(), 6, 6) =
        SE3::adjoint(inn.segment(clone->getIndex(), clone->getDof()));
  }

  Gamma *= -0.5;

  return Gamma.exp();
  // return MatrixX::Identity(Gamma.rows(), Gamma.rows()) + Gamma;
}

}  // namespace msceqf
