# MSCEqF: Equivariant Filter Overview

This document provides an in-depth overview of the **Multi State Constraint Equivariant Filter (MSCEqF)** implementation and explains the key differences from standard MSCKF/Extended Kalman Filter (EKF) implementations.

## Table of Contents

1. [Introduction](#introduction)
2. [Theoretical Background](#theoretical-background)
3. [Key Differences from Standard EKF/MSCKF](#key-differences-from-standard-ekfmsckf)
4. [Code Architecture](#code-architecture)
5. [Detailed Component Analysis](#detailed-component-analysis)
6. [References](#references)

---

## Introduction

MSCEqF is based on the **Equivariant Filter (EqF)** framework, which exploits the symmetry structure of the state estimation problem to achieve better consistency and convergence properties compared to traditional EKF-based approaches.

The implementation draws from the following key papers:

- **[1]** van Goor, Pieter, Tarek Hamel, and Robert Mahony. "Equivariant filter (EqF)." *IEEE Transactions on Automatic Control* (2022).
- **[2]** Fornasier, Alessandro, et al. "Equivariant filter design for inertial navigation systems with input measurement biases." *2022 ICRA*. IEEE, 2022.
- **[3]** Fornasier, Alessandro, et al. "Overcoming Bias: Equivariant Filter Design for Biased Attitude Estimation with Online Calibration." *IEEE Robotics and Automation Letters* 7.4 (2022).
- **[4]** Fornasier, Alessandro, et al. "Equivariant Symmetries for Inertial Navigation Systems." *ArXiv preprint*.

---

## Theoretical Background

### The Equivariant Filter Framework

The EqF operates on the principle that many estimation problems possess natural **symmetry structures**. Instead of working directly in the state space, the filter operates on a **symmetry group** (Lie group) that acts on the state space.

#### Key Concepts

| Concept | Symbol | Description |
|---------|--------|-------------|
| **State Space** | M | Homogeneous space where the physical state lives |
| **Symmetry Group** | G | Lie group acting on M |
| **Origin** | ξ₀ | Fixed reference point in M |
| **Group Action** | φ(X, ξ) | How group element X transforms state ξ |
| **Lift** | λ(ξ, u) | Maps inputs to Lie algebra elements |

The fundamental relationship is:

```
ξ = φ(X, ξ₀)
```

where ξ is the current state estimate, X is the filter state (group element), and ξ₀ is the fixed origin.

---

## Key Differences from Standard EKF/MSCKF

### 1. State Representation

| Aspect | Standard EKF/MSCKF | MSCEqF |
|--------|-------------------|--------|
| **State space** | Vector space ℝⁿ | Lie group (symmetry group) |
| **State variables** | Vectors, quaternions | Lie group elements |
| **Covariance** | On tangent space at current estimate | On Lie algebra (fixed) |

**Code Reference:** [`include/msceqf/state/state_elements.hpp`](include/msceqf/state/state_elements.hpp)

**Paper Reference:** [2] Section II-A (System Model), Section III (Symmetry Group Construction)

### 2. Semi-Direct Bias Group (SDB)

The most significant innovation is the **Semi-Direct Bias (SDB) group**, which encodes IMU biases into the symmetry structure itself.

**Standard EKF:** Biases are separate vector states added to the state vector.

**MSCEqF:** Biases are part of a semi-direct product group:

```
SDB = SE₂(3) ⋉ ℝ⁶
```

where:

- SE₂(3) is the extended pose group (rotation, velocity, position) - 9 DOF
- ℝ⁶ represents the IMU biases (gyroscope + accelerometer) - 6 DOF

This construction ensures that the bias dynamics are compatible with the symmetry structure.

**Code Reference:** [`include/msceqf/state/state_elements.hpp:115-152`](include/msceqf/state/state_elements.hpp) - `MSCEqFSDBState` class

**Paper Reference:** [2] Section III-B (Semi-Direct Bias Group), Equation (15)

### 3. Group Action φ(X, ξ)

The group action defines how the symmetry group transforms states.

**Standard EKF:** No equivalent concept.

**MSCEqF:** The group action φ: G × M → M satisfies:

- φ(I, ξ) = ξ (identity preserves state)
- φ(X₂, φ(X₁, ξ)) = φ(X₁·X₂, ξ) (composition property)

**Code Reference:** [`source/msceqf/symmetry/symmetry.cpp:24-66`](source/msceqf/symmetry/symmetry.cpp) - `Symmetry::phi()`

**Paper Reference:** [2] Section III-A (Group Action), Proposition 1

### 4. Lift Function λ(ξ, u)

The lift function maps system inputs to the Lie algebra of the symmetry group.

**Standard EKF:** Direct use of system dynamics f(x, u).

**MSCEqF:** The lift λ: M × U → 𝔤 converts inputs to Lie algebra elements, enabling propagation on the group.

**Code Reference:** [`source/msceqf/symmetry/symmetry.cpp:68-127`](source/msceqf/symmetry/symmetry.cpp) - `Symmetry::lift()`

**Paper Reference:** [2] Section IV-A (Lifted Dynamics), Equations (20)-(23)

### 5. State Updates via Group Operations

**Standard EKF:**

```cpp
x_new = x_old + K * innovation  // Vector addition
```

**MSCEqF:**

```cpp
X_new = exp(K * innovation) · X_old  // Group multiplication
```

The innovation is **exponentiated** to a group element and composed via group multiplication.

**Code Reference:** [`source/msceqf/filter/updater/updater.cpp:357-398`](source/msceqf/filter/updater/updater.cpp) - `UpdateMSCEqF()`

**Paper Reference:** [1] Section IV (Equivariant Filter), [2] Section IV-C (State Correction)

### 6. Curvature Correction

A unique feature of EqF that compensates for the non-flat geometry of the Lie group manifold.

**Standard EKF:** No equivalent (assumes flat Euclidean space).

**MSCEqF:** After each update, the covariance is corrected:

```
P_new = exp(-½Γ) · P · exp(-½Γ)ᵀ
```

where Γ is computed from the adjoint representation of the innovation.

**Code Reference:** [`source/msceqf/symmetry/symmetry.cpp:129-161`](source/msceqf/symmetry/symmetry.cpp) - `Symmetry::curvatureCorrection()`

**Paper Reference:** [1] Section IV-B (Curvature Correction), Theorem 2

### 7. Propagation on the Symmetry Group

**Standard EKF:**

```cpp
x_pred = f(x, u)                    // Nonlinear dynamics
P_pred = F * P * F' + Q             // Covariance propagation
```

**MSCEqF:**

```cpp
λ = lift(φ(X, ξ₀), u)              // Compute lift at current state
X_pred = X · exp(λ · dt)            // Propagate on group
P_pred = Φ * P * Φ' + B*Q*B'        // Covariance (with lifted Jacobians)
```

**Code Reference:** [`source/msceqf/filter/propagator/propagator.cpp:195-240`](source/msceqf/filter/propagator/propagator.cpp) - `propagateMean()`, `propagateCovariance()`

**Paper Reference:** [2] Section IV-A (State Propagation), Equations (24)-(26)

### 8. Fixed Origin Linearization

**Standard EKF:** Linearizes around the current state estimate (which changes every step).

**MSCEqF:** Linearizes around a **fixed origin** ξ₀ (set at initialization). The filter estimates the deviation from this origin as a group element.

This provides better consistency because the linearization point doesn't drift.

**Code Reference:** [`include/msceqf/state/state.hpp:49-53`](include/msceqf/state/state.hpp) - MSCEqFState constructor

**Paper Reference:** [1] Section III (Equivariant Systems), [2] Section II-B (Origin Selection)

---

## Code Architecture

### State Elements Hierarchy

```
MSCEqFStateElement (base class)
├── MSCEqFSDBState      (Dd) - Semi-Direct Bias group, 15 DOF
├── MSCEqFSE3State      (E)  - Camera extrinsics, 6 DOF
├── MSCEqFInState       (L)  - Camera intrinsics, 4 DOF
└── MSCEqFSOT3State     (Q)  - Feature states, 4 DOF each
```

### Key Files

| File | Purpose | Key Functions |
|------|---------|---------------|
| [`include/msceqf/state/state_elements.hpp`](include/msceqf/state/state_elements.hpp) | State element definitions | `updateLeft()`, `updateRight()` |
| [`include/msceqf/state/state.hpp`](include/msceqf/state/state.hpp) | MSCEqF state container | State access, cloning |
| [`include/msceqf/symmetry/symmetry.hpp`](include/msceqf/symmetry/symmetry.hpp) | Symmetry operations | `phi()`, `lift()`, `curvatureCorrection()` |
| [`source/msceqf/symmetry/symmetry.cpp`](source/msceqf/symmetry/symmetry.cpp) | Symmetry implementation | Group actions, lifts |
| [`source/msceqf/filter/propagator/propagator.cpp`](source/msceqf/filter/propagator/propagator.cpp) | State propagation | `propagateMean()`, `propagateCovariance()` |
| [`source/msceqf/filter/updater/updater.cpp`](source/msceqf/filter/updater/updater.cpp) | Measurement update | `UpdateMSCEqF()` |
| [`include/types/fptypes.hpp`](include/types/fptypes.hpp) | Type definitions | Lie group type aliases |

---

## Detailed Component Analysis

### Semi-Direct Bias State (SDB)

The SDB state combines the extended pose and biases into a single Lie group element.

**Location:** [`include/msceqf/state/state_elements.hpp:115-152`](include/msceqf/state/state_elements.hpp)

```cpp
struct MSCEqFSDBState final : public MSCEqFStateElement {
  MSCEqFSDBState(const uint& idx) : MSCEqFStateElement(idx, 15), Dd_(){};

  // Paper [2] Eq. (15): Right group multiplication for propagation
  void updateRight(const VectorX& delta) override {
    Dd_.multiplyRight(SDB::exp(delta));
  }

  // Paper [2] Eq. (28): Left group multiplication for update
  void updateLeft(const VectorX& delta) override {
    Dd_.multiplyLeft(SDB::exp(delta));
  }

  SDB Dd_;  // Semi-Direct Bias element: SE₂(3) ⋉ ℝ⁶
};
```

**Key insight:** The 15 DOF breaks down as:

- 3 DOF: Rotation (SO(3))
- 3 DOF: Velocity
- 3 DOF: Position
- 3 DOF: Gyroscope bias
- 3 DOF: Accelerometer bias

### Group Action Implementation

**Location:** [`source/msceqf/symmetry/symmetry.cpp:24-66`](source/msceqf/symmetry/symmetry.cpp)

The group action φ(X, ξ) transforms each component of the state:

```cpp
// Extended pose: T_new = T_old · D (right multiplication by SE₂(3) component)
// Paper [2] Eq. (12)
std::static_pointer_cast<ExtendedPoseState>(ptr)->T_.multiplyRight(X.D());

// Bias: b_new = Ad_{B}^{-1} · (b_old - δ)
// Paper [2] Eq. (13) - Key: bias transforms via adjoint inverse
std::static_pointer_cast<BiasState>(ptr)->b_ = X.B().invAdjoint() * (xi.b() - X.delta());

// Extrinsics: S_new = C^{-1} · S_old · E
// Paper [2] Eq. (14)
std::static_pointer_cast<CameraExtrinsicState>(ptr)->S_.multiplyLeft(X.C().inv());
std::static_pointer_cast<CameraExtrinsicState>(ptr)->S_.multiplyRight(X.E());
```

### Lift Function Implementation

**Location:** [`source/msceqf/symmetry/symmetry.cpp:68-127`](source/msceqf/symmetry/symmetry.cpp)

The lift converts IMU measurements to Lie algebra elements:

```cpp
// Paper [2] Eq. (20)-(22): Lift for extended pose
lambda_T.block<3, 1>(0, 0) = u.ang_ - xi.b().block<3, 1>(0, 0);      // ω - b_ω
lambda_T.block<3, 1>(3, 0) = u.acc_ - xi.b().block<3, 1>(3, 0) + ... // a - b_a + R'g
lambda_T.block<3, 1>(6, 0) = xi.T().R().transpose() * xi.T().v();     // R'v

// Paper [2] Eq. (23): Lift for bias (via adjoint)
lambda[key] = SE3::adjoint(xi.b()) * lambda_T.block<6, 1>(0, 0);
```

### Propagation Implementation

**Location:** [`source/msceqf/filter/propagator/propagator.cpp:195-240`](source/msceqf/filter/propagator/propagator.cpp)

```cpp
void Propagator::propagateMean(MSCEqFState& X, const SystemState& xi0, const Imu& u, const fp& dt) {
  // Paper [2] Eq. (24): Compute lift at current state estimate
  SystemState::SystemStateAlgebraMap lambda = Symmetry::lift(Symmetry::phi(X, xi0), u);

  // Paper [2] Eq. (25): Propagate via right multiplication
  // X_new = X_old · exp(λ · dt)
  X.state_.at(MSCEqFStateElementName::Dd)
      ->updateRight(dt * (Vector15() << lambda.at(SystemStateElementName::T),
                                         lambda.at(SystemStateElementName::b)).finished());
}
```

### Update Implementation

**Location:** [`source/msceqf/filter/updater/updater.cpp:357-398`](source/msceqf/filter/updater/updater.cpp)

```cpp
void Updater::UpdateMSCEqF(MSCEqFState& X, const MatrixX& C, const VectorX& delta, const MatrixX& R) const {
  // Standard Kalman gain computation
  MatrixX K = G * invS.selfadjointView<Eigen::Upper>();
  VectorX inn = K * delta;

  // Paper [2] Eq. (28): Update via LEFT multiplication (not vector addition!)
  // X_new = exp(innovation) · X_old
  X.state_.at(MSCEqFStateElementName::Dd)->updateLeft(inn.segment(...));
  X.state_.at(MSCEqFStateElementName::E)->updateLeft(inn.segment(...));

  // Standard covariance update
  X.cov_.triangularView<Eigen::Upper>() -= K * G.transpose();

  // Paper [1] Theorem 2: Curvature correction (unique to EqF!)
  if (opts_.curvature_correction_) {
    MatrixX expGamma = Symmetry::curvatureCorrection(X, inn);
    X.cov_ = expGamma * X.cov_ * expGamma.transpose();
  }
}
```

### Curvature Correction Implementation

**Location:** [`source/msceqf/symmetry/symmetry.cpp:129-161`](source/msceqf/symmetry/symmetry.cpp)

```cpp
const MatrixX Symmetry::curvatureCorrection(const MSCEqFState& X, const VectorX& inn) {
  MatrixX Gamma = MatrixX::Zero(inn.rows(), inn.rows());

  // Paper [1] Eq. (32): Γ constructed from adjoint representations
  // For SE₂(3) component
  Gamma.block(...) = SE23::adjoint(inn.segment(...));

  // For bias component
  Gamma.block(...) = SE3::adjoint(inn.segment(...));

  // For extrinsics
  Gamma.block(...) = SE3::adjoint(inn.segment(...));

  Gamma *= -0.5;

  // Paper [1] Eq. (33): Return matrix exponential
  return Gamma.exp();
}
```

---

## What's Identical vs. What's Different: The Linear Algebra Perspective

A common question when studying equivariant filters is: "Is the linear algebra of the Kalman filter the same, or completely different?"

**Short answer:** The core linear algebra machinery is **identical**. The EqF essentially "wraps" standard Kalman filter equations with Lie group operations.

### What's IDENTICAL (Standard Kalman Linear Algebra)

#### Kalman Gain Computation

Both EKF and MSCEqF use the exact same formula:

```
K = P · Hᵀ · (H · P · Hᵀ + R)⁻¹
```

**Code Reference:** [`source/msceqf/filter/updater/updater.cpp:384-391`](source/msceqf/filter/updater/updater.cpp)

```cpp
// Lines 384-391: STANDARD Kalman gain computation
MatrixX G = X.subCovCols(cols_map_.keys()) * C.transpose();           // P * H'
MatrixX S = C * X.subCov(cols_map_.keys()) * C.transpose() + R;       // H * P * H' + R
MatrixX K = G * invS;                                                  // K = P * H' * S^{-1}
VectorX inn = K * delta;                                               // innovation
```

#### Covariance Propagation Structure

Both use:

```
P_pred = Φ · P · Φᵀ + Q_d
```

#### Covariance Update Structure

Both use:

```
P_new = P - K · S · Kᵀ   (or equivalent Joseph form)
```

**Code Reference:** [`source/msceqf/filter/updater/updater.cpp:415`](source/msceqf/filter/updater/updater.cpp)

```cpp
// Line 415: STANDARD covariance update
X.cov_ -= K * G.transpose();    // P = P - K * (P * H')' = P - K * H * P
```

### What's DIFFERENT

| Aspect | Standard EKF | MSCEqF |
|--------|-------------|--------|
| **Jacobians source** | Linearizing f(x,u), h(x) around **current state** | Linearizing lifted dynamics around **fixed origin** ξ₀ |
| **Mean propagation** | `x = f(x, u)` | `X = X · exp(λ·dt)` (right group multiplication) |
| **Mean update** | `x = x + K·δ` (vector addition) | `X = exp(K·δ) · X` (left group multiplication) |
| **Curvature correction** | None | `P = exp(-½Γ) · P · exp(-½Γ)ᵀ` (optional) |

**Code Reference:** [`source/msceqf/filter/updater/updater.cpp:399-422`](source/msceqf/filter/updater/updater.cpp)

```cpp
// Lines 399-412: MEAN UPDATE via LEFT group multiplication (NOT vector addition!)
X.state_.at(MSCEqFStateElementName::Dd)->updateLeft(inn.segment(...));  // exp(inn) · X
X.state_.at(MSCEqFStateElementName::E)->updateLeft(inn.segment(...));   // NOT: X + inn

// Lines 418-422: CURVATURE CORRECTION (unique to EqF, optional)
if (opts_.curvature_correction_) {
  MatrixX expGamma = Symmetry::curvatureCorrection(X, inn);
  X.cov_ = expGamma * X.cov_ * expGamma.transpose();  // Extra geometric correction!
}
```

### Visual Summary

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        STANDARD KALMAN FILTER                                │
│                        (Identical in both EKF and EqF)                       │
├─────────────────────────────────────────────────────────────────────────────┤
│  Kalman Gain:        K = P · Hᵀ · (H·P·Hᵀ + R)⁻¹                            │
│  Cov Propagation:    P = Φ · P · Φᵀ + Q                                      │
│  Cov Update:         P = P - K · H · P                                       │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    │                               │
                    ▼                               ▼
    ┌───────────────────────────┐   ┌───────────────────────────────────────┐
    │      STANDARD EKF         │   │              MSCEqF                   │
    ├───────────────────────────┤   ├───────────────────────────────────────┤
    │ Jacobians: ∂f/∂x, ∂h/∂x   │   │ Jacobians: from lifted dynamics       │
    │ at current estimate       │   │ around fixed origin ξ₀                │
    ├───────────────────────────┤   ├───────────────────────────────────────┤
    │ Mean update:              │   │ Mean update:                          │
    │   x = x + K·δ             │   │   X = exp(K·δ) · X                    │
    │   (vector addition)       │   │   (group multiplication)              │
    ├───────────────────────────┤   ├───────────────────────────────────────┤
    │ Extra corrections: None   │   │ Curvature correction:                 │
    │                           │   │   P = exp(-½Γ)·P·exp(-½Γ)ᵀ            │
    └───────────────────────────┘   └───────────────────────────────────────┘
```

### Key Insight

The equivariant filter is **not** a completely different algorithm from the Kalman filter. Rather, it:

1. **Preserves** the core linear algebra (gain computation, covariance updates)
2. **Replaces** vector addition with Lie group exponential + multiplication for mean updates
3. **Computes** Jacobians differently (from lifted dynamics, around fixed origin)
4. **Adds** an optional curvature correction to account for manifold geometry

This means existing intuition about Kalman filters largely transfers to equivariant filters—the key differences are in how the state manifold is handled geometrically.

---

## Summary: Why Equivariant?

The equivariant filter provides several advantages:

1. **Consistency:** By respecting the geometric structure, the filter maintains better consistency over long trajectories.

2. **Bias Handling:** The semi-direct bias construction ensures that bias estimates remain compatible with the pose estimates.

3. **Fixed Linearization:** Linearizing around a fixed origin prevents the accumulation of linearization errors.

4. **Curvature Awareness:** The curvature correction accounts for the non-Euclidean geometry of the state space.

5. **Convergence:** The symmetry-preserving structure leads to improved convergence properties.

---

## References

1. van Goor, Pieter, Tarek Hamel, and Robert Mahony. "Equivariant filter (EqF)." *IEEE Transactions on Automatic Control* (2022).

2. Fornasier, Alessandro, et al. "Equivariant filter design for inertial navigation systems with input measurement biases." *2022 International Conference on Robotics and Automation (ICRA)*. IEEE, 2022.

3. Fornasier, Alessandro, et al. "Overcoming Bias: Equivariant Filter Design for Biased Attitude Estimation with Online Calibration." *IEEE Robotics and Automation Letters* 7.4 (2022): 12118-12125.

4. Fornasier, Alessandro, et al. "Equivariant Symmetries for Inertial Navigation Systems." *ArXiv preprint arXiv:2309.03765*.

5. Fornasier, Alessandro, et al. "MSCEqF: A Multi State Constraint Equivariant Filter for Vision-aided Inertial Navigation." *ArXiv preprint arXiv:2311.11649* (2023).
