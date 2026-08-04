---
title: Chapter 8 - Filters and Optimization Approaches Part I
source: input/slambook-en.pdf
tags:
  - slam
  - visual-slam
  - study-report
  - chapter-report
  - backend-optimization
  - state-estimation
  - kalman-filter
  - ekf
  - bundle-adjustment
  - graph-optimization
  - schur-complement
  - robust-kernel
  - sparse-matrix
  - key-term/kalman-gain
  - key-term/bundle-adjustment
  - key-term/schur-complement
  - key-term/huber-kernel
---

# Chapter 8 - Filters and Optimization Approaches Part I

## Overview

Chapter 8 begins the backend part of SLAM. The frontend chapters, [[Chapter 6 - Visual Odometry Part I]] and [[Chapter 7 - Visual Odometry Part II]], estimate short-term motion. The backend asks how to maintain a more consistent long-term trajectory and map under noise.

This chapter compares filtering and optimization, then studies bundle adjustment, sparsity, Schur elimination, and robust kernels.

> [!summary]
> The backend turns many noisy motion and observation constraints into a state-estimation problem. Modern visual SLAM usually favors sparse nonlinear optimization over full EKF-style filtering for large maps.

## State Estimation Perspective

The SLAM motion and observation model can be written:

| ![[attachments/slambook-ch6-13/chapter-8-uncertainty.png]] |
| :-------------------------------------------------------: |
| Motion increases uncertainty, while observations can reduce it |

$$
\begin{cases}
x_k=f(x_{k-1},u_k)+w_k \\
z_k=h(x_k)+v_k
\end{cases}
$$

The goal is to estimate:

$$
P(x_k \mid x_0,u_{1:k},z_{1:k})
$$

Using Bayes' rule:

$$
P(x_k \mid x_0,u_{1:k},z_{1:k})
\propto
P(z_k \mid x_k)
P(x_k \mid x_0,u_{1:k},z_{1:k-1})
$$

The first term is the likelihood from the observation model. The second term is the prior predicted from past information.

This connects directly to the MAP and MLE discussion in [[Chapter 5 - Nonlinear Optimization#Bayes, MAP, and MLE]].

## Kalman Filter

For a linear Gaussian system:

$$
\begin{cases}
x_k=A_kx_{k-1}+u_k+w_k \\
z_k=C_kx_k+v_k
\end{cases}
$$

with:

$$
w_k \sim \mathcal{N}(0,R), \qquad v_k \sim \mathcal{N}(0,Q)
$$

the Kalman filter has two stages.

Prediction:

$$
\check{x}_k=A_k\hat{x}_{k-1}+u_k
$$

$$
\check{P}_k=A_k\hat{P}_{k-1}A_k^T+R
$$

Update:

$$
K=\check{P}_kC_k^T(C_k\check{P}_kC_k^T+Q)^{-1}
$$

$$
\hat{x}_k=\check{x}_k+K(z_k-C_k\check{x}_k)
$$

$$
\hat{P}_k=(I-KC_k)\check{P}_k
$$

> [!info]
> The Kalman gain decides how much the estimator trusts the observation compared with the prediction.

## Extended Kalman Filter

Visual SLAM is nonlinear because camera projection and SE(3) motion are nonlinear. EKF handles this by linearizing the motion and observation functions.

Prediction:

$$
\check{x}_k=f(\hat{x}_{k-1},u_k)
$$

$$
\check{P}_k=F\hat{P}_{k-1}F^T+R_k
$$

where:

$$
F=
\frac{\partial f}{\partial x_{k-1}}
\bigg|_{\hat{x}_{k-1}}
$$

Update:

$$
K_k=\check{P}_kH^T(H\check{P}_kH^T+Q_k)^{-1}
$$

$$
\hat{x}_k=\check{x}_k+K_k(z_k-h(\check{x}_k))
$$

$$
\hat{P}_k=(I-K_kH)\check{P}_k
$$

with:

$$
H=
\frac{\partial h}{\partial x_k}
\bigg|_{\check{x}_k}
$$

> [!warning]
> EKF linearizes once per update. If the system is strongly nonlinear or the estimate is far from the true state, the approximation can become poor.

## Why Optimization Often Wins

The chapter gives several reasons modern visual SLAM often uses optimization:

- Filtering assumes limited dependence on history through a Markov model.
- EKF stores covariance over all states, which grows quadratically.
- EKF performs limited linearization, while iterative optimization relinearizes.
- Visual outliers are common, and optimization can use robust kernels.

This does not mean filtering is useless. Filters remain valuable for small state vectors, high-rate sensor fusion, and resource-constrained systems. But large visual maps strongly benefit from sparse optimization.

## Bundle Adjustment

Bundle adjustment optimizes camera poses and 3D landmarks together so projected landmarks match observed image points.

| ![[attachments/slambook-ch6-13/chapter-8-projection-model.png]] |
| :------------------------------------------------------------: |
| Projection pipeline used by BA residuals: transform, normalize, distort, and apply intrinsics |

| ![[attachments/slambook-ch6-13/chapter-8-ba-structure.png]] |
| :--------------------------------------------------------: |
| Bundle adjustment structure: camera poses and landmarks are jointly optimized |

For a 3D point $p_j$ observed by camera pose $T_i$, the residual is:

$$
e_{ij}=z_{ij}-h(T_i,p_j)
$$

The BA objective is:

$$
\min_{T_i,p_j}
\frac{1}{2}
\sum_i\sum_j
\left\|
z_{ij}-h(T_i,p_j)
\right\|^2
$$

This is the full form of the reprojection-error optimization introduced in [[Chapter 6 - Visual Odometry Part I#3D-2D Motion: PnP]].

## Sparse Structure

Each reprojection residual only touches one camera pose and one landmark. Therefore, the Jacobian has many zeros. This sparse structure is the key to making BA practical.

After linearization:

$$
H\Delta x=g
$$

Separate pose increments and landmark increments:

$$
\Delta x =
\begin{bmatrix}
\Delta x_c \\
\Delta x_p
\end{bmatrix}
$$

The Hessian block form is:

| ![[attachments/slambook-ch6-13/chapter-8-hessian-blocks.png]] |
| :----------------------------------------------------------: |
| Hessian block structure separates pose variables from landmark variables |

$$
\begin{bmatrix}
B & E \\
E^T & C
\end{bmatrix}
\begin{bmatrix}
\Delta x_c \\
\Delta x_p
\end{bmatrix}
=
\begin{bmatrix}
v \\
w
\end{bmatrix}
$$

## Schur Complement

Because the landmark block $C$ is block diagonal, landmarks can be eliminated first:

$$
(B-EC^{-1}E^T)\Delta x_c
=
v-EC^{-1}w
$$

The matrix:

$$
S=B-EC^{-1}E^T
$$

is the Schur complement. After solving for pose increments, landmark increments can be recovered:

$$
\Delta x_p=C^{-1}(w-E^T\Delta x_c)
$$

> [!tip]
> Schur elimination is powerful because SLAM usually has far more landmarks than poses.

## Robust Kernels

Outliers from mismatches, moving objects, or bad depth can damage least squares because squared error grows quickly. Robust kernels reduce the influence of large residuals.

Instead of:

$$
\frac{1}{2}e^Te
$$

use:

$$
\frac{1}{2}\rho(e^Te)
$$

A common choice is the Huber kernel, which behaves quadratically for small residuals and linearly for large residuals.

| ![[attachments/slambook-ch6-13/chapter-8-huber-kernel.png]] |
| :-------------------------------------------------------: |
| Huber kernel grows like L2 loss near zero and like L1 loss for large residuals |

> [!warning]
> Robust kernels do not magically fix bad data association. They reduce damage, but loop closure and frontend matching still need careful verification.

## Key Terms

**Backend:** SLAM module that estimates globally or locally consistent states from frontend constraints.

**Kalman filter:** Recursive estimator for linear Gaussian systems.

**EKF:** Extension of the Kalman filter using local linearization.

**Bundle adjustment:** Joint optimization of camera poses and 3D points by minimizing reprojection errors.

**Schur complement:** Algebraic method for eliminating landmark variables efficiently.

**Robust kernel:** Loss function that limits the influence of outlier residuals.

**Sparsity:** Structure where most matrix entries are zero, enabling efficient large-scale optimization.

## Connections

This chapter expands the nonlinear least-squares framework from [[Chapter 5 - Nonlinear Optimization]].

The projection model is from [[Chapter 4 - Cameras and Images]].

The reprojection residuals are the optimized version of PnP from [[Chapter 6 - Visual Odometry Part I]].

The Gaussian and Jacobian background is summarized in [[Appendix - Gaussian Distribution and Matrix Derivatives]].

The scale-control problem continues in [[Chapter 9 - Filters and Optimization Approaches Part II]], where sliding windows and pose graphs simplify backend optimization.

> [!summary]
> Chapter 8 explains why backend SLAM is not just "run least squares." The practical power comes from sparse graph structure, Schur elimination, and robust handling of bad measurements.
