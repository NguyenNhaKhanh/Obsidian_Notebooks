---
title: Appendix - Gaussian Distribution and Matrix Derivatives
source: input/slambook-en.pdf
tags:
  - slam
  - visual-slam
  - study-report
  - chapter-report
  - appendix
  - gaussian-distribution
  - matrix-derivatives
  - kalman-filter
  - jacobian
  - covariance
  - key-term/gaussian-distribution
  - key-term/covariance
  - key-term/conditional-distribution
  - key-term/jacobian
---

# Appendix - Gaussian Distribution and Matrix Derivatives

## Overview

The appendices collect mathematical tools used throughout the SLAM backend chapters: Gaussian distributions, Gaussian transformations, Gaussian products, conditional distributions, and matrix derivative conventions.

These topics support [[Chapter 8 - Filters and Optimization Approaches Part I]], [[Chapter 11 - Dense Reconstruction]], and the optimization derivations in [[Chapter 5 - Nonlinear Optimization]].

> [!summary]
> The appendices explain the probability and derivative notation that makes Kalman filters, depth filters, Jacobians, and least-squares linearization work.

## Gaussian Distribution

For a scalar random variable:

$$
x \sim \mathcal{N}(\mu,\sigma^2)
$$

the probability density is:

$$
p(x)
=
\frac{1}{\sqrt{2\pi}\sigma}
\exp\left(
-\frac{1}{2}
\frac{(x-\mu)^2}{\sigma^2}
\right)
$$

For a vector random variable:

$$
x \sim \mathcal{N}(\mu,\Sigma)
$$

the density is:

$$
p(x)
=
\frac{1}{\sqrt{(2\pi)^N\det(\Sigma)}}
\exp\left(
-\frac{1}{2}(x-\mu)^T\Sigma^{-1}(x-\mu)
\right)
$$

The covariance matrix $\Sigma$ describes uncertainty and correlation between state dimensions.

## Linear Transform of Gaussian Variables

If:

$$
x \sim \mathcal{N}(\mu_x,\Sigma_{xx})
$$

and:

$$
y=Ax
$$

then:

$$
y \sim \mathcal{N}(A\mu_x,A\Sigma_{xx}A^T)
$$

If two independent Gaussian variables are added:

$$
x+y
\sim
\mathcal{N}(\mu_x+\mu_y,\Sigma_{xx}+\Sigma_{yy})
$$

This is the basis for the prediction covariance in [[Chapter 8 - Filters and Optimization Approaches Part I#Kalman Filter]].

## Normalized Product of Gaussians

When fusing two Gaussian estimates:

$$
x \sim \mathcal{N}(\mu_x,\Sigma_{xx})
$$

$$
y \sim \mathcal{N}(\mu_y,\Sigma_{yy})
$$

the fused covariance and mean satisfy:

$$
\Sigma^{-1}
=
\Sigma_{xx}^{-1}
+
\Sigma_{yy}^{-1}
$$

$$
\Sigma^{-1}\mu
=
\Sigma_{xx}^{-1}\mu_x
+
\Sigma_{yy}^{-1}\mu_y
$$

> [!tip]
> In information form, Gaussian fusion is clean: information matrices add, and information-weighted means add.

This same idea appears in the Gaussian depth filter from [[Chapter 11 - Dense Reconstruction#Gaussian Depth Filter]].

## Joint and Conditional Gaussian Distribution

For a joint Gaussian:

$$
p(x,y)
=
\mathcal{N}
\left(
\begin{bmatrix}
\mu_x \\
\mu_y
\end{bmatrix},
\begin{bmatrix}
\Sigma_{xx} & \Sigma_{xy} \\
\Sigma_{yx} & \Sigma_{yy}
\end{bmatrix}
\right)
$$

the conditional distribution is:

$$
p(x \mid y)
=
\mathcal{N}
\left(
\mu_x+\Sigma_{xy}\Sigma_{yy}^{-1}(y-\mu_y),
\Sigma_{xx}-\Sigma_{xy}\Sigma_{yy}^{-1}\Sigma_{yx}
\right)
$$

This is closely related to marginalization and conditional priors in [[Chapter 9 - Filters and Optimization Approaches Part II#Marginalization and Fill-In]].

## Matrix Derivative Convention

For a scalar function $f(x)$ where $x \in \mathbb{R}^m$, the derivative can be written as a column vector:

$$
\frac{df}{dx}
=
\begin{bmatrix}
\frac{df}{dx_1} \\
\vdots \\
\frac{df}{dx_m}
\end{bmatrix}
$$

For a vector function:

$$
F(x)=
\begin{bmatrix}
f_1(x) \\
\vdots \\
f_n(x)
\end{bmatrix}
$$

the Jacobian is:

$$
\frac{\partial F}{\partial x^T}
=
\begin{bmatrix}
\frac{\partial f_1}{\partial x_1} & \cdots & \frac{\partial f_1}{\partial x_m} \\
\vdots & \ddots & \vdots \\
\frac{\partial f_n}{\partial x_1} & \cdots & \frac{\partial f_n}{\partial x_m}
\end{bmatrix}
\in \mathbb{R}^{n \times m}
$$

Thus:

$$
\frac{\partial Ax}{\partial x^T}=A
$$

The book sometimes omits the transpose in the denominator when the meaning is clear.

> [!warning]
> Matrix derivative notation varies across fields. Always check whether the author uses column-vector or row-vector derivative conventions.

## Key Terms

**Gaussian distribution:** Probability distribution described by mean and covariance.

**Covariance:** Matrix measuring uncertainty and correlation.

**Information matrix:** Inverse covariance matrix.

**Linear transform:** Operation that maps a Gaussian to another Gaussian.

**Normalized product:** Fusion of Gaussian estimates through multiplication and normalization.

**Conditional distribution:** Distribution of one variable given another.

**Jacobian:** Matrix of first derivatives for a vector function.

## Connections

Kalman prediction and update in [[Chapter 8 - Filters and Optimization Approaches Part I]] depend on Gaussian transformation and fusion.

Marginalization in [[Chapter 9 - Filters and Optimization Approaches Part II]] uses conditional Gaussian reasoning.

Depth filters in [[Chapter 11 - Dense Reconstruction]] use Gaussian product formulas to fuse repeated depth observations.

Jacobians are central to [[Chapter 5 - Nonlinear Optimization]], [[Chapter 6 - Visual Odometry Part I]], and [[Chapter 7 - Visual Odometry Part II]].

> [!summary]
> The appendices are the small mathematical toolkit behind the larger SLAM story: represent uncertainty with Gaussians, transform and fuse it correctly, and use Jacobians to linearize nonlinear models.

