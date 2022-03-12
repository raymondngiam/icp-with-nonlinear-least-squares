# Iterative Closest Point with Nonlinear Least Squares

---

## Overview

This project implements iterative closest point (ICP) algorithm with nonlinear least squares optimization on SE3 manifold using the `ceres-solver`, and `Sophus` library.

## Dataset

3d model data used, `Stanford Bunny` was obtained from <a href='http://graphics.stanford.edu/data/3Dscanrep/'>Stanford University Computer Graphics Laboratory</a>.

## Dependencies

1. cmake 3.19
1. Boost 1.78.0
1. fmt 8.1.1
1. Eigen 3.4.0
1. Sophus (commit:e10eb6e00c)
1. ceres-solver 2.0.0

## How to build

```bash
$ git clone https://github.com/raymondngiam/ICP-bunny.git
$ cd ICP-bunny
$ mkdir build && cd build
$ cmake ..
$ make
```

## How to run

1. To run with a line search type solver, `L-BFGS`:

  ```
  $ ./main 1
  solverType: LBFGS
  SE3 in HTM:
        0.5 -0.866025         0  0.236603
  0.866025       0.5         0  0.209808
          0         0         1         0
          0         0         0         1

  angle_groundtruth: 1.0472
  ax_groundtruth: 0
  ay_groundtruth: 0
  az_groundtruth: 1
  tx_groundtruth: 0.236603
  ty_groundtruth: 0.209808
  tz_groundtruth: 0

  Line count: 35947
  35947 points loaded:
    0: f: 9.129942e+02 d: 0.00e+00 g: 1.95e+01 h: 0.00e+00 s: 0.00e+00 e:  0 it: 9.08e-01 tt: 9.12e-01
    1: f: 6.281301e+01 d: 8.50e+02 g: 1.72e+01 h: 2.16e-01 s: 2.76e-05 e:  4 it: 3.67e+00 tt: 4.58e+00
    2: f: 6.225342e+01 d: 5.60e-01 g: 2.21e+01 h: 4.97e-03 s: 4.55e-05 e:  3 it: 2.75e+00 tt: 7.33e+00
    3: f: 8.681656e-01 d: 6.14e+01 g: 2.83e+01 h: 5.45e-01 s: 5.53e-03 e:  3 it: 2.78e+00 tt: 1.01e+01
    4: f: 8.644332e-01 d: 3.73e-03 g: 3.83e+01 h: 4.29e-04 s: 2.99e-05 e:  2 it: 1.86e+00 tt: 1.20e+01
    5: f: 7.937967e-01 d: 7.06e-02 g: 3.25e+01 h: 5.43e-03 s: 1.00e-03 e:  2 it: 1.86e+00 tt: 1.38e+01
    6: f: 7.334784e-01 d: 6.03e-02 g: 4.32e+01 h: 2.45e-02 s: 1.74e-02 e:  1 it: 9.27e-01 tt: 1.48e+01
    7: f: 2.833926e-02 d: 7.05e-01 g: 1.04e+01 h: 5.18e-02 s: 5.55e-01 e:  2 it: 1.86e+00 tt: 1.66e+01
    8: f: 9.642741e-05 d: 2.82e-02 g: 5.17e-01 h: 1.27e-02 s: 1.00e+00 e:  1 it: 9.26e-01 tt: 1.76e+01
    9: f: 8.364150e-07 d: 9.56e-05 g: 4.91e-02 h: 7.35e-04 s: 1.00e+00 e:  1 it: 9.27e-01 tt: 1.85e+01
    10: f: 5.569570e-10 d: 8.36e-07 g: 1.77e-03 h: 6.24e-05 s: 1.00e+00 e:  1 it: 9.30e-01 tt: 1.94e+01
    11: f: 8.765438e-13 d: 5.56e-10 g: 1.40e-04 h: 1.98e-06 s: 1.00e+00 e:  1 it: 9.27e-01 tt: 2.03e+01

  Solver Summary (v 2.0.0-eigen-(3.4.0)-lapack-suitesparse-(5.7.1)-cxsparse-(3.2.0)-eigensparse-no_openmp)

                                      Original                  Reduced
  Parameter blocks                            1                        1
  Parameters                                  7                        7
  Effective parameters                        6                        6
  Residual blocks                         35947                    35947
  Residuals                              107841                   107841

  Minimizer                         LINE_SEARCH
  Line search direction              LBFGS (20)
  Line search type                  CUBIC WOLFE

                                          Given                     Used
  Threads                                     1                        1

  Cost:
  Initial                          9.129942e+02
  Final                            7.074045e-18
  Change                           9.129942e+02

  Minimizer iterations                       12
  Line search steps                          22

  Time (in seconds):
  Preprocessor                         0.003599

    Residual only evaluation           0.000000 (0)
      Line search cost evaluation      0.000000
    Jacobian & residual evaluation    21.265959 (23)
      Line search gradient evaluation   20.357859
    Line search polynomial minimization  0.000063
  Minimizer                           21.267511

  Postprocessor                        0.001298
  Total                               21.272409

  Termination:                      CONVERGENCE (Parameter tolerance reached. Relative step_norm: 6.979446e-08 <= 1.000000e-06.)


          Initial         Optimized       GroundTruth
  angle   0.0000          1.0472          1.0472
  axis.x  0.0000          -0.0000         0.0000
  axis.y  0.0000          0.0000          0.0000
  axis.z  0.0000          1.0000          1.0000
  tx      0.0000          0.2366          0.2366
  ty      0.0000          0.2098          0.2098
  tz      0.0000          -0.0000         0.0000

  ```

  Visualization of the optimization progress is as shown below (the visualization was generated in `Unity` using the optimized pose logged in each iteration).

  <img src='img/lbfgs.gif'>


2. To run with trust region type solver, `Levenberg-Marquadt `:

  ```
  $ ./main 2
  solverType: LM
  SE3 in HTM:
        0.5 -0.866025         0  0.236603
  0.866025       0.5         0  0.209808
          0         0         1         0
          0         0         0         1

  angle_groundtruth: 1.0472
  ax_groundtruth: 0
  ay_groundtruth: 0
  az_groundtruth: 1
  tx_groundtruth: 0.236603
  ty_groundtruth: 0.209808
  tz_groundtruth: 0

  Line count: 35947
  35947 points loaded:
  iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
    0  9.129942e+02    0.00e+00    1.95e+01   0.00e+00   0.00e+00  1.00e+04        0    1.77e+00    1.79e+00
    1  1.535524e+02    7.59e+02    2.34e+00   5.59e-01   8.46e-01  1.49e+04        1    1.15e+00    2.94e+00
    2  1.346057e+00    1.52e+02    6.66e+00   1.06e-01   9.91e-01  4.48e+04        1    1.11e+00    4.05e+00
    3  1.092951e-06    1.35e+00    2.41e-01   9.01e-03   1.00e+00  1.35e+05        1    1.08e+00    5.13e+00
    4  2.719865e-15    1.09e-06    2.01e-06   1.02e-05   1.00e+00  4.04e+05        1    1.11e+00    6.23e+00

  Solver Summary (v 2.0.0-eigen-(3.4.0)-lapack-suitesparse-(5.7.1)-cxsparse-(3.2.0)-eigensparse-no_openmp)

                                      Original                  Reduced
  Parameter blocks                            1                        1
  Parameters                                  7                        7
  Effective parameters                        6                        6
  Residual blocks                         35947                    35947
  Residuals                              107841                   107841

  Minimizer                        TRUST_REGION

  Sparse linear algebra library    SUITE_SPARSE
  Trust region strategy     LEVENBERG_MARQUARDT

                                          Given                     Used
  Linear solver          SPARSE_NORMAL_CHOLESKY   SPARSE_NORMAL_CHOLESKY
  Threads                                     1                        1
  Linear solver ordering              AUTOMATIC                        1

  Cost:
  Initial                          9.129942e+02
  Final                            2.719865e-15
  Change                           9.129942e+02

  Minimizer iterations                        5
  Successful steps                            5
  Unsuccessful steps                          0

  Time (in seconds):
  Preprocessor                         0.022519

    Residual only evaluation           0.410349 (5)
    Jacobian & residual evaluation     5.839978 (5)
    Linear solver                      0.024876 (5)
  Minimizer                            6.291506

  Postprocessor                        0.001212
  Total                                6.315238

  Termination:                      CONVERGENCE (Parameter tolerance reached. Relative step_norm: 3.653964e-09 <= 1.000000e-06.)


          Initial         Optimized       GroundTruth
  angle   0.0000          1.0472          1.0472
  axis.x  0.0000          -0.0000         0.0000
  axis.y  0.0000          0.0000          0.0000
  axis.z  0.0000          1.0000          1.0000
  tx      0.0000          0.2366          0.2366
  ty      0.0000          0.2098          0.2098
  tz      0.0000          -0.0000         0.0000

  ```

  Visualization of the optimization progress is as shown below (the visualization was generated in `Unity` using the optimized pose logged in each iteration).

  <img src='img/lm.gif'>

---

## Implementation Details

Full src: <a href='./main.cpp'>main.cpp</a>

### Sophus Representation of SE3 with Unit Quaternion and Vector

Rigid transformation of a point in homogeneous transformation matrix (HTM) take the following form,

$\quad \begin{bmatrix} \\ \mathbf{p'} \\ \\1\end{bmatrix}
= \begin{bmatrix}\\ \mathbf{R} & \mathbf{t}\\ \\
\begin{matrix}0 & 0 & 0\end{matrix} & 1
\end{bmatrix}
\begin{bmatrix}p_x \\
p_y \\
p_z \\
1
\end{bmatrix}$

where $\mathbf{R} \in \mathbb{R}^{3\times 3}$ and $\mathbf{t} \in \mathbb{R}^{3}$

Equivalently, a `Sophus::SE3<T>` object represents rigid transformation of a point with a quaternion multiplication, followed by a vector addition.

$\quad \mathbf{p' = qpq^* + t}$

where $\mathbf{q}$ is an unit quaternion corresponding to $\mathbf{R}$, and $\mathbf{q}^*$ is a quaternion conjugate.

Thus, a `Sophus::SE3<T>` object has 7 parameters, namely $\{q_x,q_y,q_z,q_w,t_x,t_y,t_z\}$.

### Ceres Residual Definition with Sophus SE3 Object

```cpp{.line-numbers}
struct CostFunctor {
    CostFunctor(const Vector3d& target,
                const Vector3d& input):
        input(input), target(target){}

    template <typename T>
    bool operator()(T const * const se3_raw, T* residual_raw) const {
        Eigen::Map<Sophus::SE3<T> const> const se3(se3_raw);
        Eigen::Map<Eigen::Vector<T,3>> residual(residual_raw);
        residual = target - se3*input;
        return true;
    }

    Vector3d input;
    Vector3d target;
};
```

In `Ceres`, when we define a class with a templated `operator()` that computes the cost function in terms of the template parameter `T`, we get an **auto differentiated** cost function.

The autodiff framework substitutes appropriate `Jet` objects for `T` in order to compute the derivative when necessary.

Since the inner representation of `Sophus::SE3<T>` type is with a unit quaternion (4 parameters) and a $\mathbb{R}^3$ vector (3 parameters), it has 7 parameters. The derivative or Jacobian obtained takes the following form:

$\mathbf{J_i} = \begin{bmatrix}\frac{\partial L_i}{\partial q_x} & \frac{\partial L_i}{\partial q_y} & \frac{\partial L_i}{\partial q_z} & \frac{\partial L_i}{\partial q_w} & \frac{\partial L_i}{\partial t_x} & \frac{\partial L_i}{\partial t_y} & \frac{\partial L_i}{\partial t_z}\end{bmatrix}$

### Ceres Local Parameterization of SE3 Manifold into Tangent Space

At each point on the SE3 manifold there is a linear space that is tangent to the manifold. There are two reasons tangent spaces are interesting:

1. They are Euclidean spaces, so the usual vector space operations apply there, which makes numerical operations easy.

1. Movement in the tangent space translate into movements along the manifold, vice versa. They are related by the `exponential` and `logarithmic` map of SE3.

Besides the mathematical niceness, modeling manifold valued quantities correctly and paying attention to their geometry has practical benefits too:

1. It naturally constrains the quantity to the manifold through out the optimization. Freeing the user from hacks like *quaternion normalization*.

1. It reduces the dimension of the optimization problem to its natural size. 

The `LocalParameterization` interface in `Ceres` allows the user to define and associate with parameter blocks the manifold that they belong to. It does so by defining the Plus ($\oplus$) operation and its Jacobian (i.e. derivative with respect to $\Delta$ at $\Delta = 0$).

**$\oplus$ Operation**

For SE3 manifolds, the tangent space is given by the twist coordinate,

$\quad \bm{\xi} = \begin{bmatrix}\bm{\rho}\\\bm{w}\end{bmatrix}$

In homogeneous transformation matrix form (HTM), small perturbation (or i.e. the $\oplus$ operation) in $\mathfrak{se}$3 is related SE3 space as follows:

$\exp ((\bm{\xi + \Delta \xi})^\wedge) = \exp ((\bm{\Delta \xi})^\wedge) \exp (\bm{\hat{\xi}}) \qquad$ left multiply

$\exp ((\bm{\xi + \Delta \xi})^\wedge) = \exp (\bm{\hat{\xi}})\exp ((\bm{\Delta \xi})^\wedge) \qquad$ right multiply

Let's consider the right multiply case, as this choice results in a derivative that is visually neat, i.e. partial derivative of $\mathbf{t}$ is only dependent on $\bm{\rho}$; and partial derivative of $\mathbf{q}$ is only dependent on $\bm{w}$.

**Jacobian**

$\exp ((\bm{\xi + \Delta \xi})^\wedge) = \exp (\bm{\hat{\xi}})\exp ((\bm{\Delta \xi})^\wedge) \qquad$ right multiply

$\qquad \qquad \qquad \qquad \approx \exp (\bm{\hat{\xi}})(\mathbf{I} + (\bm{\Delta \xi})^\wedge) \qquad$ linearized at $\bm{\Delta \xi} \approx 0$ taking only first order term in exponential series expansion

$\qquad \qquad \qquad \qquad \approx \mathbf{H}(\mathbf{I} + (\bm{\Delta \xi})^\wedge)$

where 

$\mathbf{H} = \begin{bmatrix} \exp (\bm{w}) & \mathbf{J}\bm{\rho}\\
\mathbf{0} & 1 
\end{bmatrix} = \begin{bmatrix} \mathbf{R} & \mathbf{t}\\
\mathbf{0} & 1 
\end{bmatrix}$

$\mathbf{J}$ is left Jacobian of SO(3).

Taking the derivative,

$\frac{\partial \exp ((\bm{\xi + \Delta \xi})^\wedge)}{\partial \bm{\Delta \xi}}\approx \frac{\partial}{\partial \bm{\Delta \xi}} \mathbf{H}(\bm{\Delta \xi})^\wedge$

$\qquad \qquad \qquad \approx \frac{\partial}{\partial \bm{\Delta \xi}} \mathbf{H}\begin{bmatrix}0&-\Delta w_z&\Delta w_y&\Delta \rho_x\\
\Delta w_z&0&-\Delta w_x&\Delta \rho_y\\
-\Delta w_y&\Delta w_x&0&\Delta \rho_z\\
0&0&0&0
\end{bmatrix}$

$\qquad \qquad \qquad \approx \frac{\partial}{\partial \bm{\Delta \xi}} \begin{bmatrix}q_w^2 + q_x^2 - q_y^2 - q_z^2 & -2q_wq_z + 2q_xq_y & 2q_wq_y + 2q_xq_z & t_x\\
2q_wq_z + 2q_xq_y & q_w^2 + q_y^2 - q_x^2 - q_z^2 & -2q_wq_x + 2q_yq_z & t_y\\
-2q_wq_y + 2q_xq_z & 2q_wq_x + 2q_yq_z & q_w^2 + q_z^2 - q_x^2 - q_y^2  & t_z\\
0 & 0 & 0 & 1
\end{bmatrix}\begin{bmatrix}0&-\Delta w_z&\Delta w_y&\Delta \rho_x\\
\Delta w_z&0&-\Delta w_x&\Delta \rho_y\\
-\Delta w_y&\Delta w_x&0&\Delta \rho_z\\
0&0&0&0
\end{bmatrix}$

$\frac{\partial t_x}{\partial \Delta \rho_x} = (q_w^2 + q_x^2 - q_y^2 - q_z^2) \qquad \qquad \frac{\partial t_x}{\partial \Delta w_x} = 0$

$\frac{\partial t_x}{\partial \Delta \rho_y} = (-2q_wq_z + 2q_xq_y) \qquad \qquad \frac{\partial t_x}{\partial \Delta w_y} = 0$

$\frac{\partial t_x}{\partial \Delta \rho_z} = (2q_wq_y + 2q_xq_z) \qquad \qquad \frac{\partial t_x}{\partial \Delta w_z} = 0$

$\frac{\partial t_y}{\partial \Delta \rho_x} = (2q_wq_z + 2q_xq_y) \qquad \qquad \frac{\partial t_y}{\partial \Delta w_x} = 0$

$\frac{\partial t_y}{\partial \Delta \rho_y} = (q_w^2 + q_y^2 - q_x^2 - q_z^2) \qquad \qquad \frac{\partial t_y}{\partial \Delta w_y} = 0$

$\frac{\partial t_y}{\partial \Delta \rho_z} = (-2q_wq_x + 2q_yq_z) \qquad \qquad \frac{\partial t_y}{\partial \Delta w_z} = 0$

$\frac{\partial t_z}{\partial \Delta \rho_x} = (-2q_wq_y + 2q_xq_z) \qquad \qquad \frac{\partial t_z}{\partial \Delta w_x} = 0$

$\frac{\partial t_z}{\partial \Delta \rho_y} = (2q_wq_x + 2q_yq_z) \qquad \qquad \frac{\partial t_z}{\partial \Delta w_y} = 0$

$\frac{\partial t_z}{\partial \Delta \rho_z} = (q_w^2 + q_z^2 - q_x^2 - q_y^2) \qquad \qquad \frac{\partial t_z}{\partial \Delta w_z} = 0$

If we were to represent the SO(3) transformation in quaternion form

$ \exp (\bm{\hat{w}}) \rightarrow \mathbf{q}$

where

$\mathbf{q} = (\cos (\frac{||\mathbf{w}||}{2}), \sin (\frac{||\mathbf{w}||}{2})\frac{\mathbf{w}}{||\mathbf{w}||})$

$\exp ((\bm{w + \Delta w})^\wedge) \rightarrow \mathbf{q} . \exp (\frac{1}{2}\bm{\Delta w}) \qquad$ right multiply

$\qquad \qquad \qquad \qquad \approx \mathbf{q} . (1 + \frac{1}{2}\bm{\Delta w}) \qquad$ linearized at $\bm{\Delta w} \approx 0$ taking only first order term in exponential series expansion

$\qquad \qquad \qquad \qquad \approx (q_w,\begin{bmatrix}q_x\\
q_y\\
q_z\end{bmatrix}) . (1,\begin{bmatrix}0.5\Delta w_x\\
0.5\Delta w_y\\
0.5\Delta w_z\end{bmatrix})$

$\frac{\partial \mathbf{q} . \exp (\frac{1}{2}\bm{\Delta w})}{\partial \bm{\Delta w}}\approx \frac{\partial}{\partial \bm{\Delta w}} (q_w,\begin{bmatrix}q_x\\
q_y\\
q_z\end{bmatrix}) . (1,\begin{bmatrix}0.5\Delta w_x\\
0.5\Delta w_y\\
0.5\Delta w_z\end{bmatrix})$

$\qquad \qquad \approx \frac{\partial}{\partial \bm{\Delta w}}(q_w-0.5q_x\Delta w_x-0.5q_y\Delta w_y-0.5q_z\Delta w_z, \begin{bmatrix} q_x + 0.5q_w\Delta w_x - 0.5q_z\Delta w_y + 0.5q_y\Delta w_z \\
q_y + 0.5q_z\Delta w_x + 0.5q_w\Delta w_y - 0.5q_x\Delta w_z\\
q_z - 0.5q_y\Delta w_x + 0.5q_x\Delta w_y + 0.5q_w\Delta w_z
\end{bmatrix})$

$\frac{\partial q_x}{\partial \Delta w_x} = 0.5 q_w \quad \frac{\partial q_x}{\partial \Delta w_y} = -0.5 q_z \quad \frac{\partial q_x}{\partial \Delta w_z} = 0.5 q_y \quad \frac{\partial q_x}{\partial \Delta \rho_x} = 0 \quad \frac{\partial q_x}{\partial \Delta \rho_y} = 0 \quad \frac{\partial q_x}{\partial \Delta \rho_z} = 0$

$\frac{\partial q_y}{\partial \Delta w_x} = 0.5 q_z \quad \frac{\partial q_y}{\partial \Delta w_y} = 0.5 q_w \quad \frac{\partial q_y}{\partial \Delta w_z} = -0.5 q_x \quad \frac{\partial q_y}{\partial \Delta \rho_x} = 0 \quad \frac{\partial q_y}{\partial \Delta \rho_y} = 0 \quad \frac{\partial q_y}{\partial \Delta \rho_z} = 0$

$\frac{\partial q_z}{\partial \Delta w_x} = -0.5 q_y \quad \frac{\partial q_z}{\partial \Delta w_y} = 0.5 q_x \quad \frac{\partial q_z}{\partial \Delta w_z} = 0.5 q_w \quad \frac{\partial q_z}{\partial \Delta \rho_x} = 0 \quad \frac{\partial q_z}{\partial \Delta \rho_y} = 0 \quad \frac{\partial q_z}{\partial \Delta \rho_z} = 0$

$\frac{\partial q_w}{\partial \Delta w_x} = -0.5 q_x \quad \frac{\partial q_w}{\partial \Delta w_y} = -0.5 q_y \quad \frac{\partial q_w}{\partial \Delta w_z} = -0.5 q_z \quad \frac{\partial q_w}{\partial \Delta \rho_x} = 0 \quad \frac{\partial q_z}{\partial \Delta \rho_y} = 0 \quad \frac{\partial q_z}{\partial \Delta \rho_z} = 0$

The Jacobian for this SE3 local parameterization has the size of $[7 \times 6]$. Taking the form:

$\begin{bmatrix}\frac{\partial q_x}{\partial \Delta \rho_x} & \frac{\partial q_x}{\partial \Delta \rho_y} & \frac{\partial q_x}{\partial \Delta \rho_z} & \frac{\partial q_x}{\partial \Delta w_x} & \frac{\partial q_x}{\partial \Delta w_y} & \frac{\partial q_x}{\partial \Delta w_z} \\
\frac{\partial q_y}{\partial \Delta \rho_x} & \frac{\partial q_y}{\partial \Delta \rho_y} & \frac{\partial q_y}{\partial \Delta \rho_z} & \frac{\partial q_y}{\partial \Delta w_x} & \frac{\partial q_y}{\partial \Delta w_y} & \frac{\partial q_y}{\partial \Delta w_z} \\
&&... \\
&&... \\
&&... \\
&&... \\
\frac{\partial t_z}{\partial \Delta \rho_x} & \frac{\partial t_z}{\partial \Delta \rho_y} & \frac{\partial t_z}{\partial \Delta \rho_z} & \frac{\partial t_z}{\partial \Delta w_x} & \frac{\partial t_z}{\partial \Delta w_y} & \frac{\partial t_z}{\partial \Delta w_z} \\
\end{bmatrix}$

The derivative of the residuals' cost w.r.t. the `Sophus::SE3<T>` type's parameters is $[n \times 7]$, obtained from the `Ceres` autodiff framework.

Multiplying the residual Jacobian to the local parameterization Jacobian ($[n \times 7] . [7 \times 6] = [n \times 6]$) would yield the derivative w.r.t. the tangent space parameters, i.e. the twist coordinates. These are then used in the update steps in the nonlinear least squares iterations.

The following code snippet shows how to implement the `ceres::LocalParameterization` class with parameter block that make use of `Sophus::SE3<T>`.

```cpp{.line-numbers}
class LocalParameterizationSE3 : public ceres::LocalParameterization {
    public:
        virtual ~LocalParameterizationSE3() {}

        // SE3 plus operation for Ceres
        //
        //  T * exp(x)
        //
        virtual bool Plus(double const* T_raw, double const* delta_raw,
                          double* T_plus_delta_raw) const {
            Eigen::Map<Sophus::SE3d const> const T(T_raw);
            Eigen::Map<Sophus::Vector6d const> const delta(delta_raw);
            Eigen::Map<SE3d> T_plus_delta(T_plus_delta_raw);
            T_plus_delta = T * SE3d::exp(delta);
            return true;
        }

        // Jacobian of SE3 plus operation for Ceres
        //
        // Dx T * exp(x)  with  x=0
        //
        virtual bool ComputeJacobian(double const* T_raw,
                                     double* jacobian_raw) const {
            Eigen::Map<Sophus::SE3d const> T(T_raw);
            Eigen::Map<Eigen::Matrix<double, 
                                     SE3d::num_parameters, 
                                     SE3d::DoF, 
                                     Eigen::RowMajor>>jacobian(jacobian_raw);
            jacobian = T.Dx_this_mul_exp_x_at_0();
            return true;
        }

        virtual int GlobalSize() const { return SE3d::num_parameters; }

        virtual int LocalSize() const { return SE3d::DoF; }
};
```

### Setting Up Ceres Problem

The following code snippet shows how to set up the `ceres::Problem` with the cost functor and local parameterization descibed above.

```cpp{.line-numbers}
double angle(0);
Eigen::Vector3d axis;
axis << 0,0,0;
Eigen::AngleAxisd aa(angle,axis.normalized());
Eigen::Quaterniond q(aa);

Eigen::Vector3d t;
t << 0,0,0;
Sophus::SE3d T(q,t);

ceres::Problem problem;
problem.AddParameterBlock(T.data(),
                            Sophus::SE3d::num_parameters,
                            new LocalParameterizationSE3);  
for(uint32_t i=0; i<N; i++){
    Vector3d target = targetPoints.col(i);
    Vector3d input = sourcePoints.col(i);
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, 
                                  3,                     // number of residual
                                  Sophus::SE3d::num_parameters   // number of parameter for first parameter block 
                                >(new CostFunctor(target, input));
    problem.AddResidualBlock(cost_function, NULL,T.data());
}
```

---

## References

1. ceres-solver http://ceres-solver.org/nnls_modeling.html

