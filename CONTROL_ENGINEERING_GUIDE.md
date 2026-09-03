# SINDY-MPC: from data to constrained control

## 1. SINDYc in one equation

SINDYc assumes a controlled nonlinear system

$$
\dot x=f(x,u).
$$

Build a library of candidate functions

$$
\Theta(X,U)=[1,x_1,\ldots,x_n,u_1,\ldots,x_n^2,x_1x_2,\ldots].
$$

Then identify a sparse coefficient matrix $\Xi$ such that

$$
\dot X\approx\Theta(X,U)\Xi.
$$

Each nonzero entry of $\Xi$ says that a particular candidate function participates in one state derivative. Sparsity is the key: instead of a black-box neural network, the model can often be written as a small interpretable differential equation.

## 2. Reading `EX_F8_SI_SINDYc.m`

`Nvar=3` means the identified state contains angle of attack, pitch angle and pitch rate. `polyorder=3` asks the library generator to include polynomial terms up to degree three. `usesine=0` disables trigonometric library functions. `lambda_vec` controls sparse thresholding; larger regularization generally produces fewer active terms but can underfit.

`getTrainingData` generates the excitation and collects state/input trajectories. This is system identification: the quality of the learned model is limited by excitation, sensor noise, derivative estimation and coverage of the operating envelope.

`trainSINDYc` estimates $\Xi$. The `Xi-Xi0` calculation compares identified coefficients against known coefficients for this benchmark.

The `ode45` call integrates the learned continuous model. For a discrete implementation, the RK4 loop evaluates the learned vector field repeatedly within one sample. This distinction matters: an ODE model and a discrete-time MPC model are not interchangeable without careful discretization.

## 3. Reading `MPC_F8.m`

`dt`/`Ts` define the sample time. `Nvar=3` defines state dimension. `fmincon(...,'sqp',...)` solves a constrained nonlinear program using sequential quadratic programming. `Duration` sets the closed-loop simulation duration.

At each sample, `xref` creates a future reference trajectory. `ObjectiveFCN` computes tracking and control penalties; `ConstraintFCN` encodes bounds. `fmincon` returns the optimized control sequence, but only `uopt(1)` is applied. The true F8 dynamics are then integrated with RK4 at ten substeps per sample. This is receding-horizon control.

The most important experimental design choice is the model used inside the optimizer versus the plant used for evaluation. A fair test should identify the model from training data and then evaluate it on held-out trajectories against the true plant.

## 4. SINDYc versus NARX versus DMDc

DMDc gives a linear controlled model, approximately

$$
x_{k+1}=Ax_k+Bu_k.
$$

NARX learns a nonlinear input/output relationship, usually with a larger black-box parameterization. SINDYc chooses from an explicit candidate-function library and promotes sparsity.

SINDYc is attractive when physics suggests a compact functional form and data are limited. It becomes risky when the candidate library is poorly chosen, derivatives are noisy, the operating envelope is too broad, or unmodeled dynamics dominate.

## 5. Industrial interpretation

For an industrial process, the useful pipeline is

$$
\text{historian/sensors}\rightarrow\text{clean + excite}\rightarrow\text{identify model}\rightarrow\text{validate}\rightarrow\text{MPC}\rightarrow\text{constraint monitor}\rightarrow\text{plant}.
$$

A production implementation needs uncertainty handling, drift detection, safe fallback, data-quality checks and a hard distinction between an identified research model and a validated safety-critical plant model.

## 6. MPC mathematics

For a discrete model

$$
x_{k+1}=f(x_k,u_k),
$$

NMPC solves

$$
\min_{x_{0:N},u_{0:N-1}}\sum_{k=0}^{N-1}\ell(x_k,u_k)+V_f(x_N)
$$

subject to the dynamics and constraints. $\ell$ is the stage cost and $V_f$ is the terminal cost. SINDYc supplies $f$; MPC supplies the constrained optimization layer.

## 7. Concrete use cases

- **Aircraft:** identify aerodynamic dynamics from flight-test/simulation data, then constrain angle-of-attack, actuator deflection and rate while tracking pitch commands.
- **Chemical processes:** identify a compact nonlinear model from historian data and use NMPC to manage temperature/composition constraints.
- **Robotics:** identify dynamics from logged trajectories and use MPC for constrained trajectory tracking.
- **Energy systems:** identify nonlinear thermal/storage dynamics and optimize power while respecting state-of-charge and actuator constraints.

For regulated aerospace/medical/nuclear applications, treat SINDYc as a model-development tool unless the full learned-model validation and software assurance process supports operational use.

## 8. Validation checklist

Use train/validation/test splits in time, not random point shuffles. Test extrapolation separately. Report one-step error, multi-step rollout error, closed-loop tracking error, constraint violations and solver latency. Add disturbances and sensor noise. Compare against a simple linear baseline and a nonlinear black-box baseline.
