# Adaptive Sliding Mode Control of 2 DOF Robotic Arm

## 2DOF Robot Arm Model on Simulink

$$
\ddot{x} = H^{-1} (\tau - C \dot{x} - F_c sgn(\dot{x}) - V \dot{x})
$$

Where:

$$ 
\tau = \begin{bmatrix}
\tau_1 & \tau_2
\end{bmatrix}^T
$$

$$ 
x = \begin{bmatrix}
x_1 & x
\end{bmatrix}^T
$$

$$
\ddot{x} = \frac{d}{dx}\dot{x} = \frac{d^2}{dx}x
$$

$$
H = \begin{bmatrix}
M_{11} & M_{12} \\
M_{21} & M_{22}
\end{bmatrix}
$$

where:

$$ M_{11} = a_1 + 2 a_3 cos(x_2) - 2 a_4 sin(x_2) $$
$$ M_{12} = M_{21} = a_2 + a_3 cos(x_2) + a_4 cos(x_2) $$
$$ M_{22} = a_2 $$

$$
C = \begin{bmatrix}
-c\dot{x}_2 & -c(\dot{x}_1 + \dot{x_2}) \\
c\dot{x}_1 & 0
\end{bmatrix}
$$

where:

$$ c = a_3 sin(x_2) - a_4 cos(x_2) $$

Estimated parameters:

$$
a = \begin{bmatrix}
a_1 \\
a_2 \\
a_3 \\
a_4
\end{bmatrix} = \begin{bmatrix}
3.3 \\
0.97 \\
1.4 \\
0.6
\end{bmatrix}
$$

Friction terms:

$$
F_c = \begin{bmatrix}
F_{c1} & 0 \\
0 & F_{c2}
\end{bmatrix}
$$

where:

$$ F_{c1} = F_{c2} = 5$$

and

$$
V = \begin{bmatrix}
V_{1} & 0 \\
0 & V_{2}
\end{bmatrix}
$$

where:

$$ V_1 = 5.5 $$ 
  and 
$$ V_2 = 2.7 $$

Given initial conditions:

```math
\begin{aligned}
x_1 = 0.3 \space rad \\ 
x_2 = 0.5 \space rad
\end{aligned}
```
![alt text](assets/2DOF_Arm_Simulink_Model.png)

## System Signals

$$
Y = \begin{bmatrix}
y_{11} & y_{12} & y_{13} & y_{14} \\
y_{21} & y_{22} & y_{23} & y_{24}
\end{bmatrix}
$$

where:

```math
\begin{aligned}
y_{11} &= \ddot{x}_{r1} \\
y_{12} &= \ddot{x}_{r2} \\
y_{21} &= 0 \\
y_{22} &= \ddot{x}_{r1} + \ddot{x}_{r2} \\
y_{13} &= (2\ddot{x}_{r1} + \ddot{x}_{r2}) \cos(x_{2}) -  (\dot{x}_{2} \dot{x}_{r1} + \dot{x}_{1} \dot{x}_{r2} + \dot{x}_{2} \dot{x}_{r2}) \sin(x_{2}) \\
y_{14} &= (2\ddot{x}_{r1} + \ddot{x}_{r2}) \sin(x_{2}) -  (\dot{x}_{2} \dot{x}_{r1} + \dot{x}_{1} \dot{x}_{r2} + \dot{x}_{2} \dot{x}_{r2}) \cos(x_{2}) \\
y_{23} &= \ddot{x}_{r1} \cos(x_2) + \dot{x}_1 \dot{x}_{r1} \sin(x_2) \\
y_{24} &= \ddot{x}_{r1} \sin(x_2) + \dot{x}_1 \dot{x}_{r1} \cos(x_2)
\end{aligned}
```

## Control Law

Estimated values of unmodeled dynamics:

$$ b^{-1} = 5 H $$
$$ f = 5 (\ddot{x}_d - 2 \lambda \dot{e} - \lambda^{2} e) $$

Adaptive sliding surface definition

$$
s = \dot{e} + 2 \lambda e + \lambda^{2} \int^{t}_{0}{e}
$$

Given control law:

$$ u = u_a + u_{eq} $$

such that:

$$ u_a = \hat{b}^{-1} (Y \hat{a} - k_D s) $$
$$  u_{eq} =  \hat{b}^{-1} (\ddot{x}_d - \hat{f} - 2 \lambda \dot{e} - \lambda^{2} e) $$

The update of estimated parameters:

$$ \dot{\hat{a}} = - \Gamma Y^{T} s $$

therefore:

$$ a = \int{\dot{\hat{a}}}\space dt $$

### Numerical Considerations

#### Integrators

Used limited output integrators in the arm model to be between $$ -pi $$ and $$ +pi $$.

#### Safe Matrix Inverse

Used a small regularization term:

```matlab
epsilon = 1e-6; % Small regularization term
H_reg = H + epsilon * eye(size(H));
H_inv = inv(H_reg);
```

In addition to using pseudoinverse if matrix is close to singular or badly scaled.

#### Handling `NaN` and `inf` Values

Implemented an algorithm to detect `NaN` and `inf` values, by using these replacements:

```matlab
nan_replacement = eps; % A very small positive number
inf_replacement = realmax; % The largest positive floating-point number
```
