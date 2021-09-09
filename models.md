# Bicycle Kinematic model

This is the model used in the solver.

## Continuous model
$$
\dot{\mathbf{z}} = A \mathbf{z}(t) + B \mathbf{u}(t)

\newline
\mathbf{z} = 
\begin{bmatrix}
x \\ y \\ \varphi \\ \delta \\ v_x \\ \theta
\end{bmatrix}
,
\mathbf{u} =
\begin{bmatrix}
\alpha^u_x \\ a^u_x \\ \Delta t
\end{bmatrix}
$$


$$
\dot{\mathbf{z}} =
\begin{bmatrix}
\dot{x} \\ \dot{y} \\ \dot{\varphi} \\ \dot{\delta} \\ \dot{v_x} \\ \dot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
v_x \cdot \cos{\varphi} \\
v_x \cdot \sin{\varphi} \\
\frac{v_x}{L} \cdot \tan{\delta} \\
\alpha^u_x \\
a^u_x \\
v_x \cdot \Delta t
\end{bmatrix} 
$$

Where:

$x,y$ : position coordinates

$\varphi$: x-y orientation

$\delta$ : Steering wheel velocity

$v_x$ : longitudinal speed

$\theta$ : progress along a given curve

And 

$\alpha^u_x$ : Steering wheel acceleration (commanded)

$a^u_x$ : Longitudinal acceleration (commanded)

$\Delta t$ : progress rate (Computed or commanded?)
 
$L$ : inter-axle distance (fixed parameter)
## Discrete Model
$$
\mathbf{z}_{k+1} = A\mathbf{z}_k + B \mathbf{u}_k


\newline
\mathbf{z}_k = 
\begin{bmatrix}
x \\ y \\ \varphi \\ \delta \\ v_x \\ \theta
\end{bmatrix}
,
\mathbf{u}_k =
\begin{bmatrix}
\alpha^u_x \\ a^u_x \\ \Delta t
\end{bmatrix}
$$

$$
\mathbf{z}_{t+ts} =
\begin{bmatrix}
x \\ y \\ \varphi \\ \delta \\ v_x \\ \theta
\end{bmatrix}_t
+
\begin{bmatrix}
v_x \cdot \cos{\varphi} \\
v_x \cdot \sin{\varphi} \\
\frac{v_x}{L} \cdot \tan{\delta} \\
\alpha^u_x \\
a^u_x \\
v_x \cdot \Delta t
\end{bmatrix}
ts
$$

Where $ts$ is the timestep size

---

## Compute step (utils.py)

$$
\mathbf{w}_{init} =
\begin{bmatrix}
x \\ y \\ \varphi \\ \delta \\ v_x \\ \theta \\ \alpha^u_x \\ a^u_x \\ \Delta t
\end{bmatrix}
$$

# Formulating the NLP
Initial (measured) state $= \mathbf{z}_0$

Evolved (optimized) state $= Z_k$

Number of timesteps $= n$

$w : [\ 6(n+1) + 3n\ ] \times 1$ (Decision variable vector)
$$
w = \begin{bmatrix}
Z_0 \\ U_0 \\ Z_1 \\ U_1 \\ Z_2 \\
\vdots \\
U_{n-1} \\ Z_n
\end{bmatrix} ,


w_{lb} = \begin{bmatrix}
\mathbf{z}_0 \\ U_{lb} \\ Z_{lb} \\ U_{lb} \\ Z_{lb} \\
\vdots \\ U_{lb} \\ Z_{lb}
\end{bmatrix} ,

w_{ub} = \begin{bmatrix}
\mathbf{z}_0 \\ U_{ub} \\ Z_{ub} \\ U_{ub} \\ Z_{ub} \\
\vdots \\  U_{ub} \\ Z_{ub}
\end{bmatrix} ,

w_0 = \begin{bmatrix}
\mathbf{z}_0 \\ U_{rand} \\ Z_{1,est+rand} \\ U_{rand} \\ Z_{2,est+rand} \\
\vdots \\
U_{rand} \\ Z_{n,est+rand}
\end{bmatrix}
$$

Propogate the evolved state through equality constraints $g: 6n \times 1$
$$
g = \begin{bmatrix}
Z_{0,end}-Z_1 \\ Z_{1,end}-Z_2 \\
\vdots \\
Z_{n-1,end}-Z_{n}
\end{bmatrix} ,

g_{lb} = \begin{bmatrix}
\vec{0} \\ \vec{0} \\
\vdots \\
\vec{0}
\end{bmatrix} ,

g_{ub} = \begin{bmatrix}
\vec{0} \\ \vec{0} \\
\vdots \\
\vec{0}
\end{bmatrix}
$$

Where
$$
Z_{k,end}, J_k= F(Z_k, U_k)
$$

---

Output `params` of `build_solver` $= \begin{bmatrix}
w_{0,6:m}, w_{lb,6:m}, w_{ub,6:m}, g_{lb}, g_{ub} 
\end{bmatrix}$
where $w_\_ : (6+3)n \times 1 \,, g: 6n \times 1$
