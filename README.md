## Parafoil problem

To fly a well designed parafoil requires good understanding of its dynamics. 

We start with the following assumptions:

1. The parafoil is a point mass;
2. Gravity is perfectly balanced by the lift force;
3. The parafoil is steered by changing the heading angle of the parafoil;
4. Wind remains constant at this early stage;
5. x-y plane is the horizontal plane, z is the vertical direction;
6. x is opposite to the direction of the wind, y is perpendicular to the wind direction;
7. Target is at (0,0,0) for (x, y, z) coordinates;
8. The touch down point relative velocity to the ground should be minimized.


## Parafoil dynamics

$$\left\{
\begin{aligned}
\dot{x} &= V_x = V \cos(\omega) - V_w\\
\dot{y} &= V_y = V \sin(\omega) \\
\dot{\omega} &= u
\end{aligned}\right.
$$

Flight time is determined by initial height and vertical velocity,

$$
T = \frac{H}{V_z}
$$

## Optimal control

The optimal control problem is to minimize the touch down point relative velocity to the ground. The cost function is defined as

$$
\begin{aligned}
& \arg\min_u & \sqrt{V_x(T)^2 + V_y(T)^2} \\
& \text{s.t.} \\
& & |x(T)|^2 + |y(T)|^2  = 0
 \end{aligned}
$$

Or equivalently, even more realistically,

$$
J = \arg\min_u |x(T)|^2 + |y(T)|^2 + \sqrt{V_x(T)^2 + V_y(T)^2 + V_z(T)^2}

$$

There should be normalizations for $x, y$, and vecolity residuals, to make the cost function dimensionless.


## Discretization and dynamic programming

When starting state and ending state are defined, the problem can be discretized and solved by dynamic programming. The state space is defined as

$$
\begin{aligned}
& \mathcal{X} = \{x, y, \omega\} \\
& \mathcal{t} = \{t\}
\end{aligned}
$$

