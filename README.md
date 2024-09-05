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

    接下来的开发可能会有一点点破坏性,所以新开一个分支来弄,不影响主分支的内容.


从目前阅读的内容看,DP主要的核心是把已经处理过的子问题的解存储起来,然后在需要的时候直接调用,而不是重新计算?

这让我觉得这里可能是不是需要用的是A*算法,而不是DP?

A*算法是一种启发式搜索算法,在搜索的时候,会根据当前的状态,通过一个启发函数来估计当前状态到目标状态的距离,然后根据这个距离来选择下一个状态,这样可以减少搜索的时间.

实际上,DP和A*算法是有一定的关系的,DP是一种最优化的方法,而A*算法是一种搜索的方法,在搜索的时候,可以使用DP来进行优化?

是不是需要看一下文献?