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

这个问题我可以再描述一下, 一个翼伞系统 $V,Vz,\bar{u}$ 从一个起点$x, y, H$ 开始滑翔, 在滑翔过程中,控制变量可以简化为 $\dot{\omega}$ , 即滑翔角度的变化, 这个控制实际上由双侧差动尾缘下拉来实现, 控制输入 $u = \dot{\omega} \in [-\bar{u}, \bar{u}]$. 同时,也应该知道翼伞本身是配平的,所以忽略 $z$ 方向的动力学, 只考虑 $x, y$ 平面内的动力学方程. 

滑翔过程的目标是尽量减小着陆时与地面的相对速度,在考虑最简单的稳定风场时,这意味着要实现逆风着陆.

把着陆点定义为原点, 稳定风向定义为 $-x$ 轴方向, 考虑问题通用的解法,或者说一类问题的协调解法, 可能要形成如此的表格.

| $x_0$ | $y_0$ | $\omega_0$ | $H$ | $V$ | $V_z$ | $V_w$ | $\bar{u}$ | $T$ | $J$ | $u(t), t\in[0, T]$ |
| ----- | ----- | ---------- | --- | --- | ----- | ----- | --------- | --- | --- | ------------------ |
| 500   | 500   | 0          | 300 | 15  | 10    | 1     | 25        | 30  |     |
| 400   | 400   | 0          | 300 | 15  | 10    | 1     | 25        | 30  |     |

这里有几个有假设条件推出的事实.

1. 翼伞飞行时间为 $T= \frac{H}{V_z}$, 这个是一个确定的值,不是一个变量.
2. 着陆点的速度为 $V_x(T) = V \cos\omega(T) - V_w$, $V_y(T) = V \sin\omega(T)$.
3. 翼伞最大飞行距离为 $V \cdot T$, 这个会和风速一起确定能够达到着陆点的起始范围, 这个问题本身就有一点点意思.
4. 这个起始范围的不确定性也是相对容易解决的. 
5. 由 $\bar{u}$ 的存在, 翼伞的最小转弯半径也是有限的, 转弯半径 $r(t)=\frac{V}{\dot{\omega}}$的最小值为$\underbar{r} = V/\bar{u}$.
6. 这样,上面的可行集合就可以看作是一个几何问题.

### 可行集合的分析

只考虑 $x, y$ 方向, 匹配起始的 $\omega_0$, 根据最远距离, 可以得到不等式:

$$
\sqrt{(x_0-V_w T)^2 + y_0^2} \le H \cdot \frac{V}{V_z}
$$

这是以 $(-V_w T, 0)$ 为圆心, $H \cdot \frac{V}{V_z}$ 为半径的圆. 在圆上的任何一点, 初始方位角必须为:

 $$\omega_0 = \arctan\frac{y_0}{x_0-V_w T}$$

 因此, 在考虑问题解的表格并试图进行轨迹规划问题综合时, 初始的可行集合为这个圆的内点. 我们还可以进一步求出每个内点对应的 $\omega_0$的允许范围, 通过求解最小转弯半径相关的几何问题.

这个集合问题的实质就是,一条由最小半径为 $\underbar{r}$ 的圆弧和直线线段组成的曲线, 连接 $(x_0, y_0)$ 和原点, 这个曲线的长度是 $V \cdot T$.

这个曲线起点的切线, 由 $\omega_0$确定.