# Mobile Robot
<p align="right">
최초 작성일 : 2025-03-01 / 마지막 수정일 : 2025-03-01
</p>


모바일 로봇이 정해진 경로를 추적하여 따라 가도록 제어기 하기 위해 제어기를 설계하는 것 목표로 한다.

## 1. 로봇 상태 방정식

모바일 로봇(이동 로봇)은 바퀴 등으로 이동할 수 있는 로봇을 의미한다. 두 바퀴의 속력을 동일하게 함하면 직진 운동을 하고, 속력을 다르게 하면 회전 운동을 할 수 있다. 이러한 로봇을 제어하기 위해서는 로봇의 상태와 제어 하기 위한 변수를 정의한다.

2차원의 직교 좌표에서 구동 바퀴가 2개인 모바일 로봇은 시간이 $t$인 경우, 로봇의 상태 $q(t)$는 직교 좌표 $(x(t),y(t))$가 되고, 바라 보는 방향을 x축과의 각도 $\theta(t)$를 이용해서 표현할 수 있다. 이러한 로봇은 위에서 언급했듯, 직진 운동과 회전 운동을 통해 제어할 수 있기 때문에, 선속도 $v(t)$와 각속도 $w(t)$를 통해 표현할 수 있다. 따라서, 로봇에 대한 상태 변수 $q(t)$와 제어 입력 변수 $u(t)$는 아래와 같이 정의할 수 있다.

$$
q(t) = 
\begin{bmatrix}
x(t), y(t), \theta(t)
\end{bmatrix}^{T}
,\quad
u(t) =
\begin{bmatrix}
v(t), w(t)
\end{bmatrix}^{T}
$$

두 변수를 연립하기 위해, 상태 변수 $q(t)$를 미분하면,

$$
\dot{q}(t) =
\begin{bmatrix}
\dot{x}(t) \\
\dot{y}(t) \\
\dot{\theta}(t)
\end{bmatrix}
= \begin{bmatrix}
v(t) \cos\theta(t) \\
v(t) \sin\theta(t) \\
w(t)
\end{bmatrix}
= \begin{bmatrix}
\cos\theta(t) & 0 \\
\sin\theta(t) & 0 \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
v(t) \\
w(t)
\end{bmatrix}
$$

따라서, 로봇의 상태 방정식(State equation)이 유도된다.

$$
\dot{q}(t) = S(q(t)) u(t)
$$

## 2. 에러 상태 방정식

로봇이 정해진 경로를 따라가도록 제어기를 설계해야 한다. 이를 위해 먼저 로봇 시스템을 수학적으로 모델링하여 정해진 궤적을 추적할 수 있도록 설계해야 한다. 로봇이 주어진 경로를 따라가도록 제어하는 문제는 리더-팔로워 문제와 수학적으로 동일한 형태를 갖는다. 이러한 이유로, 리더-팔로워 문제를 살펴보자.

리더-팔로워 문제는 여러 로봇이 함께 이동하는 방법 중 하나로, 다중 로봇 시스템에서 한 대의 리더 로봇(LR)이 목표 궤적을 따라가고, 나머지 팔로워 로봇(FR)들이 이를 따라가도록 설계하는 제어 방식이다. 리더 로봇은 동역학 모델을 기반으로 상태를 업데이트하며 이동하지만, 팔로워 로봇은 리더 로봇의 움직임을 참고하여 자신의 위치 오차를 최소화 하도록 제어 알고리즘을 적용한다.

$$
q_{L} =
\begin{bmatrix}
x_{L} \\ 
y_{L} \\ 
\theta_{L}
\end{bmatrix}
,\quad
q_{F} =
\begin{bmatrix}
x_{F} \\ 
y_{F} \\ 
\theta_{F}
\end{bmatrix}
$$

FR를 제어하기 위해 상태 오차 $q_{e}$를 최소화 하도록 하기 때문에 아래와 같이 FR을 기준으로 LR에 대한 상태 추종 오차를 정의한다. 

$$
e = 
\begin{bmatrix}
e_{x} \\ 
e_{y} \\ 
e_{\theta}
\end{bmatrix} 
= \begin{bmatrix}
\cos\theta_F & \sin\theta_F & 0 \\ 
-\sin\theta_F & \cos\theta_F & 0 \\ 
0 & 0 & 1
\end{bmatrix}
\left( q_L - q_F \right)
$$

위 수식은 두 로봇의 상태 $q_L$, $q_F$의 전역 좌표계에서 계산된 오차 상태를 FR의 국소(local) 좌표계로 변환한 것을 의미한다. 이제 위 상태값을 대입하여 식을 정리하면 아래와 같다.

$$
e = 
\begin{bmatrix}
e_{x} \\ 
e_{y} \\ 
e_{\theta}
\end{bmatrix} 
= \begin{bmatrix}
\cos\theta_F & \sin\theta_F & 0 \\ 
-\sin\theta_F & \cos\theta_F & 0 \\ 
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
x_L - x_F \\ 
y_L - y_F \\ 
\theta_L - \theta_F
\end{bmatrix} 
= \begin{bmatrix}
\cos\theta_F(x_L - x_F) + \sin\theta_F(y_L - y_F) \\ 
-\sin\theta_F(x_L - x_F) + \cos\theta_F(y_L - y_F) \\ 
\theta_L - \theta_F
\end{bmatrix}
$$

그러면, 에러 상태 방정식을 얻기 위해 위 식을 미분하면,

$$
\begin{align*}
\dot{e} 
= \begin{bmatrix}
v_L \cos e_{\theta} + \omega_F e_{y} - v_F \\
v_L \sin e_{\theta} - e_{x} \omega_F \\
\omega_L - \omega_F
\end{bmatrix} =
\begin{bmatrix}
\omega_F e_{y} - v_F \\ 
-e_{x} \omega_F \\
-\omega_F
\end{bmatrix} +
\begin{bmatrix}
\cos e_{\theta} & 0 \\
\sin e_{\theta} & 0 \\
0 & 1
\end{bmatrix} u
\end{align*}
$$


