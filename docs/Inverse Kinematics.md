#  Iverse Kinematics 

## Forward Kinematics

### DH Parameters

| Joint | θᵢ | dᵢ | aᵢ | αᵢ |
|-------|-----|-----|-----|-----|
| 1 | θ₁ | 0 | a₁ | 0° |
| 2 | θ₂ | 0 | a₂ | 0° |
| 3 | θ₃ | 0 | a₃ | 0° |

All joints rotate about Z (pointing up). All links lie in the XY plane.

---


## Inverse Kinematics — Geometric Method

### Step 1 — Wrist Decoupling

Subtract link $a_3$ using the known orientation $\phi$ to find the wrist point W:

$$W_x = x - a_3\cos\phi$$

$$W_y = y - a_3\sin\phi$$

Distance from origin to wrist:

$$D = \sqrt{W_x^2 + W_y^2} = \sqrt{(x - a_3c_\phi)^2 + (y - a_3s_\phi)^2}$$

Expanding:

$$D = \sqrt{x^2 + y^2 - 2a_3(xc_\phi + ys_\phi) + a_3^2}$$

---

### Step 2 — Solve θ₂

Apply the *law of cosines* to triangle O–J₁–W:

$$\cos\theta_2 = \frac{D^2 - a_1^2 - a_2^2}{2\,a_1 a_2}$$

$$\sin\theta_2 = \pm\sqrt{1 - \cos^2\theta_2}$$

$$\theta_2 = \text{atan2}\!\left(\pm\sqrt{1 - \left(\frac{D^2 - a_1^2 - a_2^2}{2a_1a_2}\right)^2},\ \frac{D^2 - a_1^2 - a_2^2}{2a_1a_2}\right)$$

> *+* → elbow left solution  
> *−* → elbow right solution

---

### Step 3 — Solve θ₁

Define two auxiliary angles:

$$\alpha = \text{atan2}(W_y,\ W_x) = \text{atan2}(y - a_3s_\phi,\ x - a_3c_\phi)$$

$$\beta = \text{atan2}(a_2\sin\theta_2,\ a_1 + a_2\cos\theta_2)$$

Then:

$$\theta_1 = \alpha - \beta = \text{atan2}(y - a_3s_\phi,\ x - a_3c_\phi) - \text{atan2}(a_2s_2,\ a_1 + a_2c_2)$$

---

### Step 4 — Solve θ₃

From the orientation constraint $\phi = \theta_1 + \theta_2 + \theta_3$:

$$\theta_3 = \phi - \theta_1 - \theta_2$$

---

## Jacobian — Geometric Method

For a planar revolute joint rotating about $\hat{z} = [0,\ 0,\ 1]^T$, each column is:

$$J_i = \begin{bmatrix} \hat{z} \times (p_e - p_{i-1}) \\ \hat{z} \end{bmatrix} = \begin{bmatrix} -(y_e - y_{i-1}) \\ x_e - x_{i-1} \\ 1 \end{bmatrix}$$

---

### Column 1 — Joint 1, from $p_0 = (0,\ 0)$

$$p_e - p_0 = \begin{bmatrix} a_1c_1 + a_2c_{12} + a_3c_{123} \\ a_1s_1 + a_2s_{12} + a_3s_{123} \end{bmatrix}$$

$$J_1 = \begin{bmatrix} -(a_1s_1 + a_2s_{12} + a_3s_{123}) \\ a_1c_1 + a_2c_{12} + a_3c_{123} \\ 1 \end{bmatrix}$$

---

### Column 2 — Joint 2, from $p_1 = (a_1c_1,\ a_1s_1)$

$$p_e - p_1 = \begin{bmatrix} a_2c_{12} + a_3c_{123} \\ a_2s_{12} + a_3s_{123} \end{bmatrix}$$

$$J_2 = \begin{bmatrix} -(a_2s_{12} + a_3s_{123}) \\ a_2c_{12} + a_3c_{123} \\ 1 \end{bmatrix}$$

---

### Column 3 — Joint 3, from $p_2 = (a_1c_1 + a_2c_{12},\ a_1s_1 + a_2s_{12})$

$$p_e - p_2 = \begin{bmatrix} a_3c_{123} \\ a_3s_{123} \end{bmatrix}$$

$$J_3 = \begin{bmatrix} -a_3s_{123} \\ a_3c_{123} \\ 1 \end{bmatrix}$$

---

### Full Symbolic Jacobian

$$J = \begin{bmatrix} -(a_1s_1+a_2s_{12}+a_3s_{123}) & -(a_2s_{12}+a_3s_{123}) & -a_3s_{123} \\ a_1c_1+a_2c_{12}+a_3c_{123} & a_2c_{12}+a_3c_{123} & a_3c_{123} \\ 1 & 1 & 1 \end{bmatrix}$$

> *Rows 1–2* → linear velocity $(\dot{x},\ \dot{y})$  
> *Row 3* → angular velocity $\dot{\phi}$

---

## Determinant & Singularities

$$\det(J) = a_1 a_2 \sin\theta_2 + a_1 a_3 \sin(\theta_2 + \theta_3) + a_2 a_3 \sin\theta_3$$

Setting $\det(J) = 0$:

$$a_1 a_2 \sin\theta_2 + a_1 a_3 \sin(\theta_2+\theta_3) + a_2 a_3 \sin\theta_3 = 0$$

| Condition | Configuration |
|-----------|--------------|
| $\theta_2 = 0°$ or $180°$ | Links $a_1$ and $a_2$ fully aligned |
| $\theta_3 = 0°$ or $180°$ | Links $a_2$ and $a_3$ fully aligned |
| All three links collinear | Complete arm stretch or fold |

---

