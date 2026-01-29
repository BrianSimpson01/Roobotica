# Application of Frame Transformations

This document contains the solutions to exercises on **rotations** and **homogeneous transformations** in 3D.  

---

## Exercise 1

**Rotation about Y_A (45°):**

$$
\begin{bmatrix}
0.707 & 0 & 0.707 \\
0 & 1 & 0 \\
-0.707 & 0 & 0.707
\end{bmatrix}
$$

**Rotation about X_A (60°):**

$$
\begin{bmatrix}
1 & 0 & 0 \\
0 & 0.5 & -0.866 \\
0 & 0.866 & 0.5
\end{bmatrix}
$$

**Total rotation R = R_X(60°) · R_Y(45°):**

$$
\begin{bmatrix}
0.707 & 0 & 0.707 \\
-0.612 & 0.5 & 0.612 \\
0.866 & 0.354 & -0.354
\end{bmatrix}
$$

---

## Exercise 2

**Translation vector p_B:**

$$
\begin{bmatrix}
5 \\
10 \\
0
\end{bmatrix}
$$

**Rotation matrix R_B = R_X(30°):**

$$
\begin{bmatrix}
1 & 0 & 0 \\
0 & 0.866 & -0.5 \\
0 & 0.5 & 0.866
\end{bmatrix}
$$

**Homogeneous transformation T_B:**

$$
\begin{bmatrix}
1 & 0 & 0 & 5 \\
0 & 0.866 & -0.5 & 10 \\
0 & 0.5 & 0.866 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

---

## Exercise 3

**General structure:**

$$
T_B =
\begin{bmatrix}
R_B & p_B \\
0 & 1
\end{bmatrix}
$$

$$
T_C =
\begin{bmatrix}
R_C & p_C \\
0 & 1
\end{bmatrix}
$$

**Chaining:**

$$
T_C = T_B \cdot T_{BC}
$$

---

## General Vector Transformation

$$
p^A = T_B \cdot p^B
$$

---

## Conclusion

- **Exercise 1**: Sequential rotations about different axes.  
- **Exercise 2**: Homogeneous transformation matrix with rotation and translation.  
- **Exercise 3**: Chaining of transformations between frames.  
