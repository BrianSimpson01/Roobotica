#  Application of Frame Transformations

This document contains the solutions to exercises on **rotations** and **homogeneous transformations** in 3D.  
All matrices are presented in clean mathematical form.

---

##  Exercise 1

**Given:**  
A vector \( \mathbf{P}^A \) is rotated:

1. About axis \( \hat{Y}_A \) by **45°**  
2. Then about axis \( \hat{X}_A \) by **60°**

### Rotation about \( \hat{Y}_A \):



\[
R_Y(45^\circ) =
\begin{bmatrix}
\cos 45^\circ & 0 & \sin 45^\circ \\
0 & 1 & 0 \\
-\sin 45^\circ & 0 & \cos 45^\circ
\end{bmatrix}
=
\begin{bmatrix}
0.707 & 0 & 0.707 \\
0 & 1 & 0 \\
-0.707 & 0 & 0.707
\end{bmatrix}
\]



### Rotation about \( \hat{X}_A \):



\[
R_X(60^\circ) =
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos 60^\circ & -\sin 60^\circ \\
0 & \sin 60^\circ & \cos 60^\circ
\end{bmatrix}
=
\begin{bmatrix}
1 & 0 & 0 \\
0 & 0.5 & -0.866 \\
0 & 0.866 & 0.5
\end{bmatrix}
\]



### Total rotation (order matters):



\[
R = R_X(60^\circ) \cdot R_Y(45^\circ)
=
\begin{bmatrix}
0.707 & 0 & 0.707 \\
-0.612 & 0.5 & 0.612 \\
0.354 & 0.866 & -0.354
\end{bmatrix}
\]



---

##  Exercise 2

**Given:**  
Frame \(\{B\}\) is rotated with respect to \(\{A\}\) about \( \hat{X}_A \) by **30°**.  
The translation of \(\{B\}\) from \(\{A\}\) is:



\[
p_B =
\begin{bmatrix}
5 \\
10 \\
0
\end{bmatrix}
\]



### Rotation matrix:



\[
R_B = R_X(30^\circ) =
\begin{bmatrix}
1 & 0 & 0 \\
0 & 0.866 & -0.5 \\
0 & 0.5 & 0.866
\end{bmatrix}
\]



### Homogeneous transformation matrix:



\[
T_B =
\begin{bmatrix}
R_B & p_B \\
0 & 1
\end{bmatrix}
=
\begin{bmatrix}
1 & 0 & 0 & 5 \\
0 & 0.866 & -0.5 & 10 \\
0 & 0.5 & 0.866 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]



---

##  Exercise 3

**Given:**  
From the figure, obtain the homogeneous transformations.

### Homogeneous transformation from frame {B} to {A}:



\[
T_B =
\begin{bmatrix}
R_B & p_B \\
0 & 1
\end{bmatrix}
\]



### Homogeneous transformation from frame {C} to {A}:



\[
T_C =
\begin{bmatrix}
R_C & p_C \\
0 & 1
\end{bmatrix}
\]



### Using chaining:



\[
T_C = T_B \cdot T_{BC}
\]



---

## General Vector Transformation



\[
p^A = T_B \cdot p^B
\]



---

##  Conclusion

- **Exercise 1**: Demonstrates sequential rotations about different axes.  
- **Exercise 2**: Builds a homogeneous transformation matrix with rotation and translation.  
- **Exercise 3**: Shows chaining of transformations between frames.  

This material provides a solid foundation for robotics, simulation, and computer graphics applications.
