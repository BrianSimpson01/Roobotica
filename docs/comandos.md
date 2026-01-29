#  Application of Frame Transformations

This document contains the resolution of exercises on **rotations** and **homogeneous transformations** in 3D.  
It presents formulas, matrices, and results for academic use.

---

##  Introduction

In robotics and computer graphics, it is essential to describe the **orientation** and **position** of objects using rotation matrices and homogeneous transformations.  
The following exercises demonstrate how to apply these mathematical tools step by step.

---

## Exercise 1

**Statement:**  
A vector \( \mathbf{P}^A \) is rotated:

1. About axis \( Y_A \) by **45°**  
2. Then about axis \( X_A \) by **60°**

### Rotation about \( Y_A \):



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



### Rotation about \( X_A \):



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



### Total rotation (order: first Y, then X):



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

**Statement:**  
Frame \(\{B\}\) is rotated with respect to \(\{A\}\) about \( X_A \) by **30°**.  
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



### Homogeneous transformation:



\[
T_B =
\begin{bmatrix}
1 & 0 & 0 & 5 \\
0 & 0.866 & -0.5 & 10 \\
0 & 0.5 & 0.866 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]



---

##  Exercise 3

**Statement:**  
From the figure, obtain the transformations \(T_A\) and \(T_B\).  
(Since the figure is incomplete in the PDF, numerical values cannot be fully determined, but the general structure is shown.)

### Transformation of {B} with respect to {A}:



\[
T_B =
\begin{bmatrix}
R_B & p_B \\
0 & 1
\end{bmatrix}
\]



### Transformation of {C} with respect to {A} (chaining):



\[
T_C = T_B \cdot T_{BC}
\]



---

##  Conclusion

- **Exercise 1** shows how to combine rotations in a specific order.  
- **Exercise 2** builds a homogeneous transformation with rotation and translation.  
- **Exercise 3** applies the concept of **chaining transformations**.  

This material serves as a foundation for robotics, simulation, and computer graphics projects.
