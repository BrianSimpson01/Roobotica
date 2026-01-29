# APPLICATION OF FRAME TRANSFORMATIONS

---

## Exercise 1

**Given:**  
A vector \( \mathbf{P}^A \) is rotated:

1. About axis \( \hat{Y}_A \) by **45°**  
2. Then about axis \( \hat{X}_A \) by **60°**

### Rotation about \( \hat{Y}_A \)

\[
{}^A R_Y(45^\circ) =
\begin{bmatrix}
\cos 45^\circ & 0 & \sin 45^\circ \\
0 & 1 & 0 \\
-\sin 45^\circ & 0 & \cos 45^\circ
\end{bmatrix}
\]

### Rotation about \( \hat{X}_A \)

\[
{}^A R_X(60^\circ) =
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos 60^\circ & -\sin 60^\circ \\
0 & \sin 60^\circ & \cos 60^\circ
\end{bmatrix}
\]

### Total rotation (order matters)

\[
{}^A R = {}^A R_X(60^\circ) \cdot {}^A R_Y(45^\circ)
\]

---

## Exercise 2

**Given:**  
Frame \(\{B\}\) is rotated with respect to \(\{A\}\) about \( \hat{X}_A \) by **30°**.

### Translation of {B} from {A}

\[
{}^A \mathbf{p}_B =
\begin{bmatrix}
5 \\
10 \\
0
\end{bmatrix}
\]

### Rotation matrix

\[
{}^A R_B = {}^A R_X(30^\circ) =
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos 30^\circ & -\sin 30^\circ \\
0 & \sin 30^\circ & \cos 30^\circ
\end{bmatrix}
\]

### Homogeneous transformation matrix

\[
{}^A T_B =
\begin{bmatrix}
{}^A R_B & {}^A \mathbf{p}_B \\
\mathbf{0}_{1\times3} & 1
\end{bmatrix}
\]

### Expanded form

\[
{}^A T_B =
\begin{bmatrix}
1 & 0 & 0 & 5 \\
0 & \cos 30^\circ & -\sin 30^\circ & 10 \\
0 & \sin 30^\circ & \cos 30^\circ & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

---

## Exercise 3

From the given image:

### Homogeneous transformation from frame {B} to {A}

\[
{}^A T_B =
\begin{bmatrix}
{}^A R_B & {}^A \mathbf{p}_B \\
\mathbf{0}_{1\times3} & 1
\end{bmatrix}
\]

### Homogeneous transformation from frame {C} to {A}

\[
{}^A T_C =
\begin{bmatrix}
{}^A R_C & {}^A \mathbf{p}_C \\
\mathbf{0}_{1\times3} & 1
\end{bmatrix}
\]

### Using chaining

\[
{}^A T_C = {}^A T_B \cdot {}^B T_C
\]

---

## General Vector Transformation

\[
{}^A \mathbf{p} = {}^A T_B \cdot {}^B \mathbf{p}
\]

---

**END OF APPLICATION**
