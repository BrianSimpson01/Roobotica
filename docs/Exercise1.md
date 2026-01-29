# Application of Frame Transformations

---

## Exercise 1

Rotation about Y_A (45°):

[  0.707   0      0.707 ]  
[  0       1      0     ]  
[ -0.707   0      0.707 ]  

Rotation about X_A (60°):

[  1       0      0     ]  
[  0       0.5   -0.866 ]  
[  0       0.866  0.5   ]  

Total rotation R = R_X(60°) · R_Y(45°):

[  0.707   0      0.707 ]  
[ -0.612   0.5    0.612 ]  
[  0.866   0.354 -0.354 ]  

---

## Exercise 2

Translation vector p_B:

[  5 ]  
[ 10 ]  
[  0 ]  

Rotation matrix R_B = R_X(30°):

[  1       0      0     ]  
[  0       0.866 -0.5   ]  
[  0       0.5    0.866 ]  

Homogeneous transformation T_B:

[  1       0      0      5 ]  
[  0       0.866 -0.5   10 ]  
[  0       0.5    0.866  0 ]  
[  0       0      0      1 ]  

---

## Exercise 3

Transformation from B to C (30° tilt + translation [3, 0, 2]):

T_BC =  
[  0.866   0    0.5    3 ]  
[  0       1    0      0 ]  
[ -0.5     0    0.866  2 ]  
[  0       0    0      1 ]  

Chaining:  
T_C = T_B · T_BC  

Final result:  

T_C =  
[  0.866   0     0.5     8     ]  
[ -0.433   0.866 0.25    10.732 ]  
[ -0.25    0.5   0.866   2.732 ]  
[  0       0     0      1     ]  

---

## General Vector Transformation

p^A = T_B · p^B

---

## Conclusion

- **Exercise 1**: Sequential rotations about different axes.  
- **Exercise 2**: Homogeneous transformation matrix with rotation and translation.  
- **Exercise 3**: Final homogeneous transformation matrix for frame {C} with respect to {A}.  
