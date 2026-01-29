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

General structure:

T_B =  
[  R_B   p_B ]  
[   0     1  ]  

T_C =  
[  R_C   p_C ]  
[   0     1  ]  

Chaining:  
T_C = T_B · T_BC  

With the values obtained:

T_B =  
[  1       0      0      5 ]  
[  0       0.866 -0.5   10 ]  
[  0       0.5    0.866  0 ]  
[  0       0      0      1 ]  

If T_BC is defined from the 30° inclination and prism dimensions (3, 2), then:  
T_C = T_B · T_BC  

---

## General Vector Transformation

p^A = T_B · p^B

---

## Conclusion

- **Exercise 1**: Sequential rotations about different axes.  
- **Exercise 2**: Homogeneous transformation matrix with rotation and translation.  
- **Exercise 3**: Chaining of transformations between frames {A}, {B}, {C}.  
