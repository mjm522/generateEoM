syms i11_l1 i12_l1 i13_l1 i22_l1 i23_l1 i33_l1; 
syms i11_l2 i12_l2 i13_l2 i22_l2 i23_l2 i33_l2;
syms i11_l3 i12_l3 i13_l3 i22_l3 i23_l3 i33_l3;
syms i11_l4 i12_l4 i13_l4 i22_l4 i23_l4 i33_l4;
syms cm1x cm1y cm1z cm2x cm2y cm2z; 
syms cm3x cm3y cm3z cm4x cm4y cm4z;
syms l1x l1y l1z l2x l2y l2z;
syms l3x l3y l3z l4x l4y l4z;
syms m1 m2 m3 m4;
syms q11 q12 dq11 dq12 ddq11 ddq12;
syms q21 q22 q23 dq21 dq22 dq23 ddq21 ddq22 ddq23;
syms q31 dq31 ddq31;
syms q41 dq41 ddq41;

h = eval(subs(M, [i11_l1 i12_l1 i13_l1 i22_l1 i23_l1 i33_l1,... 
                 i11_l2 i12_l2 i13_l2 i22_l2 i23_l2 i33_l2,...
                 i11_l3 i12_l3 i13_l3 i22_l3 i23_l3 i33_l3,....
                 i11_l4 i12_l4 i13_l4 i22_l4 i23_l4 i33_l4,...
                 cm1x cm1y cm1z cm2x cm2y cm2z,...
                 cm3x cm3y cm3z cm4x cm4y cm4z,...
                 l1x l1y l1z l2x l2y l2z,...
                 l3x l3y l3z l4x l4y l4z,...
                 m1 m2 m3 m4,...
                 q11 q12 dq11 dq12 ddq11 ddq12,...
                 q21 q22 q23 dq21 dq22 dq23 ddq21 ddq22 ddq23,...
                 q31 dq31 ddq31,...
                 q41 dq41 ddq41], abs(ceil(rand(1,73).*10))));