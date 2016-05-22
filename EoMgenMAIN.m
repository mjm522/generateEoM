
function EoMgenMAIN()
clc;
%joint type: 0- no joint, 1-ball joint, 2-universal joint, 3-hinge joint
%rigid body type: 1-capsule, 2-cylinder, 3-cuboid
%capsule - 2 dimensions - radius, length
%cylinder- 2 dimensions - radius, length
%cuboid  - 3 dimensions - length, breadth, height

%ARB - articulated rigid body, by default first link is taken as the parent
%link
%two link cylindrical manipulator - 2 DOF
% ARB.rbType = [2,2]; %type of rigid bodies
% ARB.prntLnk = [0,1]; %parent link of link
% ARB.prntJnt = [3,3]; %parent joint of link; this means this is how child and parent link is connected

%example from tutorial - 7 DOF
 ARB.rbType = [2,2,2,2]; %type of rigid bodies
 ARB.prntLnk = [0,1,2,2]; %parent link of link
 ARB.prntJnt = [2,1,3,3]; %parent joint of link; this means this is how child and parent link is connected

EoM = generateEoM(ARB);
end

function EoM = generateEoM(ARB)
n = length(ARB.rbType); %number of links
%x,y,z position, q orientation quarternion, v velocity, w angular velocity
%dof = length(ARB.prntJnt);

for i=1:n %in generalized coodrinates    
    switch(ARB.prntJnt(i))

    case 1 % for ball joint
        q = [sym(strcat('q',num2str(i),num2str(1))),...
                         sym(strcat('q',num2str(i),num2str(2))),...
                         sym(strcat('q',num2str(i),num2str(3)))];
        %generalized coordinates
        ARB.rgdlnks(i).q = q;
        %local angular velocity
        ARB.rgdlnks(i).dq = [sym(strcat('dq',num2str(i),num2str(1))),...
                         sym(strcat('dq',num2str(i),num2str(2))),...
                         sym(strcat('dq',num2str(i),num2str(3)))]; 
        %generalized acceleration 
        ARB.rgdlnks(i).ddq = [sym(strcat('ddq',num2str(i),num2str(1))),...
                         sym(strcat('ddq',num2str(i),num2str(2))),...
                         sym(strcat('ddq',num2str(i),num2str(3)))]; 
        %local angular velocity
        ARB.rgdlnks(i).w = q;
        %rotation matrix to parent link frame
        ARB.rgdlnks(i).R =  rotX(q(1))*rotY(q(2))*rotZ(q(3));
    case 2 % for universal joint
        q = [sym(strcat('q',num2str(i),num2str(1))),...
                         sym(strcat('q',num2str(i),num2str(2)))]; 
        %generalized coordinates
        ARB.rgdlnks(i).q = q;
        %generalized velocity
        ARB.rgdlnks(i).dq = [sym(strcat('dq',num2str(i),num2str(1))),...
                         sym(strcat('dq',num2str(i),num2str(2)))];
        %generalized acceleration             
        ARB.rgdlnks(i).ddq = [sym(strcat('ddq',num2str(i),num2str(1))),...
                         sym(strcat('ddq',num2str(i),num2str(2)))]; 
        %local angular velocity
        ARB.rgdlnks(i).w = [q,0];
        %rotation matrix to parent link frame
        ARB.rgdlnks(i).R =  rotX(q(1))*rotY(q(2));
    case 3 % for hinge joint
        q = sym(strcat('q',num2str(i),num2str(1)));
        %generalized coordinates
        ARB.rgdlnks(i).q = q;
        %generalized velocity
        ARB.rgdlnks(i).dq = sym(strcat('dq',num2str(i),num2str(1))); 
        %generalized acceleration
        ARB.rgdlnks(i).ddq = sym(strcat('ddq',num2str(i),num2str(1)));
        %local angular velocity
        ARB.rgdlnks(i).w = [0,0,q];
        %rotation matrix to parent link frame
        ARB.rgdlnks(i).R =  rotZ(q(1));
    end
end

ARB = findGenCoord(ARB);

for i=1:n %in cartesian coordinates
                
    momentIntia = [sym(strcat('i11_l', num2str(i))), sym(strcat('i12_l', num2str(i))), sym(strcat('i13_l', num2str(i)));...
                   sym(strcat('i12_l', num2str(i))), sym(strcat('i22_l', num2str(i))), sym(strcat('i23_l', num2str(i)));...
                   sym(strcat('i13_l', num2str(i))), sym(strcat('i23_l', num2str(i))), sym(strcat('i33_l', num2str(i)))];
    %mass+inertia matrix of link
    ARB.rgdlnks(i).Mc = [sym(strcat('m', num2str(i)))*eye(3), zeros(3); zeros(3), momentIntia];
    %centre of mass of each link
    ARB.rgdlnks(i).com = [sym(strcat('cm',num2str(i),'x')); sym(strcat('cm',num2str(i),'y')); sym(strcat('cm',num2str(i),'z'))];
    %location of its parent joint in the parent link frame
    ARB.rgdlnks(i).lnkPos = [sym(strcat('l',num2str(i),'x')); sym(strcat('l',num2str(i),'y')); sym(strcat('l',num2str(i),'z'))];
    %parent link index
    ARB.rgdlnks(i).prntL = ARB.prntLnk(i);
    %parent link joint index
    ARB.rgdlnks(i).prntJ = ARB.prntJnt(i);
    %physical dimensions of the link
    ARB.rgdlnks(i).dim = findRelDim(ARB.rbType(i));               
    %cartesian force acting on link
    ARB.rgdlnks(i).f = [sym(strcat('Fx',num2str(i))), sym(strcat('Fy',num2str(i))), sym(strcat('Fz',num2str(i)))];
    %cartesian torque acting on link
    ARB.rgdlnks(i).tau = [sym(strcat('Taux',num2str(i))), sym(strcat('Tauy',num2str(i))), sym(strcat('Tauz',num2str(i)))];
    %transformation matrix to the base frame
    ARB.rgdlnks(i).T = transform2Base(ARB,i);
    %local jacobian matrix
    ARB.rgdlnks(i).jw = jacobian(ARB.rgdlnks(i).w,ARB.rgdlnks(i).q);
end
ARB = findJ_dJ(ARB);

dof = noDOF(ARB);
Q = ARB.Q;
dQ = ARB.dQ;
ddQ = ARB.ddQ;

M = zeros(dof,dof);
C = zeros(dof,dof);

for i = 1:n
    J = ARB.rgdlnks(i).J; Mc = ARB.rgdlnks(i).Mc; dJ = ARB.rgdlnks(i).dJ;
    w = ARB.rgdlnks(i).Jw*dQ';
    m = J'*Mc*J;
    c  = J'*Mc*dJ + J'*[zeros(3,5),zeros(3,1); zeros(3,5), w]*Mc*J;
    M = M + m;
    C = C + c;
end

%as symbolic expression
EoM.M = M; %mass matrix
EoM.C = C; %coriolis matrix
EoM.eom = M*ddQ' + C*dQ'; %total equations of motion

%as character expressions so that direct substitution works
% EoM.M = char(M); %mass matrix
% EoM.C = char(C); %coriolis matrix
% EoM.eom = char(M*ddQ' + C*dQ'); %total equations of motion

end

function dim = findRelDim(indx)
switch(indx)
    case 1 %capsule
        dim = [sym(strcat('len', num2str(indx))), sym(strcat('rad', num2str(indx)))];
    case 2 %cylinder
        dim = [sym(strcat('len', num2str(indx))), sym(strcat('rad', num2str(indx)))];
    case 3 %cuboid
        dim = [sym(strcat('len', num2str(indx))), sym(strcat('bdh', num2str(indx))), sym(strcat('hgt', num2str(indx)))];
end
end

function ARB = findJ_dJ(ARB)
n = length(ARB.rbType); %number of links

Q = ARB.Q;
dQ = ARB.dQ;
for i = 1:n
    prnt = findParentSeq(ARB, i);
    tmp = ARB.rgdlnks(i).T*[ARB.rgdlnks(i).com;1];
    %velocity jacobian
    Jv = jacobian(tmp(1:3),Q);
    Jw = [];
    for j = 1:n
        if(ismember(j,prnt))
            mem = ARB.rgdlnks(j).T(1:3,1:3)*ARB.rgdlnks(j).jw;
        else
            mem = zeros(3,nnz(ARB.rgdlnks(j).q));
        end
        %angular velocity jacobian
        Jw = [Jw,mem];
    end
    ARB.rgdlnks(i).Jv = Jv;
    ARB.rgdlnks(i).Jw = Jw;
    ARB.rgdlnks(i).J = [Jv;Jw];
    ARB.rgdlnks(i).dJ = jacobian([Jv;Jw]*dQ',Q);
end
end

function ARB = findGenCoord(ARB)
n = length(ARB.rbType); %number of links
Q = [];
dQ = [];
ddQ = [];
for i = 1:n
    Q = [Q,ARB.rgdlnks(i).q];
    dQ = [dQ,ARB.rgdlnks(i).dq];
    ddQ = [ddQ,ARB.rgdlnks(i).ddq];
end
ARB.Q = Q;
ARB.dQ = dQ;
ARB.ddQ = ddQ;
end

function T = transform2Base(ARB,indx)
prnt = findParentSeq(ARB, indx);
%link = indx;
T = [eye(3), zeros(3,1) ; zeros(1,3),1];
for i = prnt(2:end)
    %prntL = ARB.rgdlnks(i).prntL;
    %prntJ = ARB.rgdlnks(i).prntJ;
    T = T*[ARB.rgdlnks(i).R, ARB.rgdlnks(i).lnkPos ; zeros(1,3),1];
end

end


function prnt = findParentSeq(ARB, indx) %find sequence of parents till root, zero denotes world
prnt = indx;
i = indx;
while(i >= 1) %trace back to root
    prnt = [prnt,ARB.prntLnk(i)];
    i = ARB.prntLnk(i);
end
prnt = fliplr(prnt);
end

function dof = noDOF(ARB)
n = length(ARB.rbType); %number of links
dof = 0;

for i = 1:n
    switch(ARB.rgdlnks(i).prntJ)
        case 0
            jntDof = 0; % for no joint
        case 1
            jntDof = 3; % for ball joint
        case 2
            jntDof = 2; % for universal joint
        case 3
            jntDof = 1; % for hinge joint
    end
    dof = dof + jntDof;
end

end

%rotation matrices
function Rx = rotX(th)
Rx = [1,0,0;...
      0,cos(th), -sin(th);...
      0,sin(th), cos(th)];
end

function Ry = rotY(th)
Ry = [cos(th), 0, sin(th);...
      0,1,0;...
      -sin(th), 0, cos(th)];
end

function Rz = rotZ(th)
Rz = [cos(th), -sin(th),0;...
      sin(th), cos(th), 0;...
      0,0,1];
end