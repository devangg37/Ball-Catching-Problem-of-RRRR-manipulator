function [theta1,theta2,theta3,theta4] = inverse(x,y,z)

%Required orientation of the gripper frame
theta=atan2d(y,x);

%Aligning reference frame with the gripper frame:
%First translating to (x,y,z)
%Second rotation about Y axis by 90 degrees.
%Third rotation about X axis. Since intial offset is 90 degrees so overall
%rotation is 90 + theta
T1 = [1 0 0 x;0 1 0 y;0 0 1 z;0 0 0 1];
T2 = [0 0 -1 0;0 1 0 0;1 0 0 0;0 0 0 1];
T3 = [1 0 0 0;0 cosd(90+theta) -sind(90+theta) 0;0 sind(90+theta)...
                                                 cosd(90+theta) 0;0 0 0 1];

Treq=T1*T2*T3;

%Finding degree of freedoms theta1, theta2, theta3, theta4 by using inverse
%kinematics.
theta1 = atan2d(Treq(1,3),-Treq(2,3));
theta3 = acosd(0.5*((Treq(1,4))^2+(Treq(2,4))^2+(Treq(3,4))^2-2));   
theta2 = acosd(sqrt((Treq(1,4))^2+...
            (Treq(2,4))^2)/sqrt((1+cosd(theta3))^2+...
      (sind(theta3))^2))-acosd((1+cosd(theta3))/sqrt((1+cosd(theta3))^2+...
            (sind(theta3))^2));
theta4 = acosd(sqrt((Treq(1,1))^2+(Treq(2,1))^2))-theta2-theta3;

end