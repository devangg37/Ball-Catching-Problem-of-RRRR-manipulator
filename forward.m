function [P1,P2,P3,P4,P5] = forward(theta1,theta2,theta3,theta4)

%Transformation matrix for (0,1) (1,2) (2,3) (3,4) pair of axis
T01 = [cosd(theta1) -sind(theta1) 0 0;sind(theta1) cosd(theta1) 0 0;...
                                                          0 0 1 0;0 0 0 1];
T12 = [cosd(theta2) -sind(theta2) 0 0;0 0 -1 0;...
                                    sind(theta2) cosd(theta2) 0 0;0 0 0 1];
T23 = [cosd(theta3) -sind(theta3) 0 1;sind(theta3) cosd(theta3) 0 0;...
                                                          0 0 1 0;0 0 0 1];
T34 = [cosd(theta4) -sind(theta4) 0 1;sind(theta4) cosd(theta4) 0 0;...
                                                          0 0 1 0;0 0 0 1];

%Finding postion co-ordinates of the axis attached to the various frames.
%P1 represents the ground or reference frame.
P1 = [0,0,-1,1]';
P2 = T01*T12*[0,0,0,1]';
P3 = T01*T12*T23*[0,0,0,1]';
P4 = T01*T12*T23*T34*[0,0,0,1]';
P5 = T01*T12*T23*T34*[1,0,0,1]';

end