function [] = main_ball_catch(xball,yball,zball)

%Intially the links are parallel to the X axis and horizontal
x0 = 2;
y0 = 0;
z0 = 0;

%Finding the parameters theta1, theta2, theta3, theta4 using inverse 
%kinematics. 
[theta1,theta2,theta3,theta4] = inverse(x0,y0,z0);

%Here P(i,j,k) represents the following:
%i stands for the time instant i.e. t = 1,2,3.....
%j represents the jth joint. Here j = 1 to 5 
%k represents the co-ordinate of the jth axis at ith instant. k goes from 1
%to 3 i.e. x,y,z co-oridnate.
[P(1,1,:),P(1,2,:),P(1,3,:),P(1,4,:),P(1,5,:)] = forward(theta1,theta2,...
                                                            theta3,theta4);

%Projecting the co-ordinte of the ball on the sphere x^2 + y^2 + z^2 = 4.
%zlimit represents the maximum z upto which the system can go in z
%direction.
zlimit = sqrt(4-xball^2-yball^2);

%Discretising the path by knowing the trajectory of the ball into 16 steps.
%The last step represent the point on the boundary. Manipulator starts
%intially from (xball, yball, 0).
if((xball^2+yball^2<=4) && zball>2)
    for i=1:16
        [theta1,theta2,theta3,theta4] = ...
                                    inverse(xball,yball,((i-1)*zlimit)/15);
        [P(i+1,1,:),P(i+1,2,:),P(i+1,3,:),P(i+1,4,:),P(i+1,5,:)] = ...
                                      forward(theta1,theta2,theta3,theta4);        
    end
end

%Catching the ball somewhere in middle. As our maximum step is 16, ball is
%catched at 8 step.
%Simulating the process by plotting the system and ball on x-y-z space at
%various instants from t = 1 to 8.
for i=1:8
    xlabel('x');ylabel('y');zlabel('z');
    plot3(P(i,1:2,1),P(i,1:2,2),P(i,1:2,3),'k','LineWidth',5)
    hold on
    plot3(P(i,2:3,1),P(i,2:3,2),P(i,2:3,3),'b','LineWidth',2)
    plot3(P(i,3:4,1),P(i,3:4,2),P(i,3:4,3),'r','LineWidth',2)
    plot3(P(i,4:5,1),P(i,4:5,2),P(i,4:5,3),'k','LineWidth',1)
    plot3(xball,yball,zball-(i-1)*(zball-P(8,5,3))/7,'o')
    hold off
    xlim([-3,3])
    ylim([-3,3])
    zlim([-1,10])
    pause(1)
end

end