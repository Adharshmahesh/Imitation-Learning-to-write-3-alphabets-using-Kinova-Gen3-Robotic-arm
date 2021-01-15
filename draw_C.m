function [end_effector_pos, joint_pos]= draw_C(robot, q, DataO)

endEffector = robot.BodyNames{end};
dim_steps = 200;
time_step = 1/100;
logger.current_state.joint_vel  = zeros(7, dim_steps) ;
logger.current_state.joint_pos = zeros(7, dim_steps);
maximum = -100;
minimum = 100;

for i = 1:dim_steps
    
   velocity = [0; DataO(1,i)/80; DataO(2,i)/80];
   
   if max(velocity)>maximum
      maximum = max(velocity); 
       
   end
   if min(velocity)<minimum
      minimum = min(velocity); 
   end
     
   jacobian = robot.geometricJacobian(q, endEffector);
   
   pinv_jacobian = pinv(jacobian(4:6,:));
   jointvel = pinv_jacobian*velocity;
   
   q = q + jointvel*time_step;
   logger.current_state.joint_vel(:,i)  = jointvel;
   logger.current_state.joint_pos(:,i) = q;
       
end

end_effector_pos = zeros(dim_steps,3);
if (maximum<0.8373) && (minimum>-0.8373)
for k=1:dim_steps
    
    joint_pos(:,k) =  logger.current_state.joint_pos(:,k);
    end_effector_pos(k,:) = tform2trvec(robot.getTransform(logger.current_state.joint_pos(:,k), endEffector)); 
    
end
end
end