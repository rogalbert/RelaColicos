%% Flag_init 
% Initializes the positions of the flags

function [Left_handle,Right_handle] = flag_init()
    left_flag_pos = rand([2,1]);
    left_flag_pos(1) = left_flag_pos(1)*(-1.5); 
    left_flag_pos(2) = left_flag_pos(2)*1.9 - 1;
    
    
    right_flag_pos = rand([2,1]);
    right_flag_pos(1) = right_flag_pos(1)*1.5;
    right_flag_pos(2) = right_flag_pos(2)*1.9 - 1;
    
    
    % Creating Plot handles
    Left_handle = cell(1);
    Right_handle = cell(1);
    
    Left_handle = plot(left_flag_pos(1),left_flag_pos(2),'r.');
    Right_handle = plot(right_flag_pos(1), right_flag_pos(2),'b.');
    
    
end