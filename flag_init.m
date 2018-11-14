%% Flag_init 
% Initializes the positions of the flags

function [L,R] = flag_init()
    left_flag_pos = rand([2,1]);
    left_flag_pos(1) = left_flag_pos(1)*2 - 1
    left_flag_pos(2) = left_flag_pos(2)*1.6 - 1.6
    
    right_flag_pos = rand([2,1]);
    right_flag_pos(1) = right_flag_pos(1)*2 - 1
    right_flag_pos(2) = right_flag_pos(2)*1.6
    
    L = left_flag_pos;
    R = right_flag_pos;
end