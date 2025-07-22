% paths
cur_cd = cd; folder_name = strsplit(cd,'\'); folder_name = folder_name{length(folder_name)};
% data_cd = [extractBefore(cur_cd,'\Main') '\Data'];
traj_cd = [extractBefore(cur_cd,'\Main') '\Ref_Traj'];
fun_cd = [extractBefore(cur_cd,'\Main') '\Functions'];
robot_cd = [extractBefore(cur_cd,'\Main') '\Robot'];
% path(path,data_cd); 
path(path,traj_cd); 
path(path,fun_cd); 
path(path,robot_cd);

