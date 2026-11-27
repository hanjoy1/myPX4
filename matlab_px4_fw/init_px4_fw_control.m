function p = init_px4_fw_control()
%INIT_PX4_FW_CONTROL Add MATLAB PX4 fixed-wing control folders to path.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'common'));
addpath(fullfile(root, 'params'));
addpath(fullfile(root, 'messages'));
addpath(fullfile(root, 'fw_path_frontend'));
addpath(fullfile(root, 'vtol_tailsitter'));
addpath(fullfile(root, 'fw_att_control'));
addpath(fullfile(root, 'fw_rate_control'));
p = fw_default_params();
end
