function simOut = run_drone_model()
%RUN_DRONE_MODEL Open and simulate the bundled quadrotor forward model.
% The model's InitFcn calls InitParameters.m, so run from this folder.

modelDir = fileparts(mfilename('fullpath'));
oldDir = pwd;
restoreDir = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(modelDir);

load_system('drone_model.slx');
open_system('drone_model');
simOut = sim('drone_model');
end
