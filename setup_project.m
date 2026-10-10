% SETUP_PROJECT Configure MATLAB paths for the PTC3441 coursework project.
% Run this script from any working directory before calling project functions,
% examples, or Simulink driver scripts.

project_root = fileparts(mfilename('fullpath'));

project_paths = {
    project_root
    fullfile(project_root, 'src', 'transforms')
    fullfile(project_root, 'src', 'kinematics')
    fullfile(project_root, 'src', 'jacobians')
    fullfile(project_root, 'src', 'trajectories')
    fullfile(project_root, 'examples', 'kinematics')
    fullfile(project_root, 'examples', 'trajectories')
    fullfile(project_root, 'simulations', 'ex4_pd_feedforward')
    fullfile(project_root, 'simulations', 'ex5_resolved_rate')
    fullfile(project_root, 'simulations', 'list2')
};

for k = 1:numel(project_paths)
    if ~isfolder(project_paths{k})
        error('PTC3441:MissingDirectory', ...
            'Expected project directory not found: %s', project_paths{k});
    end
    addpath(project_paths{k});
end

clear k project_paths
