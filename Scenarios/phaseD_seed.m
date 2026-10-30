function phaseD_seed(seed)
%PHASED_SEED  Set the random generator so every run can be repeated exactly.
if exist('rng', 'file') || exist('rng', 'builtin')
    rng(seed, 'twister');      % explicit type: also works if something switched MATLAB to the legacy generator
else
    rand('seed', seed); %#ok<RAND>
end
end
