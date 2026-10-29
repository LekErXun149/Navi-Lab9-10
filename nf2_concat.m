function F = nf2_concat(varargin)
%NF2_CONCAT  Append frame batches row-wise:  F = nf2_concat(F1, F2, ...)
F = varargin{1};
fn = fieldnames(F);
for k = 2:nargin
    G = varargin{k};
    for j = 1:numel(fn)
        F.(fn{j}) = [F.(fn{j}); G.(fn{j})];
    end
end
end
