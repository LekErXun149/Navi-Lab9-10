function G = nf2_slice(F, idx)
%NF2_SLICE  Select rows of a frame batch:  G = nf2_slice(F, idx)
fn = fieldnames(F);
for j = 1:numel(fn)
    G.(fn{j}) = F.(fn{j})(idx, :);
end
end
