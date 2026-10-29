function v = uwb_prctile(x, p)
%UWB_PRCTILE  Percentile without the Statistics Toolbox (ignores NaN/Inf).
x = sort(x(isfinite(x)));
if isempty(x), v = NaN; return; end
v = x(max(1, min(numel(x), round(p / 100 * numel(x)))));
end
