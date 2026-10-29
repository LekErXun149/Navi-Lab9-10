function s = ternary_str(cond, a, b)
%TERNARY_STR  Return a if cond is true, otherwise b.
if cond, s = a; else, s = b; end
end
