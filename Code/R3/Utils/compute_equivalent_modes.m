function eq_modes = compute_equivalent_modes(EID_AR)
    global num_modes;
    thresh = 10^-3;
    diff_p = squareform(pdist(EID_AR)/max(abs(EID_AR)));
    inds = find(diff_p < thresh);
    eq_modes = {}; %initialize with empty cell array
    if length(inds) > num_modes
        [i,j] = ind2sub([num_modes, num_modes], inds);
        eq_ind = i== j;
        i(eq_ind) = []; j(eq_ind) = [];
        eq_modes = [i,j]; eq_modes = num2cell(eq_modes, 2);
        flag = true;
        index  = 1; offset = 1;
        while flag
            if index + offset > length(eq_modes)
                    index = index + 1;
                    offset = 1;
            end
            if index >= length(eq_modes)
                break
            end
            if ~isempty(intersect(eq_modes{index}, eq_modes{index+offset}))
                eq_modes{index} = union(eq_modes{index}, eq_modes{index + offset});
                eq_modes(index + offset) = [];
            else
                offset = offset  + 1;
                if index + offset > length(eq_modes)
                    index = index + 1;
                    offset = 1;
                end
            end
        end
    end
end