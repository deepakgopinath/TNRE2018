function best_mode = compute_best_mode(EID_AR)
     %threshold within which two modes are considered to be disambiguate equally. 
    global cm;
    eq_modes = compute_equivalent_modes(EID_AR); %eq modes contain the indices of the cm that arae equivalent mode. 
    if ~isempty(eq_modes) %there are equivalent modes. 
        if length(eq_modes) == 1 &&  length(eq_modes{1}) == length(cm)
%             fprintf('All modes disambiguate equally. Dont change current mode \n');
            best_mode = -1;
        elseif length(eq_modes) >= 1
            best_mode = find(EID_AR == max(EID_AR));
            for i=1:length(eq_modes)
                if ~isempty(find(best_mode == eq_modes{i}, 1))
                    best_mode = eq_modes{i};
                    for j=1:length(eq_modes{i})
                        fprintf('Mode %d\n',eq_modes{i}(j));
                    end
                    break;
                end
            end
%             fprintf('Control mode %d disambiguates best\n', best_mode);
        end
    else
        best_mode = find(EID_AR == max(EID_AR));
%         fprintf('The Control mode %d disambiguates best - last\n', best_mode);
    end
end