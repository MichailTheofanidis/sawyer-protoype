function [output] = CheckZero(input)

    %% if no problem then continue
    output=input;

    %%Check if input is close to zero
    if abs(input)<1e-3
        output=0;
    end
       
end

