function [output] = Check(input)

    %% if no problem then continue
    output=input;

    %%Check if input is close to zero
    if abs(input)<1e-1
        output=0;
    end
    
    %Check if input is close to 1
    if input>1
        output=1;
    end
    
    %Check if input is close to -1
    if input<-1
        output=-1;
    end
    
end

