%% Check if two values are close

function [output] = Close(a,b)

  a=Check(a);
  b=Check(b);
  
  if a*b>0  
    if abs((abs(a)-abs(b)))<1e-4
        output=1;
    else
        output=0;
    end
  elseif a*b==0
    if abs((abs(a)-abs(b)))<1e-4
        output=1;
    else
        output=0;
    end
      
  else
        output=0;          
  end

end

