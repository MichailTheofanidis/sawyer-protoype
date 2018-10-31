function [out] = CheckRange(in)

        out=in;

        if(in>deg2rad(180))
           out=(2*pi-in);
        elseif(in<deg2rad(-180))
           out=(2*pi+in); 
        end

end

