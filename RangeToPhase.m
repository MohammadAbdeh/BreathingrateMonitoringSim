function [phases] = RangeToPhase(TimeOfFlights,lambda)
%returns the phase signal for a paticular range signal
%   Detailed explanation goes here

 [m,n] = size(TimeOfFlights);
 phases=zeros(1,n);
 
    for i=1:n
        
        phases(1,i) =(TimeOfFlights(1,i)/lambda);

    end
     
end

