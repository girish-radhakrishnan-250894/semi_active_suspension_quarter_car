function [satVal] = signal_saturator(lb,ub,val,phi,useMode)
%F04b_SATURATION_1 Creates a smooth saturation function and returns the saturated
%output
%   Uses exponential functions to create a smooth saturation function to
%   limit an input val b/w the min and max values
%
%   Input :
%   lb          Lower bound 
%   ub          Upper bound
%   val         Input value 
%   useMode     1 = Discontinuous Saturation
%               2 = Exponential Satuation
%   Output :
%   satVal      Saturated output 

k = phi;

% Switch Case used so that an arbitrary value can be used if saturation is
% not required
switch useMode
    
    % Discontinuous Saturation
    case 1                      
        satVal = min(max(val,lb),ub);
        
    % Tanh Saturation (tanh = sinh/cosh = (e^x - e^-x)/(e^x + e^-x)
    % [REF} Chu, Z., Xiang, X., Zhu, D., Luo, C., & Xie, D. (2020). Adaptive trajectory tracking control for remotely operated vehicles considering thruster dynamics and saturation constraints. ISA Transactions, 100(December), 28–37. https://doi.org/10.1016/j.isatra.2019.11.032
    case 2                      
        % If val is +VE
        if val > 0
            satVal = ub*((exp(k*val/ub) - exp(-k*val/ub))/(exp(k*val/ub) + exp(-k*val/ub)));
        % If val is -VE
        elseif val < 0
            satVal = lb*((exp(k*val/lb) - exp(-k*val/lb))/(exp(k*val/lb) + exp(-k*val/lb)));
        elseif val==0
            satVal=0;
        end
    
    % UNDER DEVELOPMENT Bipolar Sigmoid function.
    case 3
        satVal = ub*(2/(1+exp(-2*val/ub)) - 1);
    
    % Non-discontinuous signum function
    % [REF] Slotine, J., & Li, W. (1991). Applied Nonlinear Control. In
    % PRENTICE-HALL INTERNATIONAL SERIES IN CIVIL ENGINEERING AND
    % ENGINEERING MECHANICS. Prentice Hall. Pg 294
    case 4
        if abs(val) <= 1
            satVal = k*val;
        else
            satVal = sign(val);
        end
        
    % No saturation
    otherwise
        satVal = val;
end


end

