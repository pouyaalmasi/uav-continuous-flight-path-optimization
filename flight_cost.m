function [total_cost,x,temp] = flight_cost(x)
    global Big_Value DW DH generated_nodes points

    n = length(x);
        
    p = zeros(n, 2);
    for i = 1:n
        p(i,:) = points(x(i),:);
    end
   
    length_cost = 0;
    for i = 1:n-1
        length_cost = length_cost + norm(p(i+1,:)-p(i,:));
    end
    
    
    % Constraint of Distance
    penalty_distance = 0;
    for i = 1:n-1
        if U (norm(p(i+1,:)-p(i,:)) - DH/2) == 1
            penalty_distance = 1;
        end
    end
   
    % Constraint of Duplicate
    penalty_duplicate = 0;
    for i = 1:n-1
        if  U (1.5 - (norm(p(i+1,:)-p(i,:)))) == 1 %norm(p(i+1,:)-p(i,:)) < 1
            penalty_duplicate = 1;
            break
        end
    end 

    % Constraint of Coverrage
    penalty_cover = 0;
    for i = 1:length(generated_nodes)
        flag = false;
        for j = 1:n
            if abs(p(j,1)-generated_nodes(i,1)) <= DW/2 && abs(p(j,2)-generated_nodes(i,2)) <= DH/2
                flag = true;
                break
            end
        end
        if flag == false
            penalty_cover = 1;
            break
        end
    end
    
    
    penalty = (penalty_distance + penalty_cover*3 + penalty_duplicate) * Big_Value;
    total_cost = (length_cost + penalty);
    temp = [penalty_distance,penalty_cover,penalty_duplicate];
end