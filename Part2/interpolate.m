function [y_interpolated, x_interpolated] = interpolate(x, y, x_cond_derivative, y_cond_derivative)
    
    step = .1;
    
    if length(x) == 2
        [a, b, c, d] = coef_first_polyn_with_slope(x(1), y(1), x(2), y(2), 0);
        x_interpolated = x(1):step:x(2);
        y_interpolated = a*x_interpolated.^3 + b*x_interpolated.^2 + c*x_interpolated + d;
    else
        
        % Première courbe
        if x_cond_derivative == 0
            [a, b, c] = coef_first_polyn_without_slope(x(1), y(1), x(2), y(2));
            x_interpolated = x(1):step:x(2);
            y_interpolated = a*x_interpolated.^2 + b*x_interpolated + c;
            dx0 = 2*a*x(2) + b;
        else
            [a, b, c, d] = coef_first_polyn_with_slope(x(1), y(1), x(2), y(2), y_cond_derivative);
            x_interpolated = x(1):step:x(2);
            y_interpolated = a*x_interpolated.^3 + b*x_interpolated.^2 + c*x_interpolated + d;
            dx0 = 3*a*x(2)^2 + 2*b*x(2) + c;
        end
        
        % Courbes intermediaires
        for i = 2:length(x) - 2
            [a, b, c] = coef_middle_polyn(x(i), y(i), x(i+1), y(i+1), dx0);
            new_x_interpolated = (x(i)+step):step:x(i+1);
            new_y_interpolated = a*new_x_interpolated.^2 + b*new_x_interpolated + c;
            dx0 = 2*a*x(i+1) + b;

            x_interpolated = [x_interpolated, new_x_interpolated];
            y_interpolated = [y_interpolated, new_y_interpolated];
        end

        %Derniere courbe
        n = length(x) - 1;
        [a, b, c, d] = coef_last_polyn(x(n), y(n), x(n+1), y(n+1), dx0);
        new_x_interpolated = (x(n)+step):step:x(n+1);
        new_y_interpolated = a*new_x_interpolated.^3 + b*new_x_interpolated.^2 + c*new_x_interpolated + d;

        x_interpolated = [x_interpolated, new_x_interpolated];
        y_interpolated = [y_interpolated, new_y_interpolated];
    end

end


function [a, b, c, d] = coef_first_polyn_with_slope(x0, y0, x1, y1, s)
    A = [x0^3, x0^2, x0, 1; x1^3, x1^2, x1, 1; 3*x0^2, 2*x0, 1, 0; 3*x1^2, 2*x1, 1, 0];
    B = [y0;y1;0;s];
    C = num2cell(transpose(A\B));
    [a, b, c, d] = C{:};

end

function [a, b, c] = coef_first_polyn_without_slope(x0, y0, x1, y1)
    A = [x0^2, x0, 1; x1^2, x1, 1; 2*x0, 1, 0];
    B = [y0;y1;0];
    C = num2cell(transpose(A\B));
    [a, b, c] = C{:};
end

function [a, b, c] = coef_middle_polyn(x0, y0, x1, y1, dx0)
    A = [x0^2, x0, 1; x1^2, x1, 1; 2*x0, 1, 0];
    B = [y0;y1;dx0];
    C = num2cell(transpose(A\B));
    [a, b, c] = C{:};
end

function [a, b, c, d] = coef_last_polyn(x0, y0, x1, y1, dx0)
    A = [x0^3, x0^2, x0, 1;x1^3, x1^2, x1, 1;3*x0^2, 2*x0, 1, 0; 3*x1^2, 2*x1, 1, 0];
    B = [y0;y1;dx0;0];
    C = num2cell(transpose(A\B));
    [a, b, c, d] = C{:};
end