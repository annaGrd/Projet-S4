function p = isIn(A, B)
    lA = length(A(1, :));
    lB = length(B(1, :));
    
    p = false;

    if lA <= lB
        i = 1;
        j = lA;
        while j <= lB
            if isequal(A, B(:, i:j))
                p = true;
            end
            i = i + 1;
            j = j + 1;
        end
    end
end