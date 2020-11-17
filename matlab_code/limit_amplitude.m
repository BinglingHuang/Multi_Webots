function B = limit_amplitude(A, N)
    if A(1) <= 0
        A(1) = A(1) + N(1);
    end
    if A(1) > N(1)
        A(1) = A(1) - N(1);
    end
    if A(2) <= 0
        A(2) = A(2) + N(2);
    end
    if A(2) > N(2)
        A(2) = A(2) - N(2);
    end
    B = A;
end