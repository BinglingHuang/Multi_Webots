function function_handle = set2(option)

if nargin == 0
    option = [];
end

while length(option) == 0
    fprintf('Choose a model:\n\n');

    fprintf('   [1] curve_integration_field \n');
    fprintf('   [2] point_field \n');
    fprintf('   [3] Gaussian_field \n');

    fprintf('\n');
    option = input('Make a choice: ');
end

if option == 1
    function_handle = @curve_integration_field;
elseif option == 2
    function_handle = @point_field;
elseif option == 3
    function_handle = @Gaussian_field;
else
    fprintf('Choose one of the options above.\n');
    return
end

