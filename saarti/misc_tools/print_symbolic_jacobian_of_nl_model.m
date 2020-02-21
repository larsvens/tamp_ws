function print_symbolic_jacobian_of_nl_model(model)
% description function that symbolically computes the jacobian of a
% nonlinear system dx/dt = f(x,u)
% input: model as struct

n = length(model.x);
m = length(model.u);

sym A;
for i=1:n
    for j=1:n
        A(i,j) = diff(model.f{i},model.x{j}); % correct, checked with kin vehicle model
    end
end

sym B;
for i=1:n
    for j=1:m
        B(i,j) = diff(model.f{i},model.u{j}); % correct, checked with kin vehicle model
    end
end
   
% discretize model
syms I Ad Bd dt

I = sym(eye(n));
    
Ad = I+A*dt;
Bd = B*dt;

disp('CT model')    
disp('A = ')
disp(A);
disp('B = ')
disp(B);
disp('')
disp('DT model')
disp('Ad = ')
disp(Ad);
disp('Bd = ')
disp(Bd);
