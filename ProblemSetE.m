% Andrew Hwang
% MATH246
% ProblemSetE
%% Number 12
%% A
clear all
syms t y
hold on
ode1 = 'D2y = -2*Dy-2*y+sin(t)';
sol1 = dsolve(ode1, 'y(0)=0 Dy(0)=0', 't')
ezplot(sol1, [0, pi])

%% B
i1 = subs(sol1,pi)
i2 = subs(diff(sol1),pi)
sol2 = dsolve('D2y+2*Dy+2*y=0','y(pi)=0.3827','Dy(pi)=-0.1914')
ezplot(sol2,[pi 15])
ezplot(sol1, [0, pi])
axis([0 15 -.02 0.45])
title('D2y+2*Dy+2*y=sin(t) and D2y+2*Dy+2*y=0')
xlabel 't'; ylabel 'y';
hold off

%% C
syms t s Y; f = ['heaviside(t)*sin(t)+heaviside(t-pi)*(-sin(t))'];
eqn = sym(['D(D(y))(t)+2*D(y)(t)+2*y(t)=' f]);
lteqn = laplace(eqn, t ,s);
neweqn = subs(lteqn, {'laplace(y(t),t,s)','y(0)','D(y)(0)'},{Y,0,0});
ytrans = solve(neweqn, Y);
y = ilaplace(ytrans, s, t)
hold on
ezplot(y,[0 15])
ezplot(sol1,[0 pi])
ezplot(sol2,[pi 15])
axis([0 15 -.02 0.45])
title('D2y+2*Dy+2*y as Homogenous and Inhomogenous Equation')
xlabel 't'; ylabel 'y';
hold off

%% D
dsolve('D2y+2*Dy+2*y=0')
% as t approaches infinity, the solution approaches 0
% the solution will become 0 after a long time
%% Number 13
%% A
clear all
syms s t Y
f = ['heaviside(t - pi)'];
equation = sym(['D(D(y))(t) +3*D(y)(t)+2*y(t) = ' f]);
ltequation = laplace(equation, t, s);
newequation = subs(ltequation, {'laplace(y(t),t,s)', 'y(0)', 'D(y)(0)'},{Y, 1, 0});
ytrans = solve(newequation, Y);
y = ilaplace(ytrans, s ,t);
ezplot(y, [0 15])
title 'Solution', axis auto
xlabel 't'; ylabel 'y';
figure
f2 = heaviside(t - pi);
ezplot(f2, [0 15])
title 'Forcing Factor', axis auto
xlabel 't'; ylabel 'y';
axis([0 5 -1 1.5])

%% Number 14
%% B
syms s t Y
f = ['heaviside(t - 5) + heaviside(t - 10)*(-2)'];
equation = sym(['D(D(y))(t) + 6*D(y)(t) + 8*y(t) = ' f]);
ltequation = laplace(equation, t, s);
newequation = subs(ltequation, {'laplace(y(t),t,s)', 'y(0)', 'D(y)(0)'},{Y, 0, 2});
ytrans = solve(newequation, Y);
y = ilaplace(ytrans, s ,t);
ezplot(y, [0 15])
title 'Solution', axis auto
figure
ezplot(f, [0 15])
title 'Forcing Factor'
% The solution travels in the same path as the forcing function

%% Number 17
%% A
syms y t 
tic
eqn = sym('D(D(y))(t) + D(y)(t) + y(t) = (t+1)^3*exp(-t)*cos(t)')
dsolve(eqn)
toc
% It takes about .23 seconds to solve
%% B
syms s t Y
tic
eqn = sym('D(D(y))(t) + D(y)(t) + y(t) = (t+1)^3*exp(-t)*cos(t)');
lteqn = laplace(eqn,t,s);
neweqn = subs(lteqn, {'laplace(y(t),t,s)','y(0)','D(y)(0)'},{Y, 1, 0});
ytrans = solve(neweqn, Y);
y = ilaplace(ytrans, s, t)
toc
% It takes about .38 seconds to solve

%% C
ezplot(y, [0,15])
title 'Solution'
xlabel t;
ylabel y;
% At y(0) the value is 1 and the slope is 0