function f = f_num(in1,in2,in3)
%F_NUM
%    F = F_NUM(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    10-Apr-2023 22:13:12

L = in3(:,1);
u1 = in2(1,:);
u2 = in2(2,:);
x3 = in1(3,:);
x4 = in1(4,:);
f = [u1.*cos(x3);u1.*sin(x3);(u1.*tan(x4))./L;u2];