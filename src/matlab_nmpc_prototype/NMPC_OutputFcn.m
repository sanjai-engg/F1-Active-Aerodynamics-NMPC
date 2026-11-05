function y = NMPC_OutputFcn(x,u)

ey     = x(4);
epsi   = x(5);
vx     = x(1);
ey_int = x(6);

y = [ey; epsi; vx; ey_int];

end