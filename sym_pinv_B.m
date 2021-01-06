% B = [  0     0     0     0     -h/Ix  h/Ix  h/Ix  -h/Ix;
%        h/Iy -h/Iy  h/Iy -h/Iy   0     0     0      0;
%        w/Iz  w/Iz -w/Iz -w/Iz  -d/Iz -d/Iz  d/Iz   d/Iz;    
%        1/M   1/M   1/M   1/M    0     0     0      0;
%        0     0     0     0      1/M   1/M  -1/M   -1/M];

syms h w M Ix Iy Iz
Bu_sym = [  0     0     0     0     -h/Ix  h/Ix  h/Ix  -h/Ix;
       h/Iy -h/Iy  h/Iy -h/Iy   0     0     0      0;
       w/Iz  w/Iz -w/Iz -w/Iz  -d/Iz -d/Iz  d/Iz   d/Iz;    
       1/M   1/M   1/M   1/M    0     0     0      0;
       0     0     0     0      1/M   1/M  -1/M   -1/M];

pinv_Bu = pinv(Bu_sym)