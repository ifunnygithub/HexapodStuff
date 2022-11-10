%Inverse Kinematics of Parallel Robots with Prismatic Legs

function l = IK(P)

% Robot Parameters (Given)

s = [92.1597, 27.055, -119.2146, -119.2146, 27.055,   92.1597;
     84.4488, 122.037, 37.5882,  -37.5882, -122.037, -84.4488;
     0,       0,       0,         0,        0,        0];

u = [305.4001, -56.4357, -248.9644,  -248.9644, -56.4357,   305.4001;
     111.1565,  320.0625, 208.9060,  -208.9060, -320.0625, -111.1565;
     0,         0,        0,          0,         0,         0];

o=P(1:3,1); 
a=P(4); 
b=P(5); 
c=P(6);

% Calculating XYZ Rotation Matrix from Euler Angles

    R1 = [1, 0,       0;
          0, cos(a), -sin(a);
          0, sin(a),  cos(a)];

    R2 = [cos(b),  0, sin(b);
          0,       1, 0;
          -sin(b), 0, cos(b)];

    R3 = [cos(c), -sin(c), 0;
          sin(c),  cos(c), 0;
          0,        0,     1];

R = R1*R2*R3;

% Put it in the IK equation

l = zeros(6,1);

for i = 1:6
    l(i,1) = norm(o + R * s(:, i) - u (:, i), 2);
end



