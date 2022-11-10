% Velocity Jacobian
function J = Jacobian(P)

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

L = zeros(3,6);
l = zeros(6,1);
n = zeros(3,6);
for leg = 1:6
    L(:,leg) = o + (R * s(:, leg)) - u(:, leg);
    l(leg, 1) = norm(L(:,leg),2);
    n(:,leg) = L (:, leg)/l(leg,1);
end

J = [n(:,1)' , cross(R * s(:,1), n(:,1))';
     n(:,2)' , cross(R * s(:,2), n(:,2))';
     n(:,3)' , cross(R * s(:,3), n(:,3))';
     n(:,4)' , cross(R * s(:,4), n(:,4))';
     n(:,5)' , cross(R * s(:,5), n(:,5))';
     n(:,6)' , cross(R * s(:,6), n(:,6))'];
end