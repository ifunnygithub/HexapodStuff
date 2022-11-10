function F = CF(x)

configurations = [850,          -600,          0,           150,          550,         -500,           600,        -700,          400,        400,          600,        -400;
                  150,           0,            550,        -800,          550,         -500,           400,         400,          650,       -550,          150,        -350;
                  150,           400,          500,         150,          150,          350,           275,         370,          500,        600,          720,         800;
                  deg2rad(3),    deg2rad(-1),  deg2rad(3),  deg2rad(-4),  deg2rad(2),   deg2rad(-8),   deg2rad(5),  deg2rad(0),   deg2rad(0), deg2rad(-6),  deg2rad(-3), deg2rad(-0.5);
                  deg2rad(-5),   deg2rad(0.5), deg2rad(1),  deg2rad(2),   deg2rad(-3),  deg2rad(2),    deg2rad(-2), deg2rad(0.4), deg2rad(0), deg2rad(2),   deg2rad(-7), deg2rad(-5);
                  deg2rad(1),    deg2rad(7),   deg2rad(-3), deg2rad(0.1), deg2rad(0.5), deg2rad(-0.4), deg2rad(0),  deg2rad(3),   deg2rad(0), deg2rad(5.5), deg2rad(-2), deg2rad(10)];

LMinNominal = [604.8652;
               604.8652;
               604.8652;
               604.8652;
               604.8652;
               604.8652];

numConfig = 12;
measurements = zeros(6,numConfig);
for i = 1:numConfig
    measurements(:, i) = (RFK(IK(configurations(:,i)), configurations(:,i)));
end

j = 0;
for k = 1:numConfig

    l = IK(configurations(:,k));
    om = measurements(1:3,k); 
    a = measurements(4,k); 
    b = measurements(5,k); 
    c = measurements(6,k);
    
    R1 = [1, 0,       0;
          0, cos(a), -sin(a);
          0, sin(a),  cos(a)];
    R2 = [cos(b),  0, sin(b);
          0,       1, 0;
          -sin(b), 0, cos(b)];
    R3 = [cos(c), -sin(c), 0;
          sin(c),  cos(c), 0;
          0,        0,     1];
    Rm = R1*R2*R3;

    for i = 1:6
        j = j+1;
        
        F(j) = ((norm(om + (Rm*(x(i,1:3))') - (x(i,4:6))'))^2 - norm(x(i,7) + l(i,1) - LMinNominal(i,1))^2)^2;
    end
end