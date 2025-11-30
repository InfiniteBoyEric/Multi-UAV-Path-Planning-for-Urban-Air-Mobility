% Problem definition
function [Xmin, Xmax, dim, fobj] = fun_info()
    global model
    N = 1;
    d = model.n;
    dim = 3 * d * N;
    
    VarMax.r = 2 * norm(model.start - model.end) / d;
    VarMin.r = 0;
    
    AngleRange = pi/3;
    VarMin.psi = -AngleRange;
    VarMax.psi = AngleRange;
    
    dirVector = model.end - model.start;
    phi0 = atan2(dirVector(2), dirVector(1));
    VarMin.phi = phi0 - AngleRange;
    VarMax.phi = phi0 + AngleRange;
    
    Xmin = [VarMin.r*ones(1,d), VarMin.psi*ones(1,d), VarMin.phi*ones(1,d)];
    Xmax = [VarMax.r*ones(1,d), VarMax.psi*ones(1,d), VarMax.phi*ones(1,d)];
    
    fobj = @(x)Cost(x, 1);
end

function position = SphericalToCart(sol)
    global model
    xs = model.start(1); ys = model.start(2); zs = model.start(3);
    d = model.n;
    
    r = sol(1:d);
    psi = sol(1+d:2*d);
    phi = sol(1+2*d:3*d);
    
    x(1) = xs + r(1)*cos(psi(1))*sin(phi(1));
    x(1) = max(model.xmin, min(x(1), model.xmax));
    y(1) = ys + r(1)*cos(psi(1))*cos(phi(1));
    y(1) = max(model.ymin, min(y(1), model.ymax));
    z(1) = zs + r(1)*sin(psi(1));
    z(1) = max(model.zmin, min(z(1), model.zmax));
    
    for i = 2:model.n
        x(i) = x(i-1) + r(i)*cos(psi(i))*sin(phi(i));
        x(i) = max(model.xmin, min(x(i), model.xmax));
        y(i) = y(i-1) + r(i)*cos(psi(i))*cos(phi(i));
        y(i) = max(model.ymin, min(y(i), model.ymax));
        z(i) = z(i-1) + r(i)*sin(psi(i));
        z(i) = max(model.zmin, min(z(i), model.zmax));
    end
    position.x = x; position.y = y; position.z = z;
end

function o = Cost(X, chos)
    global model
    o = MyCost(X, chos);
end

function cost = MyCost(Xbest, Chos)
    global model
    sol = SphericalToCart(Xbest);
    J_inf = 1000;
    n = model.n;
    H = model.H;
    
    x = sol.x; y = sol.y; z = sol.z;
    
    xs = model.start(1); ys = model.start(2); zs = model.start(3);
    xf = model.end(1);   yf = model.end(2);   zf = model.end(3);
    
    x_all = [xs, x, xf]; y_all = [ys, y, yf]; z_all = [zs, z, zf];
    N = length(x_all);
    z_abs = zeros(1, N);
    [Hrmax, Hcmax] = size(H);
    
    for i = 1:N
        yid = max(1, min(round(y_all(i)), Hrmax));
        xid = max(1, min(round(x_all(i)), Hcmax));
        z_abs(i) = z_all(i) + H(yid, xid);
    end
    
    J1 = 0; % Path Length
    for i = 1:N-1
        diff = [x_all(i+1)-x_all(i); y_all(i+1)-y_all(i); z_abs(i+1)-z_abs(i)];
        J1 = J1 + norm(diff);
    end
    
    J2 = 0; % Threat Cost
    threats = model.threats;
    for i = 1:size(threats, 1)
        t = threats(i, :);
        for j = 1:N-1
            dist = DistP2S([t(1), t(2)], [x_all(j), y_all(j)], [x_all(j+1), y_all(j+1)]);
            if dist < t(4) + 10 % 安全距离
                J2 = J2 + J_inf;
            end
        end
    end
    
    J3 = 0; % Height Cost
    for i = 1:n, if z(i) < 0, J3 = J3 + J_inf; end; end
    
    J4 = 0; % Terrain Collision
    for i = 1:N
        yid = max(1, min(round(y_all(i)), Hrmax));
        xid = max(1, min(round(x_all(i)), Hcmax));
        if z_abs(i) < H(yid, xid) + 5 % Min Height
             J4 = J4 + J_inf;
        end
    end
    
    % Sum of costs
    if Chos == 1
        cost_weights = [1 5 1 10];
        cost = [J1 J2 J3 J4] * cost_weights.'; 
    else
        cost = [J1, J2, J3, J4];
    end
end

function dist = DistP2S(x, a, b)
    d_ab = norm(a - b);
    d_ax = norm(a - x); d_bx = norm(b - x);
    if d_ab ~= 0
        if dot(a-b, x-b) * dot(b-a, x-a) >= 0
            A = [b-a; x-a]; dist = abs(det(A)) / d_ab;
        else
            dist = min(d_ax, d_bx); 
        end
    else
        dist = d_ax; 
    end
end