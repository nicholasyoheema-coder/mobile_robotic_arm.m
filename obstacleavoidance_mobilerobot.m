%%\ MOBILE MANIPULATOR - FIVE BLOCK PICK AND PLACE WITH OBSTACLE AVOIDANCE
% Requires: Robotics System Toolbox
% Optional: set enableKeys = true below for keys 1-4 (3/4, top, side, low view).
% Otherwise rotate the view with the mouse using the figure toolbar.
%
% How it works
%   1. The floor is turned into a grid. Every obstacle (and every block that
%      is not being handled) is "inflated" by the robot radius, so the robot
%      centre can be treated as a point.
%   2. For each pick / place the planner finds the nearest standing point that
%      is 0.5 m from the target, has a clear arm line, and is reachable. It
%      uses Dijkstra on the grid, then shortens the path.
%   3. The robot drives along that path with its arm FOLDED over the chassis.
%   4. At the standing point the arm yaw turns (still folded), unfolds, lowers,
%      grips, lifts, folds again and yaw returns to zero. The robot never
%      has to turn 180 degrees with the arm out, which removes the loop.
clear; clc; close all;

%%\ 1. WORLD SETTINGS
W.bx     = [-1.10 1.30];        % allowed robot-centre range in X (m)
W.by     = [-0.90 0.90];        % allowed robot-centre range in Y (m)
W.res    = 0.05;                % planning grid size (m)
W.rRobot = 0.28;                % robot footprint radius (m)
W.safe   = 0.04;                % extra safety margin (m)
W.infl   = W.rRobot + W.safe;   % obstacle inflation
W.obs    = [-0.25 -0.10 0.07;   % obstacles: [x y radius]
            -0.25  0.40 0.07];
obsHeight = 0.30;

blockStart = [ 0.48 -0.45 0.04;
               0.68 -0.20 0.04;
               0.48  0.05 0.04;
               0.68  0.30 0.04;
               0.48  0.55 0.04];

blockGoal  = [-0.45 -0.50 0.04;
              -0.70 -0.25 0.04;
              -0.45  0.00 0.04;
              -0.70  0.25 0.04;
              -0.45  0.50 0.04];

blockColors = [0.90 0.15 0.12;
               0.10 0.45 0.90;
               0.15 0.70 0.30;
               0.95 0.65 0.10;
               0.60 0.25 0.80];

robotStart = [-0.95 -0.70];     % start position (x y)
standoff   = 0.50;              % robot centre to block distance when grasping
liftH      = 0.12;              % lift height above the block
foldQ      = [-1.5 2.5];        % folded arm pose [shoulder elbow]

%%\ 2. ROBOT MODEL (kinematics only; drawing is done separately)
robot = buildRobot();

%%\ 3. FIGURE AND CAMERA
fig = figure('Name','Mobile Manipulator - Obstacle Avoidance','Color','w', ... %[output:99c5e539]
             'Position',[80 60 1280 800]); %[output:99c5e539]
ax = axes('Parent',fig,'Position',[0.03 0.03 0.94 0.86]); %[output:99c5e539]
hold(ax,'on'); grid(ax,'on'); box(ax,'on'); %[output:99c5e539]
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)'); %[output:99c5e539]
daspect(ax,[1 1 1]); %[output:99c5e539]
xlim(ax,[-1.4 1.7]); ylim(ax,[-1.15 1.15]); zlim(ax,[0 0.8]); %[output:99c5e539]
view(ax,-37.5,28); %[output:99c5e539]
axis(ax,'vis3d'); %[output:99c5e539]
light(ax,'Position',[-2 -3 4],'Style','infinite'); %[output:99c5e539]
light(ax,'Position',[ 2  2 2],'Style','infinite'); %[output:99c5e539]
enableKeys = false;   % true = keys 1-4 change the view (desktop MATLAB only)
if enableKeys
    set(fig,'KeyPressFcn',@(~,e) changeView(ax,e.Key));
end
setappdata(fig,'minClear',inf);

%%\ 4. FLOOR, OBSTACLES, GOAL MARKERS, BLOCKS
[Xg,Yg] = meshgrid(-1.5:0.25:1.75,-1.25:0.25:1.25);
surf(ax,Xg,Yg,zeros(size(Xg)),'FaceColor',[0.90 0.92 0.95], ... %[output:99c5e539]
     'EdgeColor',[0.72 0.74 0.78],'FaceLighting','none'); %[output:99c5e539]

for r = 1:size(W.obs,1)
    drawObstacle(ax,W.obs(r,1:2),W.obs(r,3),obsHeight,W.obs(r,3)+W.infl); %[output:99c5e539]
end

blockG = makeBox([0.08 0.08 0.08]);
blockLocal = blockG.V;

for k = 1:5
    g = blockGoal(k,:);  s = 0.07;
    patch(ax,'Vertices',[g(1)-s g(2)-s 0.002; g(1)+s g(2)-s 0.002; ... %[output:99c5e539]
                         g(1)+s g(2)+s 0.002; g(1)-s g(2)+s 0.002], ... %[output:99c5e539]
          'Faces',[1 2 3 4],'FaceColor',blockColors(k,:),'FaceAlpha',0.35, ... %[output:99c5e539]
          'EdgeColor',blockColors(k,:),'LineWidth',2,'FaceLighting','none'); %[output:99c5e539]
    text(ax,g(1),g(2),0.12,sprintf('Place %d',k),'HorizontalAlignment','center', ...
         'FontSize',10,'FontWeight','bold');
end

hBlock = gobjects(5,1);
hLabel = gobjects(5,1);
for k = 1:5
    hBlock(k) = patch(ax,'Vertices',blockLocal + blockStart(k,:), ... %[output:99c5e539]
        'Faces',blockG.F,'FaceColor',blockColors(k,:), ... %[output:99c5e539]
        'EdgeColor',[0.1 0.1 0.1],'LineWidth',1, ... %[output:99c5e539]
        'AmbientStrength',0.55,'DiffuseStrength',0.7); %[output:99c5e539]
    hLabel(k) = text(ax,blockStart(k,1),blockStart(k,2),0.12,sprintf('Block %d',k), ...
        'HorizontalAlignment','center','FontSize',9);
end

% breadcrumb trail of the robot centre
trail = plot3(ax,NaN,NaN,NaN,'-','Color',[0.10 0.55 0.25],'LineWidth',1.5); %[output:99c5e539]

%%\ 5. DRAW THE ROBOT (coloured parts attached to the kinematic bodies)
rotY = axang2tform([1 0 0 pi/2]);        % turns a cylinder axis from Z to Y
P = struct('body',{},'T',{},'V',{},'h',{});

P(end+1) = mkPart(ax,'chassis',trvec2tform([0 0 0.10]),    makeBox([0.40 0.30 0.08]),[0.12 0.32 0.62]); %[output:99c5e539]
P(end+1) = mkPart(ax,'chassis',trvec2tform([0.215 0 0.09]),makeBox([0.03 0.30 0.05]),[0.98 0.80 0.10]); %[output:99c5e539]
P(end+1) = mkPart(ax,'chassis',trvec2tform([-0.13 0 0.165]),makeCyl(0.04,0.05),[0.10 0.70 0.70]); %[output:99c5e539]

% wheels: each wheel is its own rotating body, with spokes so the spin is visible
wheelNames = {'wheel_FL','wheel_FR','wheel_RL','wheel_RR'};
for k = 1:4
    P(end+1) = mkPart(ax,wheelNames{k},rotY,makeCyl(0.065,0.05),[0.08 0.08 0.08]); %[output:99c5e539]
    P(end+1) = mkPart(ax,wheelNames{k},trvec2tform([0 0 0]),makeBox([0.115 0.056 0.014]),[0.85 0.85 0.88]);
    P(end+1) = mkPart(ax,wheelNames{k},trvec2tform([0 0 0]),makeBox([0.014 0.056 0.115]),[0.85 0.85 0.88]);
    P(end+1) = mkPart(ax,wheelNames{k},rotY,makeCyl(0.018,0.060),[0.95 0.40 0.10]);
end

P(end+1) = mkPart(ax,'arm_base', trvec2tform([0 0 0.04]),    makeCyl(0.065,0.08),[0.25 0.27 0.30]); %[output:99c5e539]
P(end+1) = mkPart(ax,'upper_arm',trvec2tform([0.15 0 0]),    makeBox([0.30 0.06 0.06]),[0.95 0.45 0.10]); %[output:99c5e539]
P(end+1) = mkPart(ax,'upper_arm',rotY,                       makeCyl(0.05,0.08),[0.25 0.27 0.30]); %[output:99c5e539]
P(end+1) = mkPart(ax,'forearm',  trvec2tform([0.125 0 0]),   makeBox([0.25 0.05 0.05]),[0.98 0.72 0.10]); %[output:99c5e539]
P(end+1) = mkPart(ax,'forearm',  rotY,                       makeCyl(0.04,0.07),[0.25 0.27 0.30]); %[output:99c5e539]

P(end+1) = mkPart(ax,'gripper',trvec2tform([0.015 0 0]),     makeBox([0.03 0.15 0.04]),[0.30 0.30 0.33]); %[output:99c5e539]
P(end+1) = mkPart(ax,'gripper',trvec2tform([0.075  0.055 0]),makeBox([0.09 0.015 0.03]),[0.85 0.15 0.15]); %[output:99c5e539]
P(end+1) = mkPart(ax,'gripper',trvec2tform([0.075 -0.055 0]),makeBox([0.09 0.015 0.03]),[0.85 0.15 0.15]); %[output:99c5e539]

S.robot  = robot;
S.ax     = ax;
S.fig    = fig;
S.hBlock = hBlock;
S.hLabel = hLabel;
S.local  = blockLocal;
S.parts  = P;
S.W      = W;
S.trail  = trail;
S.fold   = foldQ;
S.liftH  = liftH;
S.dt     = 0.01;          % bigger = slower animation

S.wheelR = 0.065;         % wheel radius (m)
S.halfTrack = 0.17;       % half the distance between left and right wheels (m)

% config = [x y yaw | arm yaw, shoulder, elbow | wheel FL FR RL RR]
qStart = [robotStart 0, 0 foldQ, 0 0 0 0];
updateRobot(S,qStart); %[output:99c5e539]
drawnow;

%%\ 6. MISSION: FIVE PICK-AND-PLACE TASKS
cur  = qStart;
bpos = blockStart;        % current block positions (used as obstacles)

for k = 1:5 %[output:group:48a9a857]
    if ~isvalid(fig), break; end
    fprintf('Handling block %d of 5...\n',k); %[output:3ca1c861]
    others = bpos([1:k-1 k+1:5],1:2);

    % ---------- plan and drive to the pick position ----------
    setStatus(ax,k,'Planning path to block'); %[output:99c5e539]
    [Pp,~] = planLeg(W,cur(1:2),bpos(k,1:2),others,standoff);
    hPlan = plotPlan(ax,Pp);
    setStatus(ax,k,'Driving to block (avoiding obstacles)');
    cur = driveAlong(S,cur,Pp,0);
    delete(hPlan(isgraphics(hPlan)));

    % ---------- pick ----------
    setStatus(ax,k,'Picking block');
    cur = armTask(S,cur,bpos(k,:),k,'pick');

    % ---------- plan and drive to the place position ----------
    bposNow = bpos;  bposNow(k,:) = blockGoal(k,:);
    others = bpos([1:k-1 k+1:5],1:2);       % block k is in the gripper
    setStatus(ax,k,'Planning path to goal');
    [Pp,~] = planLeg(W,cur(1:2),blockGoal(k,1:2),others,standoff);
    hPlan = plotPlan(ax,Pp);
    setStatus(ax,k,'Carrying block to goal (avoiding obstacles)');
    cur = driveAlong(S,cur,Pp,k);
    delete(hPlan(isgraphics(hPlan)));

    % ---------- place ----------
    setStatus(ax,k,'Placing block');
    cur = armTask(S,cur,blockGoal(k,:),k,'place');
    bpos = bposNow;

    if isvalid(fig)
        text(ax,blockGoal(k,1),blockGoal(k,2),0.20,'DONE','Color',[0.05 0.45 0.10], ...
             'FontWeight','bold','FontSize',11,'HorizontalAlignment','center');
        drawnow;
    end
end %[output:group:48a9a857]

if isvalid(fig) %[output:group:201a376b]
    mc = getappdata(fig,'minClear');
    if mc > 0
        title(ax,'Pick-and-Place Complete: 5 of 5 blocks placed, no collisions','FontSize',15); %[output:99c5e539]
    else
        title(ax,'Pick-and-Place Complete (a collision was detected)','FontSize',15);
    end
    fprintf('Minimum robot-to-obstacle clearance: %.1f cm (must stay above 0)\n',100*mc); %[output:4c577f48]
end %[output:group:201a376b]
fprintf('Five-block animation finished.\n'); %[output:41c405f3]

%%\ 7. LOCAL FUNCTIONS

% ---------------- planning ----------------

function ok = freePt(W,p,others)
    % true if the robot centre may stand at p
    ok = p(1) >= W.bx(1) && p(1) <= W.bx(2) && p(2) >= W.by(1) && p(2) <= W.by(2);
    if ~ok, return; end
    if ~isempty(W.obs)
        d = hypot(W.obs(:,1)-p(1),W.obs(:,2)-p(2)) - W.obs(:,3);
        if any(d < W.infl), ok = false; return; end
    end
    if ~isempty(others)
        d = hypot(others(:,1)-p(1),others(:,2)-p(2));
        if any(d < W.rRobot + 0.057 + 0.02), ok = false; end
    end
end

function d = segDist(a,b,p)
    % distance from point p to the segment a-b
    ab = b - a;
    t  = min(max(dot(p-a,ab)/dot(ab,ab),0),1);
    d  = norm(a + t*ab - p);
end

function ok = lineClear(W,a,b,others)
    n = ceil(norm(b-a)/0.02) + 1;
    ok = true;
    for t = linspace(0,1,n)
        if ~freePt(W,a + t*(b-a),others), ok = false; return; end
    end
end

function [Pout,stand] = planLeg(W,start,target,others,standoff)
    % Find the best standing point around "target" and a collision-free path to it
    res = W.res;
    nx = round((W.bx(2)-W.bx(1))/res) + 1;
    ny = round((W.by(2)-W.by(1))/res) + 1;
    xs = W.bx(1) + (0:nx-1)*res;
    ys = W.by(1) + (0:ny-1)*res;

    occ = false(nx,ny);
    for i = 1:nx
        for j = 1:ny
            occ(i,j) = ~freePt(W,[xs(i) ys(j)],others);
        end
    end

    si = min(max(round((start(1)-W.bx(1))/res)+1,1),nx);
    sj = min(max(round((start(2)-W.by(1))/res)+1,1),ny);
    occ(si,sj) = false;

    dist = inf(nx,ny);  par = zeros(nx,ny);  done = false(nx,ny);
    dist(si,sj) = 0;
    nbr = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];
    while true
        tmp = dist;  tmp(done) = inf;
        [d,u] = min(tmp(:));
        if isinf(d), break; end
        done(u) = true;
        [i,j] = ind2sub([nx ny],u);
        for m = 1:8
            a = i + nbr(m,1);  b = j + nbr(m,2);
            if a < 1 || b < 1 || a > nx || b > ny, continue; end
            if occ(a,b) || done(a,b), continue; end
            if nbr(m,1) ~= 0 && nbr(m,2) ~= 0 && (occ(a,j) || occ(i,b)), continue; end
            nd = d + res*hypot(nbr(m,1),nbr(m,2));
            if nd < dist(a,b), dist(a,b) = nd;  par(a,b) = u; end
        end
    end

    % choose the cheapest valid standing point on a ring around the target
    bestD = inf;  stand = [];  bi = 0;  bj = 0;
    for ang = 0:pi/12:2*pi-pi/12
        c = target + standoff*[cos(ang) sin(ang)];
        if ~freePt(W,c,others), continue; end
        clearArm = true;                 % the arm line must not touch anything
        for r = 1:size(W.obs,1)
            if segDist(c,target,W.obs(r,1:2)) < W.obs(r,3) + 0.09, clearArm = false; end
        end
        for r = 1:size(others,1)
            if segDist(c,target,others(r,:)) < 0.18, clearArm = false; end
        end
        if ~clearArm, continue; end
        i = round((c(1)-W.bx(1))/res) + 1;
        j = round((c(2)-W.by(1))/res) + 1;
        if i < 1 || j < 1 || i > nx || j > ny, continue; end
        if occ(i,j) || isinf(dist(i,j)), continue; end
        if dist(i,j) < bestD
            bestD = dist(i,j);  stand = c;  bi = i;  bj = j;
        end
    end
    if isempty(stand)
        error('No collision-free standing position found near (%.2f, %.2f).',target(1),target(2));
    end

    % walk back through the parents to get the grid path
    idx = sub2ind([nx ny],bi,bj);
    su  = sub2ind([nx ny],si,sj);
    pts = idx;
    while idx ~= su
        idx = par(idx);
        pts(end+1) = idx; %#ok<AGROW>
    end
    pts = fliplr(pts);
    [pI,pJ] = ind2sub([nx ny],pts);
    Q = [xs(pI)' ys(pJ)'];
    Q = [start; Q(2:end-1,:); stand];

    % shorten the path with line-of-sight shortcuts
    Pout = Q(1,:);  k = 1;
    while k < size(Q,1)
        m = size(Q,1);
        while m > k+1 && ~lineClear(W,Q(k,:),Q(m,:),others)
            m = m - 1;
        end
        Pout(end+1,:) = Q(m,:); %#ok<AGROW>
        k = m;
    end
end

function h = plotPlan(ax,Pp)
    h = plot3(ax,Pp(:,1),Pp(:,2),0.012*ones(size(Pp,1),1),'--o', ...
              'Color',[0.00 0.45 0.85],'LineWidth',1.6,'MarkerSize',5);
end

% ---------------- arm kinematics ----------------

function [q,ok] = armIK(p)
    % Analytic IK (elbow-up) for the 3-joint arm. p = target in the chassis frame.
    L1 = 0.30;  L2 = 0.33;  H0 = 0.22;     % upper arm, forearm + tool, shoulder height
    q1 = atan2(p(2),p(1));
    r  = hypot(p(1),p(2));
    z  = p(3) - H0;
    c  = (r^2 + z^2 - L1^2 - L2^2)/(2*L1*L2);
    ok = abs(c) <= 1;
    c  = min(max(c,-1),1);
    b  = -acos(c);
    a  = atan2(z,r) - atan2(L2*sin(b),L1 + L2*cos(b));
    q  = [q1 -a -b];
    ok = ok && q(2) >= -1.8 && q(2) <= 0.8 && q(3) >= 0 && q(3) <= 2.6;
end

function pl = toLocal(base,p)
    c = cos(base(3));  s = sin(base(3));
    d = p - [base(1) base(2) 0];
    pl = [c*d(1)+s*d(2), -s*d(1)+c*d(2), d(3)];
end

function a = wrapPi(a)
    a = atan2(sin(a),cos(a));
end

% ---------------- motion ----------------

function qEnd = animate(S,qA,qB,N,carry,ease)
    % Move from qA to qB in N frames. carry = index of the block held (0 = none)
    qEnd = qB;
    N = max(N,2);
    moving = any(abs(qB(1:2)-qA(1:2)) > 1e-9);
    for j = 1:N
        if ~isgraphics(S.ax), return; end
        t = j/N;
        if ease, b = t*t*(3-2*t); else, b = t; end
        q = qA + b*(qB-qA);

        updateRobot(S,q);
        if carry > 0
            T = getTransform(S.robot,q,'tool');
            set(S.hBlock(carry),'Vertices',S.local + T(1:3,4)');
        end
        if moving
            set(S.trail,'XData',[get(S.trail,'XData') q(1)], ...
                        'YData',[get(S.trail,'YData') q(2)], ...
                        'ZData',[get(S.trail,'ZData') 0.004]);
            O = S.W.obs;
            if ~isempty(O)
                c = min(hypot(O(:,1)-q(1),O(:,2)-q(2)) - O(:,3)) - S.W.rRobot;
                setappdata(S.fig,'minClear',min(getappdata(S.fig,'minClear'),c));
            end
        end
        drawnow;
        pause(S.dt);
    end
end

function cur = driveAlong(S,cur,Pp,carry)
    % Follow the waypoints: turn on the spot, then drive straight. Arm stays folded.
    for i = 2:size(Pp,1)
        seg = Pp(i,:) - Pp(i-1,:);
        L = norm(seg);
        if L < 1e-3, continue; end
        dyaw = wrapPi(atan2(seg(2),seg(1)) - cur(3));
        if abs(dyaw) > 1e-3
            % turning on the spot: left wheels spin backward, right wheels forward
            qB = cur;  qB(3) = cur(3) + dyaw;
            dw = S.halfTrack*dyaw/S.wheelR;
            qB(7:10) = cur(7:10) + [-dw dw -dw dw];
            cur = animate(S,cur,qB,ceil(abs(dyaw)/0.08),carry,false);
        end
        % driving straight: all four wheels roll forward by L / radius
        qB = cur;  qB(1:2) = Pp(i,:);
        qB(7:10) = cur(7:10) + L/S.wheelR;
        cur = animate(S,cur,qB,ceil(L/0.03),carry,false);
    end
end

function cur = armTask(S,cur,tgt,k,mode)
    % Pick or place block k at tgt. The base is parked, only the arm moves.
    if ~isgraphics(S.ax), return; end
    base = cur(1:3);
    [qAb,ok1] = armIK(toLocal(base,tgt + [0 0 S.liftH]));
    [qDn,ok2] = armIK(toLocal(base,tgt));
    if ~(ok1 && ok2)
        warning('Block %d: target may be out of reach.',k);
    end
    bearing = qAb(1);
    carryNow = 0;
    if strcmp(mode,'place'), carryNow = k; end

    nYaw = ceil(abs(bearing)/0.05) + 10;
    wh     = cur(7:10);                  % wheels stay where they are
    foldA  = [base bearing S.fold wh];
    above  = [base qAb wh];
    down   = [base qDn wh];
    foldB  = [base 0 S.fold wh];

    cur = animate(S,cur,foldA,nYaw,carryNow,true);      % 1. turn arm (still folded)
    cur = animate(S,cur,above,40,carryNow,true);        % 2. unfold above the target
    cur = animate(S,cur,down,30,carryNow,true);         % 3. lower

    if strcmp(mode,'pick')                              % 4. grip or release
        carryAfter = k;
        if isgraphics(S.hLabel(k)), delete(S.hLabel(k)); end
    else
        carryAfter = 0;
        if isgraphics(S.hBlock(k))
            set(S.hBlock(k),'Vertices',S.local + tgt);
        end
    end

    cur = animate(S,cur,above,30,carryAfter,true);      % 5. lift
    cur = animate(S,cur,foldA,40,carryAfter,true);      % 6. fold again
    cur = animate(S,cur,foldB,nYaw,carryAfter,true);    % 7. arm yaw back to front
end

% ---------------- robot model and drawing ----------------

function setOffset(joint,v)
    % Pure translation [x y z] of the joint frame relative to its parent
    try
        setFixedTransform(joint,trvec2tform(v));
    catch
        setFixedTransform(joint,[v(1) 0 v(3) 0],'mdh');
    end
end

function robot = buildRobot()
    robot = rigidBodyTree('DataFormat','row','MaxNumBodies',16);

    bx = rigidBody('base_x');
    jx = rigidBodyJoint('base_x_joint','prismatic');
    jx.JointAxis = [1 0 0];  jx.PositionLimits = [-5 5];
    bx.Joint = jx;
    addBody(robot,bx,'base');

    by = rigidBody('base_y');
    jy = rigidBodyJoint('base_y_joint','prismatic');
    jy.JointAxis = [0 1 0];  jy.PositionLimits = [-5 5];
    by.Joint = jy;
    addBody(robot,by,'base_x');

    chassis = rigidBody('chassis');
    jc = rigidBodyJoint('base_yaw_joint','revolute');
    jc.JointAxis = [0 0 1];  jc.PositionLimits = [-200 200];
    chassis.Joint = jc;
    addBody(robot,chassis,'base_y');

    armBase = rigidBody('arm_base');
    j1 = rigidBodyJoint('base_yaw','revolute');
    j1.JointAxis = [0 0 1];
    j1.PositionLimits = [-pi pi];
    setOffset(j1,[0 0 0.14]);
    armBase.Joint = j1;
    addBody(robot,armBase,'chassis');

    upperArm = rigidBody('upper_arm');
    j2 = rigidBodyJoint('shoulder_pitch','revolute');
    j2.JointAxis = [0 1 0];
    j2.PositionLimits = [-1.8 0.8];
    setOffset(j2,[0 0 0.08]);
    upperArm.Joint = j2;
    addBody(robot,upperArm,'arm_base');

    forearm = rigidBody('forearm');
    j3 = rigidBodyJoint('elbow_pitch','revolute');
    j3.JointAxis = [0 1 0];
    j3.PositionLimits = [0 2.6];
    setOffset(j3,[0.30 0 0]);
    forearm.Joint = j3;
    addBody(robot,forearm,'upper_arm');

    gripper = rigidBody('gripper');
    jg = rigidBodyJoint('gripper_fixed','fixed');
    setOffset(jg,[0.25 0 0]);
    gripper.Joint = jg;
    addBody(robot,gripper,'forearm');

    tool = rigidBody('tool');
    jt = rigidBodyJoint('tool_fixed','fixed');
    setOffset(jt,[0.08 0 0]);
    tool.Joint = jt;
    addBody(robot,tool,'gripper');

    % Four spinning wheels (added last so they are config entries 7..10)
    wheelPos = [ 0.13  0.17 0.065;     % FL
                 0.13 -0.17 0.065;     % FR
                -0.13  0.17 0.065;     % RL
                -0.13 -0.17 0.065];    % RR
    wheelNames = {'wheel_FL','wheel_FR','wheel_RL','wheel_RR'};
    for k = 1:4
        w  = rigidBody(wheelNames{k});
        jw = rigidBodyJoint([wheelNames{k} '_joint'],'revolute');
        jw.JointAxis = [0 1 0];
        jw.PositionLimits = [-1e4 1e4];
        setOffset(jw,wheelPos(k,:));
        w.Joint = jw;
        addBody(robot,w,'chassis');
    end
end

function g = makeBox(sz)
    h = sz/2;
    g.V = [-h(1) -h(2) -h(3);  h(1) -h(2) -h(3);  h(1)  h(2) -h(3); -h(1)  h(2) -h(3);
           -h(1) -h(2)  h(3);  h(1) -h(2)  h(3);  h(1)  h(2)  h(3); -h(1)  h(2)  h(3)];
    g.F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    g.edge = [0.08 0.08 0.08];
end

function g = makeCyl(r,h)
    n  = 28;
    th = linspace(0,2*pi,n+1);  th(end) = [];
    c  = [r*cos(th)' r*sin(th)'];
    g.V = [c -h/2*ones(n,1); c h/2*ones(n,1); 0 0 -h/2; 0 0 h/2];
    F = zeros(4*n,3);
    for i = 1:n
        j = mod(i,n)+1;
        F(4*i-3,:) = [i j n+j];
        F(4*i-2,:) = [i n+j n+i];
        F(4*i-1,:) = [2*n+1 j i];
        F(4*i,  :) = [2*n+2 n+i n+j];
    end
    g.F = F;
    g.edge = 'none';
end

function p = mkPart(ax,body,T,g,color)
    p.body = body;
    p.T    = T;
    p.V    = g.V;
    p.h    = patch(ax,'Vertices',g.V,'Faces',g.F,'FaceColor',color, ...
                   'EdgeColor',g.edge,'LineWidth',0.5, ...
                   'AmbientStrength',0.55,'DiffuseStrength',0.7,'SpecularStrength',0.1);
end

function updateRobot(S,q)
    names = {'chassis','arm_base','upper_arm','forearm','gripper', ...
             'wheel_FL','wheel_FR','wheel_RL','wheel_RR'};
    Tb = struct();
    for i = 1:numel(names)
        Tb.(names{i}) = getTransform(S.robot,q,names{i});
    end
    for i = 1:numel(S.parts)
        T = Tb.(S.parts(i).body) * S.parts(i).T;
        V = (T(1:3,1:3)*S.parts(i).V')' + T(1:3,4)';
        set(S.parts(i).h,'Vertices',V);
    end
end

function drawObstacle(ax,c,r,h,zoneR)
    red = [0.85 0.18 0.12];
    [X,Y,Z] = cylinder(r,32);
    surf(ax,X+c(1),Y+c(2),Z*h,'FaceColor',red,'EdgeColor','none');
    [X2,Y2,Z2] = cylinder(r*1.02,32);
    surf(ax,X2+c(1),Y2+c(2),0.10+Z2*0.06,'FaceColor',[1 1 1],'EdgeColor','none');
    th = linspace(0,2*pi,40);
    fill3(ax,c(1)+r*cos(th),c(2)+r*sin(th),h*ones(size(th)),red);
    plot3(ax,c(1)+zoneR*cos(th),c(2)+zoneR*sin(th),0.004*ones(size(th)),'--', ...
          'Color',red,'LineWidth',1);
    text(ax,c(1),c(2),h+0.06,'Obstacle','HorizontalAlignment','center', ...
         'FontSize',9,'Color',red,'FontWeight','bold');
end

function setStatus(ax,k,msg)
    if isgraphics(ax)
        title(ax,{sprintf('Block %d of 5',k),msg},'FontSize',15);
        drawnow;
    end
end

function changeView(ax,key)
    if ~isgraphics(ax), return; end
    switch key
        case '1', view(ax,-37.5,28);
        case '2', view(ax,0,90);
        case '3', view(ax,0,0);
        case '4', view(ax,-60,15);
    end
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:3ca1c861]
%   data: {"dataType":"text","outputData":{"text":"Handling block 1 of 5...\nHandling block 2 of 5...\nHandling block 3 of 5...\nHandling block 4 of 5...\nHandling block 5 of 5...\n","truncated":false}}
%---
%[output:99c5e539]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAApAAAAGaCAYAAAC4xnOrAAAQAElEQVR4AeydB5wdVfXHz9u+6SGAAQMJIfQSeheRIl2KIghKkya9\/lGkdxAQkaJ0lKYoRelFmvTekRIChJZAEpLsJtv\/7zu75+19szOvl9n3Tj75vdvPPffMzL2\/OXdmtqbH\/pkFzAJmAbOAWcAsYBYwC5gFsrBAjVTRv48++khuvPFGOfHEE+XAAw+UXXbZRfbcc085+uij5eyzz5a\/\/OUvQp0qMkkkh\/raa6\/JSy+9lATyIqmsKTVoLdDd3S3vvfee3HnnnXLNNdfIH\/7wB7n44ovlqquukunTpw+CcQWr+PXXX8ubb745AHPmzAlukEMu86T\/GiU9f\/78HKRFs0l8HZWpU6cOwMyZM6OpcAS1Yt7mvHBBXgRVHfQqzZs3b8A1zzzwxRdfFG1sFU8gmQQefvhh2WabbWSzzTaTk08+WW6++WZ56KGHPILy3\/\/+11tArr76ajn11FO9OtS79tprpaOjo2iGN8HBFujs7JSddtrJI\/cQfAV5XV1dwY0styAWeOWVV+Qf\/\/hHzoCMFUSREgh5\/vnnZcstt5StttrKu4E866yzPAJ5ySWXyDnnnCPvvPNOzlp89tlnOdsQ+z\/99NM5903DK6+8Un70ox8NwL333ktxQfDb3\/52wDXKtcqCVZAOIiDk\/fffl0033XQAjjvuuAhoF30VbC4v7TF68MEHB1zzzANnnHFG0RSpaALJnfgee+whBxxwgLz77rsZG5G76zPPPNMjndw5hTW0\/Mq1QFtbm7AQ+3H77bdX7KD\/+te\/yv\/93\/\/lDCawwWAciMFuu+0mH374YVHU5aY0HzteeumlRdHLhGZnAZwPQS3sRjbIKpZXjRaoWAKJNwT2\/eyzz+Z8XFlguKt+8sknc5ZhDQenBdjuO\/fcc8WPiy++eHAOKAOtuXHKoNqgr\/LnP\/+5qGOYNm1aUeWb8IJYwISYBcwCeVqgIgnkrFmzZO+995Yvv\/wyT\/P0Nt9rr72E7b3elP2aBSrTAh988EFlDswZFY+lFNuL\/Mknnzg9WtQsYBYwC1SmBSqOQLK9cNRRR2VMHocOHZrRkT3kkEOEbc2MKlul9BawGpGyAA9gt7S0REqnYijzzTffhIpdaqml5KCDDpLDDz9clllmmdB66Qo+\/vjjdFWs3CxgFjALDHoLVByBvOuuu+SJJ55IeWB4Weaee+7x3sB84403vJdpeAN76aWXDm2HN\/Nvf\/tbaLkVmAUGswXC3tRbe+21JVOMHz8+8ibguegwJZk7eHbxyCOPlMUXXzysWtp8Hn3xV1p11VUztuPkyZP9zS1tFoiUBUwZswAWSCKQfIKBt+jAjBkzJOwhYhpmCmTwViIPlj\/33HMyZcoUwduRafts6vFZjiuuuCK0Cd5GFgk+3bPCCitIXV2dV3f06NGy0UYbyd133y2\/+MUvvLygn9\/97neCjYLK\/HlslUFOH330UbnjjjsE8slLBi+88IKwxe6vn28aO\/PpEY4d9qb\/VDKxFYspb5t++umnBTnWbn\/t7e3eJ5FYTHkbzy0rRZw+eVni8ccf9x4\/YIzoVIq+g\/rg0x+c\/1wH\/\/vf\/4pyDgT1m2ne559\/PqAqpIfzNlNsv\/32A2QUIoNzuVDXUtjcA2kbNmxY3uoiP8iTyw1qpnY8\/vjj89YjEwHsqLDdzhyQ6byWidxM6xTyuKbqk\/mWR5B4Ln7u3Lmpqpa1jDmcdRdd77\/\/fuH5fZ5L5pwqp2KsFawp5TpPGLvaBR2IoxP5+QKbMzZelsXWhboOkMOcxfsTrMk4oAqhM2sYNkAmdsh3\/Pm2r7n11lvliCOO8D5fs9JKKyVeA1933XUFj9wWW2whf\/rTn+Tbb7\/NuC8m0Isvvlh22203YRH63ve+531v8Wc\/+5lsvvnmXh7PKEKsWOgzFpymImQBwhJW7d\/\/\/resssoqYcXS2NgoJ510kqdfUCXGle5tbr5xdeihh8oaa6whO+ywg\/zyl7+UY445Rn7zm99422O77rqrrLnmmt4nMPhkR7rxc3LzzUo\/OG7oyEl\/7LHHesdqvfXW844f9l5uueW8t8+ZjKin4AF\/PlnCgrnOOuvItttuK9\/\/\/ve9MfOs53\/+8x+tGhied955Qj0X+++\/v1eXixGSvN1228nyyy\/vnVOcP8suu6w3Xl5IKeZJj634xif90yefadlnn33kxz\/+sTdGdMJWkDhPYd8PC9rvf\/97ueCCC7xPuviKvST2o1wxdepUL9\/\/gy1uu+02wTZ48NZaay3h\/OfmZeutt\/bOAfTkLW8WOH\/71OnCl2I7v1Ts5c8rZbpQ1xLHSI8XRC5oDDz\/qXUImfyD6qXLCyLiY8eOlREjRqRrWrJybmp333134SZ6k0028eYA5n4+dcbnecKuj0IpWKjjGqYPJJE32fXaY75lDthqq62EeY81gJcjWQ+45sPkZJvPGon9mGOCAJkIkok9mBdYb1l30fXggw8WjhGflGMN3WCDDbyvQhRy\/mT87jyucdYU9ISkYMNJkyYJawprhZ4nv\/71r71vDlKvGOAzW\/TB59s4XmoXdCCOTsyffG4PcpaNDoyPNRD7YlvGxvmArRkf8zXrLc6ebOSis8ulWP+xKS\/zcvzQmzUwFUcJ6g+nG5\/iwRbMydgAmchjfWWdCWpXiryaE044QTiRMGpQhwz2\/PPPl9VXX10ghSyMQfU0D8LCAsn31DAopEvL3JBtZojVT37yE8Ej45blGmdiDGuLZ3HChAlhxYl8vJI8A5XI8EXCFhbu5vWE53trYeNWcdzxsF3GiYCnTPP9IV5Cvlnpx6uvvir33XefR9LCXgrg+5dMRkwEyIUccmz4aLJfP9JMcPvtt5+wxR92t\/Tiiy8K9Vw88sgj3vOhe++9t0eS3377bbpLAuOFLLFI0TapsAAJyDgTAd\/4DOpfu8BWTNZ8PN5P3Lhr\/OMf\/yiXX365961QbeMPKVdwcfvL2Q6GuOJJwjZhz92hJxPKD3\/4Q4F4++X40\/\/6178Ct0GZSP11s03jofW3mThxoj+rJOlCX0scIz1eeHaCBsH5r3UIWdSD6qXLCyLiELV07UpVzoLLTS3eLX+f3Bzfcsst3s0+BCxsDvC3yzRd6OMa1C\/rDzeOF110kYRdexxr5iMWe26ema+DZGWTh2foV7\/6lWA\/5hg\/OJ9WXnnlJJHsSkA0IQbpSDskibkC0sD8nSQoxwRrO3OxH+jFR\/VZm7ChXzznyd\/\/\/nfPWYEuhTxPOEdwUuB8og\/sxvHy60Ca+ZPP7UHOLr74YrLSgnUCoojeXANBspmvWW9x9nBs8PilEsy6wR8iQedUXAq5rIFbxJ1y7JSmcx7RJ2s4TrfrrrtOsAV5LjiGrDNwtEIeB7ePVPGkLexUFSmDFDKxEA8CZRAWvDRB5UF5r7\/+ukBq0h2koLb+vKeeesqflUhDbhKJNBGICN487lb9qK2tHdCaE4hvTXLCDyhMk8HFuPPOO2f94WLujnixJ414r5i7WS4WyGHQBeNVcn7w0lx\/\/fVOTvooZJiJKF1NLiLuypig0tXNpFxtT\/+Z1Nc6d955p\/z0pz8VPtejeYUIuYmBEHKDlKk8bMLLG0xuqdrMnj1bqOtHIV7aCCKQ3OW7+mBrN12MOH2U+loq5DiCPJB4w90+IBuZLB5um0LE8bqw4GYiCwIGIeJ4ZFI\/XR3kFPu4QvxZfyBb6fTRcuqyY8T8qHnZhjhVcMSEycADfcMNNwiPSqlsjj\/9QjQ1L9OQ45hLu0zlX3\/99aE7MH4Z6MLNMue0vyzbNM4Sjl8un9mCm0A8w\/rkMQDsne06AWnD4cDjYUGycbxxA88fIgkqD8vjcTgcTpw7QXXIZ43kmgkq9+exS5zt2PwycklnRSDpgC0+WDFxF3gbmXTcvGzi3BXn456nLQczqM+NN95YllpqqaCiwLz6+nrPA8UWux98mNzfiIkrE\/Lkb6dpSB0ndzbbKWFjVZluCKHHXe\/mpYtzh5SNPtx5pZPplnNxqGfUzc82zp1k0PmYiRzu3vA28+Z+JvXT1cGjiTyOZ7q6QeVMADwnGVRW7LypU6cO6IJnhpmUOXfY1mF7h+0k0iwceHEGNMozowTXUp4apm7Oteav8d3vflfw\/LEdyFzENhSkcscddxS2PLl2SuE9yPa8xAvD9eUfTy7pYh\/Xxx9\/3Hv0JBfdaMOxCbqJoiwdLrvsMgkjdFxDN95444CXsmgTRjjT9Uc5nstCOF2Q5QcvmPrzUqVZ+\/C8pqqTSRnEEa9iJnWD6tA+zKacf7l6mrlhZ0sb76i\/X67fbNZitz3nDMTPzdP4Y489ljGJ1zblCLMmkCh5+umnEyTA3dSpp56aSAdFmDTHjBkTVOTlcSfI9qGXyOEn6M5fxay44ooaLXjIlhWTQSrBjJu70FR1OAlz8WCmkplPGRdN2HZfPnLdtjzX4aazjWP7VDctTN6cd4RhsvEUPvDAA14xNw7caIwbN044Zl5mwA\/lipEjRyZqsI2SapHWNokGARE8GYUitAHiQ7OCvJg8s6mTMucDjRkfkzTEgueG0BfiTFm+4HgW41rC88N5AFJdh5QrFl100ZyGE0RC+POpeP7YDnQJJrsvLLxspWJrXvLIqdMiNuL482xfPl0U67iqTqw\/2FfTuYSc12GLeSp5d8Z3MlLNQZBH\/6Mg7HpwAx0kl7UKzxR\/TpPzgnRQPfI4nwijAJ4bzmcewMPHnBI2FublH\/zgB8Jz+2F1yOeaInTB+ZfLsXVl4In0P2bEI2TMhW49N84awnOtbp4\/jicS\/dx8vI8XXnihmxXZeIJAsshCAjkp33rrLeEljbDJlknQHTRvF7IVGzRKXLDcKcH+2Xa97bbbQr2BXGyu3CB5YXlclGFlYeMIq59NfthEgAxOeJ5tYdxPP\/20cHLz7B1lQeAZF1ztQWVheUwyeEl5vhFdOI5hdcnnhOZCxWPMnSYP4ZIfhFSPBATVJ4\/tf86jf\/7zn94dFM88kh8E7JIPSWWSDZJLHhcg9ua8I2SCC7MNdSFtzc3N3nNTkEraIccPSCDlCh7Qpw7nOH8KkLgf2ITjr224Y+c5HH890txIhPVNeTHAtcMCmots5gm8ajxnmEt7tw3nr5t24\/lcSzy\/jU0BzyC5cjXOcaRcwWMIWpZNGOTJzaQ91wIvedB\/JvXzqQNJxoPFscPmPEITJo\/zguevwsozyaePsHr5HFeVCYljN0HTbggBo3+e3eb5SK51t9yN8\/iJm04XZw5NNZ\/zKBCPQPnlMFf480gztzBvsmby\/B1zO2Pj5oJyP5hH\/HmFTLNWMD5uclgz8MKFyec8yccBxBwdJpvtf3gJOnDO8h5C2LrCuwF+Oeya+vM0jRzkw3lY71KNkXJtxyMZYQ4QjiPeRa5pvPjITiWX3T6VS8hYU3liOSbYAdk4LeiPduVAgkBiZPb6YfospDD9VG\/3uFttYV4DnjHjboq3m3VwTNRcWJr2h7luR86ePdsvKpEuFoFk3ev5KAAAEABJREFU0gp7bo3tKRYr93tyfCaErWrITEI5J8JFGEZCnGqJ6CmnnOK9Qc8kxQtCfEYFV32igi8Cgbr++uuFSXvhhRf23sLk2RGOuQT847GAgOzQLMbMxch5xKKNPkwqe++9d2gb\/11daEVfATcxYYst9uXh51gs5rWKxWLCIhm2eEDawiZ1T0AGP4w7qBq25Txwz0G2NDlOLG5BbdjSDMovVl6uN22qD+ctL8ThCdK8bMNyX0vZ6htWP8iTG1Y3KJ8H4nkJK6isEHmQAuZ1notmjuca5XpJdTPGwo1XhP6zRSmOa5gHkPkODypjXGihhYTdAuYFCGXQONgFY14JKvPnMS4eu\/Lna5o++DScpt0QQuGmNQ6xd9dK8nmpk2fXifsBQYHI+PMLkWbewjHBesWLO6wZnJu85xAmn\/MqrCxdftj8C3nmBtxtz3HlOXo3T+P+x6LYEYDMabkb8gY2vAf5cJ7FFltMGCN9uvU0jgNA43gfOV807YasBauttloiC9nI5VnRRKYT4dOC7twZxqdowrXIMeHa5VrmRSfWQXd9oV6p4BFI3oTmTR9\/pyx0nDj+fNLcfRFyAocZkkWFOn4g97DDDvNne2mX5XsZGf6wiIVVXWSRRcKK8srHrR0mgLuEoBduqM8kxsEn7odLzP1l\/jQPHPvzNtxwQ+EC8+eTpj62J65gwuIC0rQbcmzddKo4E47\/0Qbqx2Ix4dM6YXdJuXquwjw92BX70rcfeJW4gfHnk9bzmXguCPuSAJ4ebOyXyXY5N1f+fNLPPPOMuBMKeQCvJRO4H9wEUJ4rCkFYuBZ4zi9XHWgf1rYU11JY39nks9Wbah7KRBbt+eRXroQtXR\/czATND3i8uAEMao9OvOAQVJYur9jHFe952PrDfOSf79CXtS7IBpRlMg9ww8VXPbALbfxgBwbS6s\/XdNCzdJThPIEYQXjca5K5lZ0aSIsLHr2BnNC20GCOweHhl4vHLoxgYY9sd9BUPl8qwKZ+YA+toyG7RWGOB\/+8meoLJ5BH5mGVqyGP5mjcDTnP9AYDp4NbpnH05Xhp2g35C3luWuPYTXdwGVvYNYPsID7GceImUOWVMvQIJCw2rFMeEsVb4se+++7rNYHhexHfDxcoFxou5SBgKF8TL8k2gxfJ8qehoSG0Ra4ndajAvoKwk5OLLIww0TQWi3nfaCTuR+rnoPprY19OnP6c3lhNTY0sscQSvQnfL3e4viwv6X\/b1suM\/4Qdo3jRgP+cD0H6UBGdwryQ3MlTJ1uEeXq4WeACDDrnyBsyZEhgV\/4718BKKTL5jmBQcVNTk9BvEMLsy4TCB5798lgMObf8CJpU\/G1TpXVS9NfhHMMzxU0NExyPPNC3v56meQQjbHHUOmFhOa+lMJ2yzU\/1HDZbWHgwuL4fe+wxgRSHyaceW65h5bnmQxDDFjdk8hkSwiCEnSNBdd28Yh9X1hi3Pzce9ngOpIudGP+aRjrsZtqVy5wFmXDzNP7zn\/\/c+wySpoNCbnKD8sljW5qbThwBPBrCM8bscrG+8W1fF\/n8uU36CgPXOIQurDyMQFI\/jA9QlgqQ+tNOO0384HyFrPEoGI\/K8fIejgA8cankaVnYeYsjAdlazw3xHnKjxfngB15K6qIToR+p5nvWCOZUfxvS6pXmWVDSQUhldz5lxPkS1K6YeR6B5O3KsE54AJhnc\/zgLULaBC105LMIsm0YBg4M9fzgYf2whdVf102PGjXKTSbFU03sSRWzTDCRBDUJI2pu3TDSxsSUCeHlxQBXnhsPO0mXXHJJt1oizjZJIuFEYrHeLWAnKzTKh7JDC+MFYds5nCe5PHwdRiB5qDnsnCOfCTquzoD\/uXpYEIT+jIO4H2xz0W8QwrY0kMF1QFgKcO0w+TCpcu6yHcID4HgU8UxBylnAWFTwdqbyroQdl3TjKOe1lE63bMoh87yxzuMJ3ERiRzzNbGGR5lrjOmQbKuwZKvoLI16U5Qq2vVK1Ra+w8rCFOKy+5hf7uIYRFs7jsBtadONc969ppDnXKc8VrudwgIy+jFTrbV8VL8DmPOvG1xk23XRT4QaAx7\/4ULpXoUg\/66+\/fkrJfM8ybI1JRehTCnUK2ZXi2T52kiDM7LzwWBSeeYhjGHlzRCSiYfPRaqutlqgTFIHQcj74obuK6BjUjkcmguZ6zQtbJ3S+D+Mq2Fs5V1C\/5DE\/E5YSHoHM568khB2gfAaRzdap9sPzLRr3h5lc1P42maTxygTVYwEOynfzUr3lGbY967bPJY4nMJd2mbRJN2a9cwuSlcukk80kEtSnP4+tMH9epumwm6hM2wfVW7BgQVB2UfLYgcAjwzNMPE\/DCz88XwUR8nfIOcTD\/f58TYdNrFoeFg62ayloHEzgLHB4SvizqHgSsSMvRQTVh5yHXTd4K4La5JPHc8+p2n\/nO98JLYbMhBamKCj2cQ1bf9KNNYXKeRXxAkS6l3HYSYAMZdsRL5qceuqpApnEI5dt+0zrp5qrVcb48eM1mhTmMperANZ9brYgb9yospOkZbmGYWtpKgdMur745FbYeZ2ubVh5a2urVxRGIMPs7TXq+5kwYUJfrHSBRyCVVefSLd6XXNqlasMBSlUeVBY2EVM3lwWerXS27vxwXcyc8Mj3I+iZN38dPDr+PE3nug2o7QsVQhYylYXrPlXdVONNVRYmM9sXfEQkTFTe+fmQz7DO\/c\/yhNUrRz67EmHbcLkSn0q\/loKOE95I\/ohCUFmhFyj6CHrei3xFLBYL\/XxVJnOaynHDYh\/XsHkgV31d3XON83JjujmcR37CnolL1y\/eKjxyYc8BpmufrjyT+ThsfOnOsbC++RA535ct9KfswpxH6darMD3JV7JHvFDQXVd2IHOVmcoplavMdO08ApnNB6P9AlNtE7B9ky3Y5snl4ueujr78+pHmUwhhEw3lfnAQeeGEZw784GFmrR+2iIadtNqOMNWWaaqtJNpGEWF3TqrrV199pdEBofum+oDCkIxUHmfOg2wR9txoSPdJ2YW+BtA9nzvkJOWKlAi72011XqdSpVqvJeatILukul6C6meS5978htUP83wzL4e1SZVf7OPK29VB\/Rd7mzeoT83DW8vjH5oOCnHa8CIpj9TwfGwqB0hQe\/J4STXX6432YWD9CyvT\/LCvnuQ6j+K5T+1xFOGtaZ4x5WsavGikuqQKwx5JCNtKTiVLy8Keo6ecrWbm72yhxz9s3s\/ESVEMZx5jSgWPQKaaWFCK51j8mDlzpieX56e8iO+H5xbYvskWbPvkQiDpngdJCYOQ7oJ22\/CygJt245tsskkiyXM2iYQTSUemqBrm6ucEDDvpaRdVhG0VqL5hXmDGm4qAaXt\/GHbe8cZ3tucc9fnckL+PTNOpHgHhmUzkZ4t0z+hkqlu6enj78RL5ke6mMswDEUaI0ulRCdeS34akw+yk9ggrDyPo2i6XMOwaVFksqkDTbpjLTR7ti31cwwhLurEy\/\/rXNNIcM\/TOBGGf16Ft0AeiyfeD64XnY\/n6COsOW9usnf56QWmOFS9kBZXlk4cdUrXHRnhBg+ooEQoqS5UX9vk6iBhzMy+Z4AjizXqejcyUI4S9KJPOyYMNgsAY2Jlj3SLux8033yzZzvXU5w1rZIVtVXNTkm5XKt05j\/xCwyOQL7\/8cqhc3sLmbTY\/eEaBRmHPzbBwUl5K8HZWWH883MqkEVau+RBmHuDVtBvyYLzrHQw7OfkYLBeZ29Yf52Lw55EOe7mGslKDLbZM++T5nFR1w+4Ycx1v2IKW7i42lY65lqUi\/Lk+E5irLtm249MhPNTvh39xdOXyiZmw77ale9DblePGB\/u1xA2134akeS4yjCQyfhYpQj94ecCfl2+aFzJS3Rik2hKF6OTSf7GPqzsfu\/rhRQsjCdiAlyP8axpp\/iCDKycsftxxxwlvSKd6m54vGPjbc8MGYfCDrzRwrvAnFa+66irhs2CsI3gpU5GysOvQ32826VR8ADn\/+te\/CAIRNi8HVu7L5PNXHK++ZFLA88Tbbrut8Oa8W5DpM\/Bh50eqZ0ixKedCENTZFuZZh+i6emYbD7shQg5zNWEQIJeF3v4P6sef5xFIvoQeRHggU+6WrduYyZH0GmusQTAA3B3xzaoBBX0ZTGa47v0Iezu7r1nKAALJHUtYJd5i49nGsHIIJs+moHtQHU5kNz\/sJKI9LyS4dd04pIKXFdw8jUNSNT6YQj6cy4UXpDMXexhhDrvAg+S4eUEveFDOAkB\/xP2A+HAH6z\/nSKe6OP1y\/Gk8kGHHjTtSf31Ncy7Stx+8pIKuWk9DHsPgBRc\/kKN1sg3DPgXC1hr9BcnjRYEwD0TYNREkx80LazdYriW2UsPmHmzpjlXj2LfQN1YqOyjkmIV9pJ4FKGzuxdsStrUW1I+bV+zjmmprnW8ZurponOPBeaVpNwy7jt06zD2\/+tWvvCy+pBB23JnjuVa9in0\/EBBenvKDR6ZYb\/uqCc8S8scYeE6Sl3LCSGQxHnWA3ELcVBc3hCfwiRs3T+OcJ0DTmYZ82iqoLucOz1sHlb300ktB2QPywtYXPHph10IYQeY463OG66233oC+yIDXYCPifnB8+fqCf74nrTt4YQ45ZLHNr89KknYB1woj4SLiVvXi3LBwbvoBB\/IqZPjjEUge2D7\/\/PMHNOGTAQMy+zJ0i41n0Xg2oS87KTjkkEMkiFRw8Ll740\/x+JHPsytcdHphS8A\/JlAuVCYWFl0ONBMnhI6JnLvSVB6s7bbbLkkqnzsI26LhbTm+35XUIJ7AHjzHEY8G\/ufh6MCCQZB5+OGHCxemqyp30vxFIjfPjW+55ZZuMuM4n+EIqszCgPcs6Dy6+uqrhbeN\/eccaZ5JCpLnz2MSCCJ3fAbHX5c0f5UgaDLm2SWuD\/r2A69ULDbwE0pMEpw7fuRzzuAhYmJEVz\/4JhvXh5vP8Q3yrFCHxTwXDwRtK+FaCvuGIDcteFkYp4K5h8ctNO0Pw27M\/fWyTfONQf\/CiS78ZRrOuyB5LHBB+ZnkFfu4QmzDSB9\/go9rxtUTbzDXpJuncchPJo8OuN4w2rDtrDL8IS\/U4PHUfIggbTTthkHrBeVs14a9bFWst80Zk38XkXMYwhx2g87air7ZgmMS1IbnLIMIEwQ30z+vyV9pC5JNHjfq\/m1fSFjYcXAfkeO8RoYf7MSddNJJ4l8jOAfYzeWmwj\/fk+Y8RlYsFhO+w0ncD3gTHm9eOHLLOMcZi5uXLs4H2\/3rCGkIcLq2bnmNJrioMBAnPASLPXm8Slruhjyj4XouUk0weO0gU0xaMPuTTz5Zwr70Th+QOMJcwWIaRi5UJn\/6ipMdLypbbnw2gIkV8qF1\/CFfeve7lyEdXGj+uprm+13caZ533nmCTbET9uAk1TpuCNEqxtaV20c28VhsIIlJ1Z4FCCLFZ0s4oTnOTHwQ96B2kO9cjzc3LlwEQXKZ4PjUBc\/OcHHhDeZ7jGHEhwk97Lr5mhAAABAASURBVI7SL59zhGOOZ5GbDiWqe+yxh79qIk19tqaYmNAHvfheYNh5wDM+icYliIQRHyY7\/koVBBi9mQ84ntg3SC0IEc8HBZWly6uEa4k\/+RY0zmnTpglzDB4EtokJmX9YOILqs0hnQmSC2maSx7XJIsVNPAsS12wqD3yqcztdf6U4rrxMEqYHN2nc+PPiBX95hDgeyKD6ePtisezmPOTssMMOwlxG3A\/mROYKNz\/se5zMT2xZQ9y4xvAGPf7448IjVWGEKez7um5\/ucZ5K5o5nPNl7733FvpK9Zd6DjrooJy6CvMysm6cdtppgi0QzGMi7JayS0g6CBA1N5\/Hi1h33Tw3vskmmwhjw\/t+9tlnC9vW9OvW0TiOCY1jCzykmnZD1gXWeRxw7OASss4Rd+tpHD7FeqbpVOczPIqPzHNMIKScz5zj2rbUYYJA0jELGg+zQrDCLjLq8fwHoQIvUpgXkjoYkAEfeeSRcuONN5IVCAhBqjuGwEa+TJ7bY7KAFPiKck7yJjYfAg0SwOILgsrI426J5y+xadiCQT2AfQgHO1iMONG5Y0o1Fs6jXAkHcpmwwo4zkwALBhcXD6mzDUSbIOB9d70K1GFbmjAIPGsCWeKmg+NLHZ4RSzVRse3LDQX6oBdElHZ+4E3hOvDnFzONTmHyuaOGAFMHAhmmNwScR0jC5GSSz3UEwupi6yhfS3xPM4xIcD7yVzQ4ZwnZ9QkaJ+cznvygskLm0T\/eBh4tQbcw2ZzT6h0Jq5Mun2MKwurle1y5GUvlNHj77beFvy\/MjRuELkiPsWPHSq5EGZLMfBAklzxIEI8rEAfclBEGgRdpIG6QCmzGzYS+bxBUP8wTFlQ3lzzmcOZyXm4Nu\/aRy3mCDYlnC747yXkf1A6+gC0gmfyxCl5OSqVH0FcEcGgEydY8xoaDiB2qMNn88YRVVllFmwjHnHkxkeGLcH3hOMOzT6gk2FfNS+Kx9CJ9P+wK4fDoSw4IuF45JnjYObcHVChhRhKBzKRfLhS8dm5dSAAP\/rKF5eZnE4c48hHRbNqE1eW5Bxb5vq25sGoZ5UOM8Zqmqkx52AWQqp1bxkmW6wXoyhkscbweTPz56MvEk83b9UF9MUHjVfOXsW2U7fnMnSOeJr+sTNOcr5C0TL7DlqnMTOpxJ42HPJO6QXU49\/2TYFC9TPIG87XETQhEBXtkMtagOpAN1xsRVKdUecxHqRaybPQo9nHNd+2AXHLNZzMmty67d6mufXahtD7zDSRR07mG+Z5rufbrb8e8lY6k+du46VgsJvm0d2VBAP1fQeEGKJ\/zg3UA3uP2Q5xdrnzkIoPzDr5C3AXEE7u6eVGMZ0Ug8SKGuY9xFfPWWKq7qzADcMcN4WMCDquTbT5vtN1xxx2S6s40nUw8TGzfpZtYWIDZnsz1gPOMFFvv6fQpdXkslvl2Du7\/TPVjK4Abjkzrp6rHA+08GpHtcWaRhzSlIj6cl6n69pfh\/WZCwLPqL0uXZvF54IEHhHMpXd1ilLOlyXZmtrK5wcJrwvWWbdug+ox\/MF9LeA\/wGHF+BY0vLI+5g52asJ2OsHbZ5LM9mqle6IP3p1DP2OV+XHtHmG6O5PpnHmCx722R+S+PFNA+8xbBNVORCbY1ee5eW7L9yDaoprMN2c2AiGbbLpP6bOVnakfGwKMukLRMZIfV4d2FTI8B5zDHjN2aIHk8huffyoagcnMeVD9VHna46aabBH4TVA+5EHlutoLKw\/KYJ5g32eoOqoM9sWvY4w7+NtiE9cyfX+x0RgSSbRmIFAQylUIYme1AiCQLS6q6lHFwkMvWLS\/AkFdIIB9i+qc\/\/UkYQ6ayeQ4CtzZ3AenIo8rk2UXeAGarT\/PShXjgeC6Ch1fD6oZ5o1LpFWbLsPxUssL08udz8bMlFnZRU59FiWPBs1dhusRi4aQ1Fgsu42+zcpyZIOiDvlKBSQ+7Q5pS1eNmiOePWPxS1XPLsCWTId91461\/tywsDvlma5Y3ecPqkI9sQj\/y+asKrizeEGSrPZ1dtA2TJ3YPuoPWOrmExbqW0CXsego7H2mTLbiuOf7MH5m0ZauSxYLzMpP66eqEnSdsATI3018qGTyLzbOabBuG1cvFjsU8rujJPIAdM73uuGnjUS2OF+39CBtjmH2xF+8O+OVomuf3NI6zhPcOsiUfzElcozwKobIKHbKO89wmz+2lks0NNjtA1A+qF4sFz9fUjcWSy7j5hhSmGxeEihsFjlnY9j32CXoxh21oylKtUeimYH7j2W\/\/uw9ariFEHrnp+JHW5xEl\/tRpuptu7MojheySadugkN1bznt3i92tFzS3ha0ZYee2K8+N1\/DdIpgwiy\/kB\/LEiYNHjDsqFlqUw9PjNkwV5\/MD3HFBqHiuADncWTMxcXfAnTavj0PSspGbqs+wslgsJjybxRiYLHhQmYdmOZmYsJlEGCtbyOhFHRZzyGeYzLD84cOHC9uY2BRZnFA8P4k9eY4DG\/CsCKSEP5HIBeN\/HMAvm5OMt8T9SPU8JRe\/vz5pSJ5fPmle2qDcD95YpjxTcHHzbAZE8swzzxTGj1cL0ohdedaJY5FKHs+W+PXQNI9KhLWNxWLCMeX7XpzP3I0xwZHH8ea4s3jylj3HhmcWw2S5+TzXhq05X5m4aAu48bnrrrsk7Pxle53nXjnOeHI4v9i2YnHDJnwKg0nn\/fffl912201iseQJ1dVB47yQpbZwQ+yqdfINuTPGdoyZxQ1vKuct5y+eY54VwrPPs5Fc12GLbL56FONaQieuN9d2GueaobxQ4BladjB4IYI5kC0w5hnsyHmAR43rhJcSIBIsFoXqmzlIx+WGEEfGT388p8ycz3NcHGM8YhxvvGTnnnuu4AFJpQ9zgytb4\/p1jrC2xTqu2h9\/lIDrjnFwvXPdswBjd+YCtuS5YeO4cA2mmge4FnRcbsj6oP35Q9Y3t64bpz+3fiwWE8gHcxZzAceDa4w5gmuOkJuQM844QziHmENx0KCXK6cYcdY\/Xixh\/eeaZy5l3iJOHp\/ewfED8QvrP9u5nGsGzyrjxBasH8zd2IT5kx0arhkdP+esa183znkQpBdtWaO4wWMe53gx\/0JIsTnrM9eBzm949oLk+PO4IUBfPo\/DecdxQm9uxjmGjId1ia+w8JgK9f0ygtKQP3bJ3njjDWHN4XrlfAbMv6xJzMeQXODaQOP07ZfNmqHl8BMt57hrPJOwhoFAUri4mHi4ODhxOGAwcCacTAT567DYT5gwQXhOADmceExM3KFB3MaOHetvUvQ0kwWLMM\/jYFROIC5qxsrkjl7UyVcRbIosTigemMeebNliA17ogJQUcsHIV99Cto\/FYgKRZJFk\/Hi1II2FsGsmenIHxfnMhcsEx3HmeHPcubFhActEjr8O5yseDo4rgDhyx5dOHseZ56M4vyDVLG7YhBsXJjMmWX9fUUjjdWVxw5vKecv5i+eYSZY73rAJutC6D\/Zrie\/GMQfy6A\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\/yBolTbyOkEXIHCkka\/abDC0kf9IeX88svv5S2tjZ\/tYpMcx0ABsc1wrVC3GAWGKwWMAI5WI+c6W0WiIgFWAiBkkZehoEsKmnE8xIRVataDUgjUNKYy3ONhTIgJBUiibwZM2YIZLaatrW5PrhmIJLmjeQsMAxGCwwGAjkY7Wo6mwUq2gIsfkBJI4NlUVTSSJw8Q3ktAGEEShrRBu8fzyRC4gB55QDeyLFjx8oiiyzidQ+JRFcvUeE\/XB+uN9KIZIUf8AodnhHICj2wNiyzQKEtgKfEJY32MkyhLVwYeXjyIGJKGov5XGN6jdPXaGxs9L4jCbFlWxsiyRjStxzcNfDMQyIBI4FEcn0RN5gFBoMFjEAOhqNkOpoFymgBFjUWN0Ac7wmeRhY+4mVUzbp2LKCkEQIGEYOQ4Wlkq7icnkZHxZRRdERXKjEGQLzSoUSSa4nri+uMm7VKH7eNb\/BbwAjk4D+GKUdghWaBXCzAQgZ0i9r\/XGMuMq1N4S0AaYRoqbcR0ggJgzhCyArfY3Elsq2N\/oCeGFc1eCMZKwSSmzLikEhA3GAWiKoFjEBG9ciYXmaBElsAwgiUNNI9ixreRkJAnqG8FoA0AsgVnsZyvgxTLEsokYQQQ5BBNRBJ9Ub2EUnhWjRvZLHOMpObrwWMQOZrQWtvFhjEFmBxckmjPdcYzYMJeXJJI1pCrtTTOBi9jYwhHRiXeiMhkdggXZtKKFciyU0bnkjAtVoJY7MxVI4FjEBWzrG0kRTaAhUsT0kjCxNxFio8jXg+iFfw0AfV0CBMeBohT1F\/GaZYhnW9kXhcsQWEulj9RUku1yLXJDrptUrcYBaIggWMQEbhKJgOZoESWACiyCLEthhxXZwgjsRLoIJ1kYEFII2QJIgjhAlPI144gEcuAxEVWYWx43FlcNgHVAORdL2RXLdcw+aN5CxIDSstvgWMQBbfxtaDWaBsFmDBAUoa\/S\/DsDiVTTnrOGEBSCNQ0liJzzUmBptnBCINEAOJxG7EKx3c5HGzxzghkcCIJNYwlMsCRiDLZXnr1yxQJAtAGIGSRrrRxYcQkFdcmPR0FoD4ACWN1MfbiJcNbxsgzzDQAtW8rc2WNsAqkEiudeIGs0CpLWAEstQWt\/7MAkWyAAuJkkZ7GaZIRs5TLFuuLmms1uca8zRjojkk2\/VG4pFMFFZwhJ0DSCQ3g1z3EEnzRlbwAS\/10DLszwhkhoayamaBKFpAFw8ljiwobHPp4hJFnatRJyWNEBx9rhFPI+QHElSNNinUmNUbiS2RiUcXexOvdHC9c60zTkgkIG4wC5TCAkYgS2Fl68MsUEALQBqBkkb\/c40F7MpE5WEBSAyA0ChphORAHNOQxjx6rd6mSiR5DAB7Q9bx+Fa6RdQbqURS54VKH7eNr\/wWMAJZ\/mNgGpgF0loAwgjcxQHvA95GQpBWiFUougUgjEBJIx1CaJQ0QnLIMxTPApBziDo9QCI5HsQrHUokmQuYK\/BG2rZ2pR\/18o7PCGSQ\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\/\/G6l0yBMnWk3kglkjo3l0kd67aIFjACWUTjmujiWADCCNyJCbKopJEJrDg9m9RsLABpBEoaeRmGxVRJIwtsNvKsrllgMFoArzo3S+gOieSaIF7pYB6GRDI3M1\/jjeTxokofdynHV+6+jECW+whY\/xlZgImHSSiMNDJJZSTIKhXVAiyOQEkjnbmkkcWUPINZoJoswM0SJBLgjeT6qAZvJMeYuRkiSRwSyTxO3DD4LWAEcvAfw4oeAZMNpJGJx16GieahZiF0SWP1fOQ7msfDtIquBSCSeOC5qcIbCbh+oqtxYTRTbyREUud0nAKFkW5SymUBI5Dlsrz1G2oBJhgII8SRuN7BMvkQD21oBSW1AKSRBRDgVWFRZHHEy2KexpIeCutskFmA64PrBLW5friWiFc6IJL6qBFzPDAiOXiP+qAmkIPX7Ka53wIQRaCk0T7y7bdQNNIsdIAtOEgjzzWyEEIcWRSjoaVpYRaIvgXwRnLtcOPFtQSRrAZvJEcGRwAOAeKQSOZ+4obBZQEjkIPreFWUtkwaQEkjg2Ni0TtU4uQZymsBCCNQ0og2LHpKGlkIyTOYBbKwgFXtswA3XlxLJCGRoBqIJN5ISCTzPOsARNK8kZwFgwdGIAfPsaoITZkgmCyUNNpzjdE8rCxgLmm05xqjeZxMq8qxAN5IwIggkVx\/xCsdEEiIJOOERALWCdKGaFvACGS0j0\/xtCuxZEgjEwMgzqSBp5GJg3iJ1bHuQizAooWnkQWMbTX1NLKw4SkJaWbZZgGzQAEsgDefa43rjuuP65CbuQKIjrQI9UayHqCorhPEDdG1gBHI6B6bQa8ZRBGot5HnGpkgII5GGqNzeCGNLFQQRxYtFi8WMbbVjDRG5ziZJtVjAa47rkFGzLUJiCsqNVQiyfrA2gGRNG9kdI+2EcjoHptBqRkXPVDSyCCYDJQ0MkGQZyivBSCNQEkjL8NAHJU04gkpr4bWu1mgui3ANQiJBFiCa5VrlnilgzUDZwPjhEQC4oZoWcAIZLSOx6DUhjvEVKSRyWBQDixU6cFZwOIDWIjwNDIKlzTi9SDPYBYwC0THAkokuVa5bvFGVuO2Nk4J1proHBnTxAiknQM5W0BJI3eHxCGK6mkknrNga1gwC7DQuKTRXoYpmGlNkFmgpBbgBk+9kZBIruuSKlCmzti1whvJmsJaA\/IikmUaRyV2awSyEo9qEccEUeQC5m6QOBc1F7cSxyJ2baKzsACLC4sMwGuB94LtaRYgFqIsRFlVs4BZICIWUG8k1zHXNdc3N4kRUa+oauhaQyesQaw\/xA3ls4ARyPLZftD0zIUKlDTyMgwXs5JG7hAHzWAqWFFII9Atav9zjREZuqlhFjAL5GkBiCQ3hIiBRIJqIJKsNTgsAGsSRNK8kZwF5YERyPLYPfK9cnECJY0o7JJG4uQZymsBCCNQ0og26m3E0wjIM5gFzAKVZwE8kYCRQSKZC4hXOiCSODAYJyQSGJHEGsVGsnwjkMn2qOoUF6BLGu0j39E8HfA0sFAoabTnGqN5nEwrs0ApLIA3EhLJjWO1bWvjiQTYGRLJ+kXcUBoLGIEsjZ0j3QsXHRcfII53kbs7LkzikVa+ipRT0oingYWCBYNtLBYP8zRWx4lgozQLhFmAOYC5gHLmCMDNJelKBt5IXatYv1jHcIZU8pijMjYjkFE5EiXWgwsN6BY1zzVyEUIcjTSW+GCk6A7SyEKg3kZII4sExJEFI0VTKzILmAWqzALqjWSOYOjTp0+XOXPmEK14sG6xhjFQSCQwIok1igcjkFnZdnBXhjACJY2MhotOSSN3cuQZymsBSCNQ0uh\/GYZForwaWu9mAbNAlC3AHAGJ5Ibz22+\/FW5CefQlyjoXQjfWMEgkQB4kkjWPuKHwFjACWXibRkoid2BcQGGkEQIZKYWrVBkII1DSiBmY\/NXTaN5GLGKIsgV4ZjrK+kkVKjdixAhZdNFFvZFDIoGXqPAfJZKsb6x\/EEnWwgofdsmHZwSy5CYvTYdcNJBGLhziXEjqaSReGi2sl3QWcEkjzysZaUxnMSuPkgWYWwBzzYWX\/Fmeev41IU5elPSsZl0aGxsFbyTAC8lNKmE12IS1zvVGsh5Ww7hLNUYjkKWydAn6YdLmAtEJXC8eJY4lUCFVF1bWZwFII54AJnJ9GYbJHZinsc9IFkTWAswzgHnmp3vsKwDiqAoTJ4+5yLw+apXyh2xrs6PBTSrzD6gGIqneSIgk5yPnLWH5j8jg18AI5CA\/hkzkgIuCkJdhII5KGrl4BvkQK0J9SCNQ0mjPNVbEYa2aQbDgMr8wz0AOAUQxlQHwSEIiaZeqnpWV1gLcpHKzSq+QSOYl4qkx+EtZC3Vd5LwEnNeDf2TlG4ERyPLZPueemZABkzkhglzSSJw8Q3ktwMQMlDSiDXf\/eAGYxAF5BrNAVC3A\/MJCCzIhjf5xQDJpx1xli7XfOuVL442ERDIfsQsCkawGbyQWZ33EG0mc85pznLghewsYgczeZmVpweTLic5ETMgD61wIekdFvCyKWadJFmASdkljoZ5rTOrEEmaBIlqA+QUw10D+8CRCBPPpkvbjJ60kLNjMZfnIsraFswA3sdzQIhESCZjDSFcy8EZCIlk3OdftvMztaBuBzM1uJWulJzcnOHFOeEijnvwlU8Q6SmkBPpWBp5EJmDt67uyZmLnLZ5JO2dgKzQJltgBzC1DSCHGE9BVaLciozmWFlm3ycrcA8xRAAnMYN8HEKx26njJOzktQhBscxFckjEBG8LAykQMmc0L\/c40RVLkqVWKSZbL99NNPvY\/1QhqZhCGORhqr8pQYVINmoWR+YZ6BMIJikEa\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\/QYAcgnCpVhJoS3A\/AKYayCNbKlmQhxdPfyEDDLklmcTp62CdiqbkHQhoTLd\/gopPxNZbt\/og+05BhwPjksmMqxO8S2g3kiIJN5I5nzm\/+L3XP4eIJCQSXhI+bXJX4PKJJBxuzBhcPepkwcHDjcyIB6vYv8jYAEljdyNMpmop5HJhbvVCKgYORXmzJkjLmbMmCFtbW1JgGB+9cVnstqa60dO\/0pSiHkGMM9AGgHEJd8xQoAAciBGhJmC+kDrIwdoulghfQDk0z8gXkzQh4J+6B8QV3A8OC6sB7atrVYpfwiRZFeJOZ\/5H1QLkSy\/9TKu4EQAABAASURBVAujQUURSCZywGROaM81FuYkKbQUSCPgzlNJI4SRycRIY3pr19TUiAtauIRS4+Q3DxlKYCigBSAhzC\/MMxATAEkpYBcJUUqGXJKUKHQiWk5INu0UpEsJt1\/VJ5v+09VFpoK62h8h6TDgjYREcuzC6lh+6S3AnM\/8T8+QSNYG4oboW6AiCCQTOdCJAQ+jPdcYrZOPSQEoaUQ77jyVNHI3Sp4htQW23XE3aWxsTIDaLpnU+NfTv5Qvpk2RpuYhVDEUwALML8wzkJBikka\/qhAjQL5LnDROSBl1FKTLDXRBB\/QDxHMF7RXIQLaCdKaA6HPsOIbcCGTazuoV1wLM\/5BI1gScChBJ80YW1+aFkF4RBFINYaRRLZFPWLi2TACpSCN3noXrrToksVXtjtQlk27crWPx3C0AaYRsQBwhHhAQkLvE3FtCmLQ1ZIo4eQrSUYOrGzqDTHWkroI2KouQdD7AG2nfjszHgsVpy5qAUwHpkEjAOkLaED0LVBSBjJ55q1MjJY1c\/PYyTOHPAUhkOuB9pOfa2loCQxYWgDQCJY2QjXKRRtRWEkVI2iVQmkd+lIHOAB3T6Uw5oC6gHSBeaHBsuUHgeBdadlnkVUineCMBw2EdYU0hboiWBYxARut4DFptuMC50HWLmq0IJgDAXeWgHVjEFO\/q6pIwBKn68H13BGVbns8CbGf++c9\/lvc++krwNIJykkbUg0QB4gASBfxxtw5lUYbqj85AdSWuII96CtLFBMeZ4w2R5DwoZl8mO3ML2LZ25rYqV00jkOWyfAX0C2kEShqbm5sF4sgWBKSRCaAChpnNEMpa1yWWZVVkEHWO5wlPI9uZkAhUd4kM6VJC+yakXyVRhKT9IB9QH\/jLo5hGX4Bu6AyIk6cgXWqoN9KIZKktn7o\/1hIcEdTCSQGIG8pvASOQ5T8Gg0oDCCNQ0ojyLmnkYifPUBwLLJjfKm++8ow8esKbHlL1MnmtjVIVV20ZpBGSAHGENOKBUmO4BEaJjZYVK6QfBX2oDoSkM4HWdeVk0q7UdVQ\/QvpWvYlHAZwLSiQ5T6Kgk+kggjMCEgmwB+sP6xDxwsEkZWsBI5DZWqwK6\/MQMxcrFy1vyNlzjeU7Cea3tiR1rkRSQ8glJJNK3d3dBIa4BSADQEkjJAGyEC8K\/K\/EBqIDAivlkYlMBWLoT0E6F7jtkZ2LjGK1QR+AfNWTUNOElAPi5QbnBjcX3GjYtna5j0Z\/\/0okcVqwFuGNZH3qr2GxUlrACGQprT3I+oI0coECLlYuWranuQs0T2N5DuarLz2TtuMP3n3NqzNixAgvHCw\/hdYTwgiUNEIIIAaZ9gPBAdSH2ADi+QAZQGUgH2i6EKHKox9QCJm5yKBvBe3RCxD3g3xAPm0IowBuNCCRIAr6mA69FmD9YR0ixfrEWkXcUFoLGIEsrb0j3xsXIlBvI881cqFCHLloIz8AUzBhgaceuz8Rr5YI3qJ8SGOQnSA2gLJcyA1tFMhAloJ0MeDKd\/suRl+uTO2LkHzVg5B0Omg92oN09UtRzk0HRJIbEc6vUvRpfaS3gHojWZ9wcLBmmTcyvd0KWcMhkIUUa7IGkwUgjIALkAsR3dXbCGnkQiXPUH4LLLLIIqFKLLLL3ETZZRtV1\/a1kkY8Rdl6GhNGSxPJhtxAfhSIpa2CdKng9ok+xegXuQrka5+EpLMF7QDtXLmkywmI5LetIpxjRiTLeSSS+2Z9wsFBiDcSGJFMtlGxUkYgi2XZiMvlAktFGiGOER+CqZeBBVIRzgyaR7oKpBHgGVLSyCJfTKUhNoA+IDeECtIK8qinIF1UpBGOHlRx9SOdK1QOITKQryBdCLjytJ9CyM1FBv0D2u613+EeieTcI22IhgXwRAK0gUSyvhE3FM8CRiCLZ9tISuaiwtPIBYa30fU0GmmM5CEboNSM24YPyKuWDBZtoKRRiWOpx6\/kBlKhQAfNJyQdJaATQCdXZ9KZwt8OeSDT9rnUU\/n+vnORlU0b7Y+Qduih4EaFc8+8kVgmOsALCYlkXWN9Y53DWRIdDStLEyOQhTmekZYCaeRCgjhyUXFxcZHh9jfSGOlDl7Vyb+5en3WbqDdguzAKpNG1E6QCuHmDJa4kCH0zGQN1FLTR9oSkSwH6AvTl6kK6kFDZhMilTwVpP3g2km+IGpH0W6a8adY11ji0YO0DRiSxRmFhBLKw9oyMNEgjUNLIyzAQRyWN3KlFRllTxCwQYAEljSzQeHvw+gRUK1kWpEJBp0osNCSPcsLBAPRGT3QGxBWkFeRRV0G6XHB1QL9C6PH6628JsgDytA9C0pkAIgmJ5JzNpL7VKb4FWOMgkYDeIJGsicQNhbGAEcjC2DESUrg4gJJGlHJJI3dl5Bkq1wIr39wxqAfHAsxCzBb18qusK1dcfbO3uJdrUJAKBTpAKhSkXZBP2q1POspAZ4COqjchafIVpKME9EIfdAXEswFtFLRDnoJ0LuAGhxsdzl+85rnIsDaFt4ASSdZCduAgkuaNLIydjUAWxo5lk8KF4JLGwfiR77IZzzqOhAUgjQDSyAKMN4fF2F3QWexLpSx9KehT9SAknQrUAdRxZZCOKtDTr5uOwZ8fpTQ6AnQKGgP5flAPaD7tgaYLEXL+QiJBIeSZjMJYAAeK642ESBZGcvVKMQI5SI89pJELAHBXxd0V29NcIFwog3RYpnYKC1TSG9UQRqCkEeIIaQwavi7wLPwgqE4h8pANVBb9Ak1nE9IO0MaVSToKQCcF+qCrgrRbRjrKUL3DdNZ8Qsah9QlJFwOcyxBJzm\/O82L0ERGZg0oN9UayTqI4u3U4YYgbsreAEcjsbVa2FpBGwEkPaeS5Ri4EiKORxrIdlkh07H4DMhIKBSjBth6LKYsqhBGw0AZUHZDFYg8ogAgA4vkCOQpk0YeCdL5AFjLcPkiXA6oDIf2jm4K0ws3TuloW5RC90Q+dXZBHmYJ0qcD5zXmON5Lzv1T9Wj+pLaBEEscLThhgRDK1zYJKjUAGWSVCeRBGoKQR1TjplTRyIZBnqFILDIJhK2lkEWUxZVHNVW2XBEAScpFDOwXtVSYh6UIDuQC5br+kiw3tj5C+0ENBOhWoRzltAfEoI0hHxgDKrTfeSM5\/roVy62L991sAxwtOGHIgkay1xA2ZWcAIZGZ2Kmkt7oQ4kcNIIyd9SRWyziJjgVTfgExVVuoBsFAC9TZCGkGh9FBSAGkA6eRSR0Fd2itIlwJuf+hSrD6RraAP7ZeQdDagDaCNK5N0VODXC30B+vnLyCsXOP+5geKaMG9kuY7CwH5xwkAiATt7EEnW4IE1C58z2CUagYzQEVTSyAnMiex6Go00RuhAmSqBFoAwAhZIFkrAohlYuQCZkASAKIgCoR\/kA\/KpqyBdLqADfaMXIF4IIAuoLPoBms4nRA5AhtsH6XIAHRT0j24K0sBNu3UpKye4Jvg0FR5JI5LlPBLJfUMk2dkjlzUYGJHEGuHIiED29PRIZ2dnuBQrydkCkEZOVPU2Qhq5E+JENtKYs1mrsmE5noNkASwlaQw6sEoUlCS4IfW1nHgyypdydULfXDWhrQIZKpeQdKGhct0+C91HmDztk5A66KIgHQa3jrYNq1vKfNvWLqW1M++L9RfQgrWZNZq4YaAFUhLIBx98UHbZZRdZeumlZdlll\/XiF154oXzyyScDJaXJOfXUU2XXXXdNiXfeeSeNlMoo5oQE06ZNk2+\/\/VZ4GQbiqKSRO6HKGKmNolwWOOS\/KS\/tvNVS0ognpdiexkyUDSIGLnHIREY56qiO6A8y0YF6CuqrDELSxQb9APpBD8JiAfkK+qBfBelsQDvqu\/JIlxN4I7l+zBtZzqMwsG\/WYEgk6zK7gRBJ80YOtFPoKnPppZfKQQcdJC+99FKiFfHLLrtMdtttN\/nss88S+ZlE7rnnHnnhhReS4E\/PmTMnE1GDsg6EEainkUGMGDFCxo8fL3gaAXkGs0CQBaLwCR9IIwudblGz+AXpWqo8JQKE9AlBUJDWfOJRB3qjIzoD4i7IU5BPfQXpckD7d\/UqlB6uTO2HMF\/5yADIcfsgXS6gx177HS7cjHF94dUvly7Wb7IFWJchkuRCIoERSazRi0AC+fzzz8tFF13k1Rg3bpxccMEFcscdd8gpp5zi5X355Zfys5\/9TDIlfC0tLfLNN994bffbbz85\/PDDA0FfXqUK+eFEc0mj\/yPfEMgKGaoNo4wWcLeuC\/23sCGNANLIX4ZhoSsncWSxVWByyICCtII84m5d0lEGOgN0RG8NNU6ackC8zEh0r\/qgJ0gUZBmhrYKmyAXECw3kAuTSJ2EpQZ8K+lVdbFsba0QL6o10iSTrerS0LI82gQTy4osv9rQZOnSo3HDDDbLzzjvL5MmTZa+99hItY\/v14Ycflkz+UVfrHX\/88XLkkUcG4rvf\/a5WG9Th7NmzhTsVgPsbNzjb05yA3NEM6sGZ8hVvAQgjgDSyvQYgjbrIsfCV2gj0CbRfdAGaDgopB5TRFhAfLFB9GYMiqrq7+qnemehKXQX1VQ4h6WJD+3F1KFaf2gchfdC3grSCa41rzryRapFohEokWc9Z11nf29raoqFcmbQYQCAxjLrQf\/rTnwrEx9XtRz\/6kYwZM8bLYlvai6T5+fTTT70ayy+\/vNTW1nrxSvuBNALdovY\/11jw8ZrAqrNAtp\/pycYbyTUfRBr9RmbBI49FEBAvFpCvoA\/6VpDOFG4b5GXarpT10EtBv36dKSM\/6kBvdERfQNwP8hWU0UZButRw+0avQvaPPAVytS9C0qmg3kiIZKp6VlZaC+AEwhlEr9OnT894J5b6lYYBBJLnHHWQP\/jBDzSaFGr+o48+mpQfltCXbpZZZhmvSnd3t8yYMWPQv9kNYQRKGhkcdyeQbk4yQJ7BLFBsC2RLLlUfSCMLFMDrgfdDy8JCFj9AOYsjYaGAPAUy6UdBOh8gh\/aufNLlgupBiA7opyAN3LTWIz\/K8OuM3i7QXesQko4CVBfVNR+dXBnIVajMTEOuR4gkuwFcq5m2s3rFtYB6IxdddFHB6QYH4JG14vYaPekDCCSGUDWXW245jSaFSgTJnDdvHkFKKIH8+OOPvRdwJk2aJOuuu673ZveOO+4omW6Fp+ykRIWcJC5p9D\/XaKSxRAfCusnZAixEgEUJ0sgCxUKVrUBdFN3FMlsZ1Nf2hKRVLiHpQgKZAJn0B4iXCvSnoE90UZAOA3Uoc9uSjjJUZ1dH8oCbF6U4ugF0ytbWWp+Q9sgBxPMF1yfXKjd67BbkK8\/aF8YCjY2N3ouwOI7Y0gZwhMJIj76UAQSSz8qo2mEveWAsrQP71nhYOGXKFK\/o9ddfF17Q8RJ9P+QdcMABcuyxx\/blRDNQ0sgJwpixAZ5GXNlGGqN5zMK1qrwS90WaoNFBGIGSRhYjFqWgutnm6SLJwgkybU9dQH1kKEgXG25fqkMx+6QPoH24\/WteutBtgyyQrk05ytEic4x+AAAQAElEQVRLQf\/oTQjIJ4w60BmgZyqdKVNQlzYK0oUGN3uQSK7lQss2eblbAA4AF0ACHAG+QLzSMYBAugOHXQcZYMiQIYnsTDyQ7vcdjz76aO+N7ieffFK4CMaOHevJuv322+XWW2\/14lH5wRacDHhllTRykkAcR40aFRU1TQ+zQEoLFIM0+jt0F00WVH+5pilTkOe2I11q0D99ujqRLgRUJiHy6EtBOle4MlR2rrIK1Q49FMhUHQndNHG3HukoQ\/V3ddY4IbpTR0G62ODGjxtAruuq9EYW28A5ytdtbZxL8AW4Q6V7IwcQSF7+SGc\/PsujdYYPH67R0JCPkXPX9Je\/\/EUOPfRQ741u3rjefvvt5Z\/\/\/KfwtjeN+Ug5YTkBaQRKGrEHJ4SSRk6ScupnfVefBVJ9A9L1PIa9NMOCUyqrsZDSF4srcOOapo6C8nLD1UV1zFUn2iuQobIJSRcSKtPtr5Dy08nSfgmpiz4K0kFwy7VdUL0o5aGz6qM6k6fQslKHXNf27chSWz19fziX4AvUhESCSiWSAwiku1i1t7djgwGAYGkmxtJ4WPh\/\/\/d\/cuaZZ8pGG200oAoevb333tvL51uRX3\/9tRcv5Q\/jAUoa6dsljZmMkTYGs0AULbDmfx+RNR78t4cC6JdWBAurVorSgqs6hYWqNzqDsHr+fOoqKEOOgnQx4faDDsXsS2XTDyCt\/ROSzhRaHzkg03alroduQPt19da8coe2rV3uIxDcP9wGUAqJhGMQryQMIJALL7xwYnxhf21m5syZiTrDhg1LxHONKFunvSubdDHBAVXSaC\/DFNPSJjsbC3AT54K2ubxlfcj0LeX7m28nI357ioydNMnDNlOK9+dCWWgV6MxiC4iTTxh1oC9AT3QGxINAGdAy2gFNlzKkX4A+oNB9I1OBbPoCxHMF7QHtkU0YBaCLAn3QUaFpQrcO6XICbyTb2uz02bZ2OY9Ect\/sWEIicUjptnZyjVKnCtvfAALJcxXaxSuvvKLRpFDfmt5ss82S8oMSfOqHv2Tzxz\/+MajYy+Mv23iR+M8SSywR\/03+j06K5JLsU5BG7gaUOHJgIbAcZPM0Zm9Pa1FYC0Acs5UYRi75XNbjD98tzUOSb\/IKTSL9C6m72DIW0oT+euRFFegM0A+9CQFxBWnqKEiXG+iCDq6OpHOByiCkPbIVpAsFlUk\/oFBys5FDvwraqU6EpP0gH5BPO8IoAG\/k+EkriRHJKByNfh3gFnAMcnBWEVYCBhBI\/uKM\/knBK664Qvzb2G+++aZ8+OGH3tg333xzL0z1w5b05ZdfLr\/\/\/e+Ftv66fMn9xhtv9LKXXnpp4ZlDL+H88BY3ICsXIglpBEoa6UOJIwcWuQazQFQssNRPrhcXueqlX1RoHjI0ScTHt92WlM4lwaKpoD2LqYK0H26Z285fL2pp9EYnV2fyFJRlg1LUdXVD72z6pL6CdiqLkHQxoX24\/RezP+2HkH7oX0E6E2h9ZIBM2pSiDkQSEsmLqqXoz\/pIbwH1RsI\/0tceHDUGEMhYLCb8BRrUhyjiPZwzZw5JgYDxyR0SvPiyxRZbEPVAnW222UbApZde6uXxs\/baaxN4OOKIIwQZ0vePN7iPOeYYUQ\/kUUcd1VcSHEAiAaUQScIwQBgB\/eE6pp5LGo04YhFDVCyA5xGgz0f\/2FsUpNPBfZEmqG5jY5OMOvFM4d\/nL70ktddfL6uuMDEJlKUDC6SCurp4EpLOBNQF1EUWYRSBbgr0U52Jk08YdajO6AvC9KVMQR3aKUiXEm6\/6FSMvpELVLbbp+ZlGyKDNsgFxMsNtrXX+96WcvDBB4tta5f7aFRm\/wMIJMPcf\/\/9ZeONNyYqV199tay22mqywQYbCFvWSvZOO+00WWihhbw6\/HR1dcm7777rwf3b1xMmTBBeoqEOZA4ZgA+Ir7rqqnLvvfdSJPx1m6233tqLp\/txSaRLJHnTafbs2R5JhTTiKk4mjfbpnXS2tfLyWWCTTbYU4GoAkSQNSQwCZYp9pn5PgKZnfPWFRoW\/mLDsJVd66a6995ahG5\/nxfXHTyj9aV0UWSgV2jaXEBm0Qy4gXm6ghwJd0FHhpolTjzDqcPVHZ6A6EweadutqXrlCdKFv9APE8wEyFMhBvoJ0IeDKo69CyMxFBn0raH\/5VX\/xtrTxSJI2mAUKZYFAAsn3H9m+huhpR0oc8Tyef\/75svPOO2uRF9bU9Iuqra318vTnoIMOkjPOOEPGjBnjZUEk+YC4l4j\/\/PrXv5Yrr7xSYrFYPJXZf0gkcGvzbCPEUUkjzxyMGjXKrWJxs0BZLYCX0cWSSy4pwFUKEpkKbl03\/sLT\/xGgeV9M6\/2AP+RR8wgnzdskTii\/k7RN7m6ZE\/cTzJ\/vur0AP7FEXq6I2oKr43D10jw3pJy0u0iTjjLQGaCjqzd5CsqiBFcvT+fX38pKPW1DSEOVR0i6WFD59AuK1Y8rl34U5KODgjTeSLa1cbjYtjYWMRTCAv2szyeNffqrrrpKXnvtNbnuuuvk7LPP9sJnnnlGfvKTn\/hqi4wcOVIgdOCss84aUL7HHnvIE0884X1EnBOY5yLvv\/9+4SPjbIv7SecAASEZ9KdFEEZeiDHSqBaxsBgW6GltEenuzkk0H853wXWDpzwbYZv0eSrDwlSyxlxxdariRNmii34nJcFMVCxARBc6dwEsgNiUIrQvQiqqDoSk04F6gHoqg3gUgX6KKOqXTifsDKjHOAjDQLmCOrRTkC4V3D5dfQrZv8olRK72SUg6CBBJe1s7yDKWl4sFQgmkCuND4d\/\/\/ve9v2FNOCyPz\/ZASnlJhw+Ib7XVVt7fwsbbqX3lG\/KQar4yrH2oBaq2oHPGdJl+4dky7YA95MMfrCnvr7+SvLf6RJm667byxW+OlK5n\/xtqm3kP3SPf\/OliD7NuulZ4xMIPbfzYYw8I0HSuoRLLsPYd+ywSVlTQ\/I7775AF117iof2OW9LK1oWPBRGkbZBlBWQqaEp\/CtK5QNu7cnORU+g2qg8hslVPNySfckA86kB3dERfQBwQV5CmnoJ0OeHqgY6F0AU5QGW5fWheuhBvJFvaOHPS1bVys0CYBdISyLCGlm8WqHgLxL2Ms\/9xs0zZ\/gcy+y9XSutzT0nXzG8Sw25\/9y2Ze++d0nrqcdJ65m+k86svE2UamfNAnEBecbF8E8f7550ufHVAQR2e2yV0AYlMBbeuP97Q3SUbffOpLP7CI7L70DpZvr7\/En\/v3be96vXXzfDCXH\/0ucx07TseuUc6brrKQ9udN6Wr7pW7i6G7SHqFOfwgQ0FzlU9IulBQeW5fhZKdjRy3f3RSBMlwy2gXVCdqeX6dXb3dsijqjU7oC4hnA9ooaKdjJSSdHgNrqDeSbe1nn312YAXLMQuksUD\/6pKmohWbBarNAjNvvk6mn3GCyPz4lnWawXf99zH5ZP+fSXdbm1eTO3ugL4mR2dPTQ5AARJIEzxwC4i8uthhBSqQily\/99xHZ+puP5ZCRjR5WaaiRadM+lY8+6v30VkrBGRQqefQ\/I5lB06yq6MLoLprZCPC3Qx7IRka2dZEPaEf\/hKUAfSnoDx0A8UygdV0ZmbQrR50wHXUM5dAp0z7REVA\/bByUKbQOIXm0VZAuFCCS9u3IQlmzuuQYgayu422jzdACbe\/\/T77+3RlJtWsXGiNjDjpCFr\/wClnkhNNlyPc3TSrv\/PgjuX\/\/Xwh39DxnBDbd7IdeHf2pr68XhZ9Qah0NIZWKRRcfL0tMWE4mLreKLLfymh60XqnDmmV2L0mX7mLJIgpSdUy5gnranpB0qaD9uboUum+VTYhs+lSQzhZuW5WZrYxi1UcfBX2orhqS55aTjjL8eru66jgIyde6hKSLCdvWLqZ1K1O2EcjKPK42qjwt8MWJRydJaFpzPZnwjwdkzK+OkmGbby2jd91Txl1yrbTutm9SvUmvPS9P3pT+eb8xsR5Zt7FG1t3uAolJsmcySWA8UdfTKUvGsWbXfNmgfa6s0DFPxnR1yIqT1\/Ww8urrx2tl\/7+pc4FMmPeJrDHzNVnh2\/dkbOv0lELU+zj8g1tkmWd+IxsPaZSfrLacfG\/V5WTNPqQUEFTY2Sndn06RzrjntPPl+Dba3N5vzrpVWTwBebqwEgekFaSppyBdLrg6uPrlo4\/KIUSO9kFIuhBAFqAPUAiZucigbwXt0UlB2oWbTxu3LMpx9EY\/dFaQJl9BupTAG8mNL89H2rZ2wvIWCbGAEcgQw1h29VqAZxl5vtG1wOLnXCS1YxZOZLE9zSS781XXS93GyV7GjsceSNTzR1ZtiMnNo+vkH2Ma5KIxzXLRy6fIn585Rsbf\/Ut\/VS+92szX5fyXT5c\/xGbLcV3fyK\/aZshxc6fJ7779UI6Z9ZGMb2\/16vHz1GLD5eZFmokmcGR8K\/uyme\/Ltu2zE3kLz\/9GfvPGxXL188fIma+fJ0e\/e6X89q0\/yAWvnhbv6zTZ+KunE3XdyPJ1Mblx4Sa5d7Ehclm8n3NG1cl18XHcu0iDHNQckyEdHd7Hyd02ofH4dn7H\/XfI3C1Xl5a9d5D5pxwp84\/bX+buuKHM3Ws76Xr71QFNWVTJ1MWWkDT5CtJRgquX6puNfrRR0E7lEZIuFlS+23ex+nLl+vtDD+DWCYtTD\/hlhNUvdz56+nVAf39eOdJ4I8fbn0Qsh+kHVZ9GIAfV4TJlS2GBtr6XTbSvkbv8Quq+s7hAGoFuUTPJcsfeuHsy+Zv1RvDfkB9dE5M\/jKiTxWpjKtoLh\/S0yVkLNclPmmtlrS++8PL4We2bN+XYd\/8si7b1v7hDvmL5zhY5evZU+U7nfC8rJt0yzCebgiGxbmmMEzbiE9Y8Qs5+9SxZae77JAdg8QXT5YAPb5LNv3w8qWyZe\/aTaxYdIks1JH\/jVSvtNqxezo2Ty\/q+fjQ\/LJx\/3gmy4HcnBxdP+1haD\/uFdD39WG9532\/QgttXFPlAiQFjAOkUpg7QerQHmi5FSH+AvlxdSBcSyFYglz4VpLMFbWnjyiQdBahOhOiDri7IB5RFAcxx3Cgz70VBH9MhWhaoiZY6po1ZoPwWWPD2a0lKPD7ja1HSyPYOpNGtUDNpBTcpjdOmSi6LwFGjm2RUfGtb4v+gmAdMuTEe6\/3f0h2TB2JD5O76kTKnpp\/EDYmTw01aZnqVaqReXmzrFPffF7UN8nL9UPmytt7LXufrlwTC6iXiP62xRnlx9GT5pHnxeKr\/\/xafP5FIDO1slWNGNCTSGpnXlbz1vmqcXO4wpE6LQ8PO556UzofuTi4fN15io\/r\/shWFrScdJm898YRnS7Wnu9gSp56WEY8y0BegIzoD4grSCvKoqyBdLqADfbu6kc4HKosQOfShIJ0vXFnaR74yc21P\/wpkqG6EpF1onlvfLS9HnPmOeQ8i+ay9rV2OQxDZPjMhkJFV3hQzCxTDAp8\/\/2yS2A122E2YRJMy3QR\/QSlOgDSr7ttZUtPe5hGfVEPE5QAAEABJREFU2d9+q9le+Hlnt+wxvVV+scGlcvzk38r0xjFevv7sPKyXqC0avzJHdMzVbHmiplGuq19I\/tY0Ro4eOUkghlq4anvvc4N10iynz2oX99\/jzQvJH4eNk6frhnnZG814wQv5+XDoEnLo2ufKxSscICes\/lv5YNhSZHv47oIvpbGrV9Y6jxyR5Nl8pLVDfvBFq2w3fb6cPbv3rXOvUfxn3ab0BLLtTxfEa\/b\/b77wGhl+w90y7G8PS8Nev+oviMeGvvy0sKgq4llJ\/8knI0oLLvqkAjoD6qjehKTJV5COClyd0BVkqxttFLRVmYSkiwGV7fZbjH78MrU\/QsrQQ0E6Fdx62j5V\/VKVXXjJn+1PIpbK2IOkn\/gyNUg0NTXNAkW2ANs0eBqffO655J6ak58rTC7sTcWakuusNH4xj\/j0lvb\/njqrTaZ29kiPxOSzoYvLnyft1V8Yj61Y3+tdXHrrc+Kp\/v9bywI5vPMb2bB9rizS1SHnDFtSjhu9rIdzFlq6v2Ka2FOLjJFLlv2lh6sm7SntdQ0yJL4FPnnmmzJp3kdJret6er2Zq\/XppIUXz+uW7tpa6YnjgXaR\/8zvlJ6Jy8nbce9nLBZL\/Rxka4t0fzJFRUnN+htL3WrreGT79bffk\/eXWytRRmSR154hSImoLrhhSkMKgL+ccfjz8k8XVgI6AqQGjYF8F9RRkE9bBelSwO3P1aUYffvlu31n2x9taeOXSV65wI00RJJ50ryR5ToK0enXCGR0joVpUgYLQBrZmmFCZJuGCXKb\/Q9N0qRnRv9ziUkFTqJ72idOSiQ2ehHh36iRIwkS+F9Ht\/cnAjXjg+HjNeqF4+vYvBaZ0bSIzKkf7uXpzwbd8+Wgthlyzpwpcubcj2SzBTOlJl792ddf0ippw3vGHSRTho6XxVq\/kp99\/E+58pmj5crnj5Xj3r0isC1vXk9s6J8mZnR1S6+\/s7\/68V\/Nldh5V8r+37TJsTMX9BcExLq+mJaU2\/3MEzJr67VlieP3lgm\/2VeWPP2w5HKHbCYVBCRYcEGUFlxXTVcv9HRBPcoJBwPQHT3RGRB3QR7QPOoDTZcjpH9A365upPMBshTIoQ8F6XzgynH7yEdmPm1VhyuuvlnG20s2+ZiyItr2rwwVMZzoDcI0ip4FII1ASSN31BBH1bR2\/ASNemH3Z8nk0Mt0f+bOkdiC\/reha5ZdUaRu4Fbu9M4uWeIn17stpaumLokojoh78KjQGavzvIR+EkkZGNHdJVvN\/1rOm\/mebNBQS1bSCzhehu+nfu3fyrJx8nnxKyfLLtPulsmz3016HtJX3UvW9\/TIaFiqlxKZ1hVLfMdSv2fZV5RR0PP1wE8F1bUvEIANwQBBXV0DslJlsOhSrosd8XJBdSBEB3QDxF2QB6gH3LKoxtEXoB86uyCPMgXpqACd0EX1JZ4ttC0hbZGpIF1ouLK1z0L3ESaP\/hTUcXVh7uQGnPmUMkN1WaCmuoZro61WCzDBASWN6m0Mskfsu8kEsuPOW0U6O4Kqennt997uhfpTu9yqGk0KF62rFV5IcTNJu886fum8mPLuyGXk2NVPlqsm7iGvjlrJbZYUP2V0o7w4dqwMHz5chg3rfdZRK8z+5iv54pP35esvPpaWBw6Vk9+8UIu88K3hy8hNS+4kZ6x0tNz93S28PP355F+HSPeye2jSCxeq6fFC9+fLn4+SJf6wsZs1IN7W1u5tU3\/49eyksto11pWGPX8VDt8zkUmNUyTcRY7FL0XVghfRnwLhqgsh6VTQOm77VPXLXYaefh0YA\/DnRymNfgCdGAMg7kNSkjoKCmivIF0K0B\/9uHqQLjRUPiGy6VdB2gU338ynEEnb1nYtU\/lxI5CVf4yreoSZkkbXSLXfXVJ6moYksrq\/\/Eza77g5kXYjbG8v+Evy9m\/tiqu4VZLiy835MCm9zJwPktKfxLeIyRg37zNZ++tXZKXZ\/5PWuua4N3I\/+eW6v5ezVjpC7l58c+HtaeoBPt1zyPx5ROWg7uQ\/u7hZT5sc091bNjG+3e1V6vt5dJH15ZxVjpT7xm0u\/xu5tIxsT37hh2o98Z8PZET8t\/f\/+Loa6V6wQPgzjKC9rU2+elXk9SdE7h47RK4a0yivv9P7jOO8ucm6sAAt+\/1NegX1\/dYuv7I07nVwMvY4QBp+tp+Hxt0PEKmt7audfUCftGIhBMSLAWQrkE+\/CtLZwG2HzGzalqIuOinoT\/UlJO2WkY4y0BmgI3oT+kE+IJ+6CtLlgNs\/eoFC6YEsoPLcvjQvLMQbadvaYdapzHwjkJV5XCtjVDmOAtK4++67i3obuUPOSlRTsww5\/vSkJrw5vODy86Xny77nIee3SuczT8jcvXcUd9uV7ev6TbdJausmtv3sQe+lFfKauhbI1l88SjSBV9u6vfjk2W\/JEe9dncBqs9+UttoGeWfksnLrhJ3k7gVtXj39+cfNf5Ipbz4lez\/+sGZ54XqHHCSHvP2aV\/bXs37j5enPc588o1EZ1TZL1vz6tUSaSM1SOxPIO32fCfIS8Z9fDu\/9JFA8Kus21Ej7vJi3zT26JiYzYjWep9FdhKjX2NhAILFhwyXmfK6n\/eZrpPujD7wyfrrfe9v7uPi8rdcU0HpM8l\/6oU62cBdB9ALZygiqjxwF5doPIel8oXLcPvKVmU97vx7oB1yZpAF51CccDHB1Rm8X6E85IB4VoA9AH\/QlzAW0VdAemQrS2QIiyfzLPJxtW6s\/uCxgBHJwHS\/TNsQCTFYA0rj8KuvKXvsdnvrTOyFyNLtu4y2lbsPNNOmFHf\/8q8zb44cyZ9t1Ze5268r8Ew9JIo9U2v3JF2XVVZb13kRed42V5MEH7iU7geXmTpGLX\/itnPTGhXLJ8yfISt++lyjDq\/jWlpd5L9lMGZ68jX74e9fInh\/eKlt\/\/rDs8ughsm0\/h\/PaN06c5IU19ckFs2+\/Q7484wxp+e9T0rD4WK+O\/uzRKLLefw7x5J7\/8hkDnoes6cH\/KPK3luTt+z2G1cuNCzfKdQs1yu\/iUHmE\/2rr8d4+ZwEaNnwoWQPQcOCxSXkt++0krcftJ61H7Cktv9o1qax+652S0vkk0AkggwWTMBfQFmhbZAJNFzJELkAmfQLipQL9KegTXRSkw0Adyty2pAcTGIMiynqjI\/plY2utS0hbZChI5wtu2m1bO18rRr+9EcjoHyPTMMQCEEYAaWSyAkxcTIQ0YXIExHNB4xG\/lRpeiPE1dj2ObtHFXY0yf5HvyNixYz2M9L2BrXX5kDdEklDzCK9f5meel5E4nsb\/Lrw20QR++NWTssfUO2SHOPEb6ly5Cx9xhMSG9G6518T7bFh22USbrlmzZM6998r8t9+W5hVWlLpFFk6ULRGXcWiTCHL9ulBpcWklkI86u+XvPhLJVvakuPfRq9D3c\/2cNnlxfu+nf\/qyAoOGH24vdb4\/\/9j18nPS9eYrSfXrtthO6rfcMSmvEIlczg\/OIwU6IENButhw+0KPYvaHfAX9aN+EpDMF9QH1XXmkowLVixCd0FdBHiA\/6lCd0TNMZ\/IV1NM2hKSLAfVG4pG05yOLYeHyyowvIeVVwHo3C2RjASahINLol8GkCMhn0iTMFjVjFpGhf7xJGg89IWXT1hVWl09OvET+\/W2rzJ49Owk9fV48BHzWNFYuXXYf8X88HM\/j75b\/lTztI4zXLb273LbkdkI57f2oHT1aFj3+eFlo12Sv3aKHHip1iyzsry6xoUPlu7+7QOqWXCKprCW+a84LNDxf6Rbs0\/FuIvmHb9vl3Fnhn+g5Y1abXDWvU66\/6Y5Em56a\/umlxvnrOVRoPvF8aTzmtKRnTclXNB13ujT\/+hyRWEyK8Y9zAyCb8wMQd0GegnzqK0iXA\/RPv65epPOFyiNEFv0oSOcDV47Kz0devm3RQYEs1Y+QtELTbl0ti2qIzsDV2Y2jN+WAeCnATb0SSebuUvRpfZTGAv0zfGn6s17MAjlZgIkHTyN3suppzESQTpT+STSTtl6dujpp2OlnMvyBV2TYDXdL89mXS+NRJ8v0nx8mnx15pkw9+1r56oDjZfdD9ku8WOK1i\/8sWLBATp7dLj\/f4DIPx69xkjy78Fpy9Jqny+FrnSHnrHSYHL3GaXLg+hfKawutHG+R\/J9nHu8at7UcvvbZcuLkX8tFy+0vVy79czm5JSZL\/uWvMvGOO2XUjjuKn2QNWXttmfivf8vS990vE\/52iyx1299lod13E\/41LrecTLzpJlniqqtksTPPlMPmifx0XkxuHb+j93zlhp+3yPeG7uDhuKYNaJLAnfPa5fuffCu\/+KpVTpwZH9s3C+Tn8fj3423udzyPauuh510pwx95w8PQ6+5KyPEitbXSsM3OMuKOJ2TIVbdL8ykXSNMpF8nQ6+8SbF2\/1U5Sin+cH4C+VG8NyaNMQToKcPVxdc1FN7e9yiXMRVa6NirX7TNdm0KW+\/tFH5CqD8oBdWhPOJigOjMGRbn0h0gydzOH4wgolx6Dot9BoqQRyEFyoKpRTUgjk83EiROFiYcJCGRrC3fiZEIF2cqQOJF8Y+Y8ebt5IXl3yRWlZc0NZfntd5BV1l3be+bPldfW1iaQRzfPH5\/ZsJC8NXJ5md60sPdXafzlbnpBXZNMHbqEvDxmNXniO+vL890xWXGnvSTW92KKW9eN144aKQ1LTpD6cUtIzbDh\/UV19dK88soy+YiT5f2eGhm91aVeGR8N9yIBP11932Jsi3sE32\/vlIdb2uXB1nb5IB5\/\/KkXEy38tk4UhEUaGqV24jLxLe0tpX7jLaRmiYmCrcOqlyrfHUep+sy2H1fHbM5p6iro05VDuphw+3J1KFaf2gchfWj\/hKQzhdZHDsi0XSnroZeCftEZECefMAowb2QUjkJhdKgpjBiTYhYojAUgjUBJI5NNLqQxSBsmU0BZphMq9RS0o\/2KKy8nE5ZN3gaur69P+sA2dRUQs3TQupmE68\/52qs2ceUNBXiJLH5oA2iydetMggRqltk9ESfS\/f7NBAKBDINXwfeDnchybUc6SlDdCNELnYGIeG+SkzcY4OqsY\/HrTb6CMtooSJcabt\/oVcj+kadArvZFSDpX0B7Q3pVPulxQPQjRAf0UpAFpQuoA4uUGczpOAeZ580aW+2jk3r8RyNxtZy0LZAEII2AyYVIBTDDpxLd1LZBbXrlVbnzpxpS49dW\/yX3v3ifvz3hfOrs7Ex5DJlMQ1A\/5gLKZbTPliTmPyrUfXSvbXbODrHT+yrL2xevIGhetLrv\/dQ855YFTpH1Cj0gNtZMxdOPzRNG86W+lZtudEuhcLiZd42NegzCC+eHde8n7b+wjM+v+k0Bbk8jWLTNl17bZXlvIYDag0X4dc7z2ny5RS1LonwiEEdROvU2AEmPK\/ASSPLDhhmsRDAALF6AAWwLi5QQ6KNAD\/RSkgabdeuRHGaozOqK3hsQBaa1DSDoKUF3QEeSqE20VyECugnQh4e\/AJ7MAABAASURBVMqlz0LKzkQWfSqor\/oQkg4CZYAyty3pcoJ5frz9ScRyHoK8+g5Y8vKSV5jGJqUqLJALaXQN88nMT+XUB06VMx46MyVOuf8UOfLOo+Lkb3tZ5+K15ZrnrpNlVlh6AJHUiZWQflZeZQV5t\/tNOfrJw+WP\/71U7nvnnjgJ\/R9FHlra58tLn70kt77yN2lbPybzNu+R7oViUuO8PCLOv66aWTJ\/4XsS6FqvXjo3qZeGzQ9OkEwlm4Re06E1Qr3pjbeKYn48zyuL\/0Aifxkng4d1zRUQzwr9f2z3PK8O5FErvb1cfYI8ah5hR0eHuGhoaCBb3p\/6RQLvfjhNQHt7u\/fxcP2AuFfR+WHhAmSpbYmXCvSpoE90UZAOA3Uoc9uSHgxAZ\/RkDArSUYSrn+qdqZ7UB9RXOYSkiw3th\/5BsfujD6D90D\/QdCYh9QF1XVmkywl2mnhciTWhnHpY39lZwAhkdvay2nlagAmCiUK9jdyB5ikyq+aQvvMfPU92vHZHmdEyY0BbJlewwkrLyN637isnxcknbQZUDMjoGROTli17pG3puDeyr7zlieNFMf\/Vy\/tyk4PWhflWZH8bLYVENq9xuCYzCo+Jk0RwfE+LuIA8phLAn0AMg7arre31VmpaQ0ikxlOF2JVyFi5AvFhAvoI+6FtBOlO4bZCXabtS10M3oP2iN3HyAPFSIdd+0BmgLwiTQ5mCOrQBxEsN+gX06+pEuhBQmYTIoy8F6VyBDNoiFxAvN664+mbhG76sD7atXe6jkVn\/RiAzs5PVysMCkEagpJG7zVITR7\/6U2dNlV2v203md873PJH+CfWypy6X5z5+1t8so\/SCtXqkZ2RMZrw8Lgn3Xr9oYPuuxs\/l22knJIimEk5CKdG\/lpYWCYOqEEYgtTyTEDsD6rJwAeKFAvKAyqMvoOlcQ5WBbJCrnEK2Qw8FctFR4aaJU49wMIAxoCc6AzeuaeooKC83XF1Ux1x1or0CGSqbkHShgDyAPLc\/0qWC9ktIn+jD+mDb2lgj+jACGf1jlKWG0agOYQSQRu4qf7rHvnn9ZRjJ8N8qi60kB653YAK7r7G7rL746jK0oXmAhM\/mfSbv9bybyGfyAu\/OfFeuePpPiXyNLDp8rJy33fly9y\/\/Lc8f+Zzc\/PObpPa1Li1OCjvWr5VF1pomi6zRj232np5Ux02M+mGdTH1ucfnsvt6tYi2b\/\/IlGi1KOG5apycXcgi8RMhPuvKQZoHZ2BlQqIsH8VxAewXtkasgXSi4Mt3+CiU\/EznaLyH1VSdC0kHQMtqAoDpRy1Od0Ut1Jk9BfhSBfuiFzoB4JqCugvrIUZAuJtx+0KGYfals+gGktX9C0gqIJN5I1hHNszBaFjACGa3jMai1YduBix3SCGEE6mlksgDFHuBPJv9Ujt7kqAR2GvtjOWG1E+XKTa6RDRfbaED3Vz57tfdijRbwks0f3\/qjJhPhhhM2lH\/vc4fsuPKPZJlFlpGRTSNlzXFrytu3vC\/1D3SIdHQn6hLp+U6NnHhRvUcIIYXgrniasiB8M1\/ku\/t\/Id\/duj1RjAfz4ZsWTaRzjdSm+FDQSu\/2EkiVDUl08d\/\/vqBFXrjqChO9sFA\/umhwboBM5VJXQRvkKEgXE24\/6FDMvpBNHwrS2j8h6UxAXUBdVxbpKEF1I0Qv1Zm45hGPMtAZoGM6nSkH1AWrrrqStyNCvNRwdXZ1KpQeyFQgk\/4A8TCwfrCOQCRZX8LqWX55LGAEsjx2r6helTRykXOxc9G7A2SSAOQxgRAWE\/ShoB\/6XmuNNeTqPa+U9SesT1YC0+d+KY++959E+oOv3xfyEhnxSFNdo+y3zIEyasjoeGrg\/5ove6Tuua4BBZe92\/uG9YCCkIzu1evl+X\/UJ0rxXm6+x\/REOtNIQywmExvqZHJzvWw4vEk2HjFENhs5RNYe2hjPa5DF6wc+y6hvW2\/SGJO9htR4+MmQgfUy1SGbeqs6iybHLawtZQrqaDtC0vmi4\/47ZMG1l3hov+OWtOK0X1entI2yqODKpS9FFiIGVHVlIH9AhTJloAvQ7l093Th1gNaLcoje6Ie+gDggriBNPQXpcsLVAx3z1QUZCmSpfELSmUK9kawxmbaxesW3gBHI4tu4InuANAL1NkIaQarB6qThTiip6udSNm3a514z+lJ4GfGfmliN7LLKj+Ox5P\/\/+eBRzZC3vno7EdfIgesfJMPqh4Z+G7Cnp0dqpnRLT0vyizBfxr2KrckOPhUZGp75Wk+S1\/K+S\/oJZWijvgLoKuRwg2FNMr6pXhaqrxPIZF+xjKyrlXGN9bLB8GZZd1ijDNECJ9y0uV72HdaLXwytc0qKFF2wQNpuucbDcm89Kys3xTw7c45oj8SBpv3HVfPzDTseuUc6brrKQ9udN2UkztXF1TGjxgGVkKGg2JVPulBALrLcvkiXEto3If2ik4K0H26ZtvHXiVrar7PqrfmEUdMZfdALoC8gL1NQX0Eb5ChI5wrWF4gkaw5rT65yrF3hLFBTOFEmqdItwEULuIDxNAIu6mzG7U4k7iSTjYxUdceNWzzlFtD3Jn1\/QPNP50xL5L3++euJuEZ2XmWnhMxQnbtFat8d6IX8aG4yqVSZGp42Obn8tk9q5Pmvk\/O0brpwfGOdrDSkUeprYumqyhIN9bL5mGFSK711+es5QY2W+MPGSdlhn+pJqpRFortlrrRffXECnW++HGhrPW8IsxBfsqroBULPjxSaaBtCqiFHQbpYcPugb1CsvlQufSjIUx0ISWcCrevKyaRdqeuofoT0rXoTHyxQnRkDCNObMgV1aKcgXUiw5rD24I189tlnHdEWLbUFjECW2uKDrD8u0HxJY9CQ3cmFiSeoTlge9cF7730QViU0f0Tj8AFlH8\/6NJH30rRXE3GNLDq89zlEv87oQJ3u7m4BPXPjLJIMB1Pm9RI0JyspuuniItssnkwYf\/tyj3QnZyW1CUoMq6mRlZobk4ra457Rjxa0y5utbfJO6wKZ3tGRVD4m7pFdZ0h9Ig8SycfCNQPPKuPSdFg4d7NVwoqyzv\/88y88DyQNsTfhYILqzLkBwnSnTEEd2ilIlxJuv+hU6L6RqUC29kdIOhfQFtDWlU263HD1QUcFehEndOuQjjLQGaAjehMqSANNUw9oupgh3khIJOtTMfsx2eEWMAIZbpuqLuGixNPI5xS42+OurxgG0cmGSQiE9UGZgjq0W3bZSUSzxhIjvpvUhmceIUtkvj\/jfwQJTBg9Qdj6TmTEI\/QN4lGP7EC6QE2At3G\/P3Z4L8bwcswORycTONqDU1ZLZouvzKyR2z+RrP5NHpL89vbMjm55fs58mdreJTM6u+WzePr11g55s6X\/JR06WGf4EFm2LpzkTvjHAqrJwvGZ4nvDm6T7i2kicWLqZTo\/kEjFvM1Wlq4p70vX049J+4P\/lq6Xn5GuLz5zavdH33k72d6LL76Y54FU+2rIsX\/zuRel+\/23pfPp\/0jXa89Lz7SP+wWliPXMnh2v\/6J0PvOEdH34P+lpaUlRO01RZ6d0fzpFOv\/7iHS+HPd+zJ0zoAE6AwrQmxAQV5CmjoJ0uYEu6ODqSDpX+OUgH+QqL6gd8gBl9EdYDtC3gv7RCRD3g3xAvtuGdJTh1xnd0Zd8BelSgnWJ9Ym16lnzRpbS9F5f8WXBC+3HLCCQRu7ouBi5KLk4S2EWd\/LRSYl+iStIr7jicvL6K895uPGGa+X8c04nOwn\/+Pst8vCD98nbb70pCxb0kp+kCvHE1\/Nmxn\/7\/zfVNkgsFkyihjcN7a\/oi6E3WfpXWjrbBz7w2NOY+hLb7FcdsvGunRJ7vRNRCZzwSrdk+vxkc7zVqLra+G\/\/\/3cXtElHwJg+jnshP417JftrimzWXCd4GoGbT3zVhpjctmiz3DV2qJw7vFZafr61zNluPel84iGKBdLoRfp+IOLwy9b9d5bWkw6TtvNOkNbjDpDWn28lLUftK13vvumRbo7rjD23l8XPPqqvZW\/Qdtl5Mmfbdb1nIntzRFZeZLQsfd2FMv6EfaTloF1l\/klHSOvRv5R5e20nc+PouP8OCfpHXy377CDzfvy9eP19ZP6Jh0jrAT+ReT9aTxZcfr70zJ4V1Cw4Lz4o+pm75erSsvcOMv+UI2X+cfvL3B039HToenug51rPD8YKVDD5QNNRCtELoJOrM+lMQBsF9ZGlIF0s0Aey3b5JFxPaFyH9oIOCdDq4dVVGujblKkc\/UK7+0\/XLWoWzg\/XrWSOS6cwVVJ5TXurVLSeR1mgwWQDSCJQ0si3AxViOMTCh0i8TFSBOnqK7u0uuuOwPCbz+2sBFe8qHH8gpJ\/5a9t9nD\/nJDlvLv++6XTrjXiNkgW8XfCvzu+cTTaCxsyER51uPiUQ8MuWbKfHfgf\/XuuFHAvZ95TfS8OuVZdjJa8jvJ44YULGmj8M+tfhQ+est\/xhQfs7vLpUb\/\/ZvqX8rvv0d9xJqhZkLamTCCR2y9eHBXkut117bI37y+OmCTmlLdmpqdS\/8X1uyzKWb62XYsGEeYrGYV4ef0TUxuWLhIbJ4XQ3JBGILWmX+aUdLx+03yfBH3khA4kOIxeLtQaJ2f6T79RdkzlH7SP30Xm9kc1ur1LX3Gai\/miBf+sg\/nst5++0ktHWq9EenfSwLfneytN+V\/NZ0x8P3SOshP5PuT4KPX8c\/\/yqtJx8u0t7WLytFbH6cCNNPYJW4Dq2H\/cLzuGo55y\/Q9GAL9ZpjDCCV\/pQrqKdtCUmXCvQH6A99CAsN5CqQTX8K0rmA9rRz5ZIuN1QfQnRBTz\/I13Li5Qbr1+677y6saeXWpRr6T14ZqmHENkbv4uICU9JYSm9jmPmZhIC\/PCjPXycsPWvWTDn3rNPksksuSlT58JuPEnGNDO\/pfy5yTPNozfZC\/ozh\/I5+kgNpBF5h38+jf\/lIwFeNA1nbZede7f2daIhWX\/UBAZPyG69NkbN3ODeprHuNernw6iuS8oISo30Eb3acaAfV07zZXcl6LhOfBWKxWKgXVtv5wwWXnSvds77uzY576GILLdQbj\/92x8lk3K0ps3+wfTzV\/x\/COO7so71t6rp1N5ba1dfpLyQ2brzUbrCJ1I5fipR0PnZ\/L6H0UiI9TUOkbsPNpG3skn05vUHHnbf2RviNbyvP\/\/3pxJJAWzej+61XpePugaTerUO887knpfOhu4n2I65nbFT\/eClojXtc33riCc\/DSprj6gf5gec0BREE+qMWOgPigLiCNPUUpMsJ1cPVLx99VA4hclQ+IelCAFkAWdoP8XKA\/gF9o5OCtB+UkUd9QLzcwAHCmgaRNG9kcY9GfOkobgcmPRoW4EKKKmnUiYfJyA+sp+XEc8Hfb71JXn3lJa\/pRzM\/9EL3Z0jPkERy2UUHPle5\/nXbed5GP3HURj\/Ys5fsvD4K1qS5veFyiyzjRRjDBx8MJK\/LLru0V87PjivvICssugLRBM585OxEPCzy2YQ9+pQTAAAQAElEQVTkz+0s8BHEoHZzuvp1XTjuaWwKqkTeoovJHtNbZaPPW2To1XdIzdjvkptAx7\/+7sW7vvo8viU804vzM+97W8hHf\/i7zPrRHnHWJzgnyU6Are+Oe\/4hTb9JJs2NO+wmQ874o9Rtuo1Xt+Phe72Qn5plV5QR\/3xMmk+\/WBa+6R6pXWFVsj14nsYFvUS\/7fYbk0hn3cY\/lGH3vSjD73hSmo473auvP50vPKXR0LDtTxcklTVfeI0Mv+FuGfa3h2XWVj9JKpv0xRSPHHMeJxX0JTSf8wH0ZUc6QGeAkugMiJOnIB01oBs6oS8gng1oA7QN8oCmixEiH9AvKEYfQTLpS0E5OgDi6UA9QD1XBulyAm+kbWsX9wgYgey1b8X+Kmnkboy7siuuvtnzkHChl2PQ9KugfyYeBWk\/KCOPNm+88Q7RlFhzrbVly623HVDnwfvvlZa2Frny6asGlC3W9Z1E3rpLJn9onIK2eX1eNhIh+Li5W+5bPLmwqa5RZkyd5dmbkkmTeokm8SDU1tTKST88MamIF3ySMgISHbXJmV2x5HRQqtP3mnd9+\/z4rvGCuNOwn1h67Y44WaZ29gg+y5qlJknT8Wd72frT9c4bXvS9N9\/yQv0Z8dRDssy\/bpDlv\/hQhsXJ1sjbn5Rhtz4cZ5LIQlpvzZZdNu2NhPw27XWQNJ18QS+OO1OkqVl65s0VvIJd77ye1OqtV3vTna++mJTfdMRJEmtojKNB6rfaSSCUkFHAM5tJlf2J1pakbfCa9TeWutXW8Y7p62+\/J3M23jqpRWfI85huJc5pQB7nNSAeVaAfiKp+qfTCzoA6mYyBOgra0FZBulSgT\/pydSFdSKhsQuTSp4J0tnDbqsxsZRSjPkTy21YR1sJiyK9mmUYgK\/Doc6EA3aLGpQ8YajkuciYThasDupBOB+qBdPUo32yLreTk086W7X60E8kEeDbyzIfOlKmzpibyvEhbj4zrWcyL4mE864WBBLO7fYH03DFF6v\/1ldQ+8rXU\/q9FpL1bXtzrXx6e\/cXtctV+P\/RkuD9DZw+RG665Qs4980S55PfneM9muuVBcf484jL1A72gbt3hP0wmyJ\/wFo1ToSmDq5rP\/jhNZNrcVmlpaRE\/oYotvaxbTWqWXzkp3fbhex6Z6hzzHekaNiKprPPR+6Tt\/N\/KvL23k3n77iBtd9wkQ255QEb8J5lsJjXyJeo23kJql10lTuKmyoI\/\/054wWbeDhvI\/BMO9tXsTXKedUzpf7M7tvCiEhs1qrew77f5lAtl6BV\/8zDk3NSPCHTx1nlfO4LuZ56QWVuvLUscv7dM+M2+suTph5GdgOcJTaRSRzinAbXQmzAqQB8FOqGnC\/LcctLlQ\/qe0Z1aQTprHiF1qKsgXS64Oqhu+eqCHAWytA9C0oWAynL7KYTcbGVo\/4S0xYGCI4XdONKG\/C2QwVKTfycmofgWgDACJY1cLEoag3ovxUXOhQvon\/4UpHPBKqskb+8GyYAAgVgs5hV3N3XL\/CW65Kmm5+X2N+\/w8tyf5g\/r5O7Wp70tavJr6uqltmco0STMW69T5jfMk7p3W6Tu4a+l8apP5flnn5FZLbPlgL8fKPe\/e19SfRJdz7TIs888KR9PnSLvvPWGfPnF52SH4ptvvpZfH3ekzPhX+m\/4jDnwcFnkmF5v5cfDkr2GQ\/rG7u9oZlNMZsUxf0iNNNTGEsVvzm+Xtq4u4VNE2E4Lpnd2yQYbr6dJL4w1NCQTxXlzvC3bVVafLMNP\/4PEfM8Feo3iPz2zZ0rH366T1j228j6nwzOhIF6U9J+3sN0MnlNs+fmW0n79pdL94jNJW9NuPeIrrbS8rLr8JKmN60QaxBYfT5AzprzwwoC2PMcJeNkHDKgQt+WAvBQZXBcUc60A4uUAfSvoH70UpF24+bRxy6Ia9+uM3gB9tYyQdJSgOqEryFY32ihoizwF6WLAlU\/fxegjTCb9AS13dcEbCYkEWm5h7hYwApm77crekjupbEijX2H3wuKCA\/46maTdOshQkO\/2QbrYOPWBU2X581aQ62pvka93bpOZ23RIy9qd0j6uf9s0oUPc+wiBTKSJxLd2hz\/YIbH4ti1JF\/Mnd8nX2y6Qb9fpkJZVO2Sf234p6\/1xPXlq6lNuNS8+5M1aqZsb8+KZ\/LS3t8mB+\/5Cnnz8UambH5Pmt3370o6Q1157LZFa+sSzZGpzIulF+Csz7oV93I\/r5Nc\/qZfzt6uTi7evl5mrNHr19Ce26iry\/tQvPGy9Tf9LL4vW1UpTZ4f3ljhvinNc33r2+SSCVr94\/8sstZPXkqHx7eqmY06T2rU3UvEDwtYzj5PN115VNlxnsuzeUpNU\/qe2mJdP2ebrrCoth\/8iqbxm1bWl4YBjZMjvb5CG3fZNKvMS9f1v1JOe7\/MgkpcJ2traPc9q9\/CRSdVr11hXGvb8VTj2+lVS\/UwT7nWCnTNtl289+lIgS\/UgJJ0OWs+Vka5NOcvR09+\/jsGfH6U0OgJ0YgyAeCpQB2gd2gNNlyLU\/tADFKtPZCvog34VpF3gWIFI4mxh\/XTLLJ6dBZJn7+zaWu0yWYCTnpN\/\/KSVJJ2nUTL4515oXIQZNEmqQhsFBSqPkHRUMeKZBqnpSNYuNiXuYWyJybCn65MLNNUYk45xca\/mpG7pXCTZ86dV6maJNL\/fTwDXXW8DWWHFlbQ4MPz7LTfJF46Hsvm9mrhuwQR03rx58vXX\/c9lThvSI\/Nq+glyc22NvLRmnUAcgdvhdxb0yC8\/TL7s\/\/3Ru3LSb46TGdO\/Ev+\/m087KymrIV7XzahdcoKX5APdHU881Ptx7WHDZcipF8nwe16QIRddI\/W77iM9Tf0vKuG1W6pW5NjuefLso3d57fXnpOOPkClvPiWHdc2Vib7h1265kwz9\/bXSuOveUrvqGtL9zXQZ8C\/ufY1NWDaR3TjjC3njpVc9Muhl9vRI66F7yLwff99Dy6929bI5f+fNbfHi\/DQ2Ngjn77Lf34RkArXxLfzGvQ6WJOxxgDT8bD8PjbsfIFIbH1yiRXYR+gToA7JrnXltZANtQZ9A09mEtAO0QSYgHhWgjwKd0FVB2i0jHWX49fbrqmMhpEzrE5IOQNGz6BvQEXoB4vkCOQpk0YeCdDpAJFk\/8UbijElX38oHWiB5JRlYbjkRsQCkkRMd4shJz8lfaNW4+JDpXpSkg6B1CCmnrYJ0lNG4oE5GPVIvDTMHnv77fOdHnuqNX9fKiCdrpTHgr8tIin8N02pkxDP1EuuJyejRC8m\/7ntELrrkCrn6+ptlwlITB7R8770P5dm4V4\/vW7qFxx17opy783luVlL8o4\/63+heUBeTM1dO\/hD50e\/Xy1H\/q5Wxrb3NmuMe1Y2mx+RvT9XLsO5Yb2b8963hXfJofDv7P488KMcdfZh09yST4sWefUhWmbikeMd2maVk5OP3xlv1\/69ddS0v0fn8f2XBaUcn0PHcEyJNTVI7eR1pOuBouY2n2L2avT\/\/uPlPcvDb\/Z7U3lyR7rlzvOhR77wufz3rN15cf866s\/8RhO4ZX0rHk\/\/Rot7Q+3aQSM1yyWR92Zcf9co5V9GzKy67J76lDuY0D+snl16t5J9YnAzHnG359puvke6PPkhU6n7vbZm75eoyb+s1PbQeE+AVTdTOPIK9qY3OgHi+QI4CWfShIJ0vXFn0k6+8fNrTvwI5qhshaQVpQNqtTzrK8Ovs6k6ZIkpjcHVC31x0o52C9iqTkHQuwBvJ2soam0v7am4zcAWtZmtEbOyc0EBJIyd6MYijO2wuREAeFyqhgrSCPOopSEcZY5u\/I5tO2lQu3vH30vRCvdR9O\/DU56WYVW+7Wo5t6vGw\/\/aHyclrnS07Tdw57dBGDxktw5+ukxHP10vNgl7ZRx7zfzJmzMKJtvvse2AirpFFFxojX89I9qQNGTJEttluB9l+xe1klcWSyZDXLu5s\/PDDD4UPpMck5mU9\/B2RR30e0T0+qZG744TxsUfq5MlHG+Ti1+qSyCMNT1upWzp7Rcj77\/1P3vvf\/8hOgGcQ5+66ubQesafM+ckP4p7V\/hdgOhua5L0llvPq1q6wihfqz4LTj5UFl5wp7bddLzdutIpsW68lvWHjxN4XhWrqkwtm336HfHnGGdLy36ekYfGxvZX7fk9cbgk5Z8NVPLnz9t5hwPOQPX3fv2zcZc++Fr1B+63XylIXHi+Ln\/d\/A16+mbfRlr3keNWVZNjwgc++IqHhwGMJEmjZbydpPW4\/zybqwdTC+q130mjeoXttcd3lIpB2CtqrTELSxYDKdvstRj9+mdofIWXooSCdCm49bZ+qfrnLwnRkHOXWLV3\/qiNjAOnqU0dBXdorSBcCrKs4ZlhrzRuZuUVrMq9qNUthAQgj4ETmhAac3KXo2+1DL1C9cAm1XMs0Xa6wbm6NLHx7Y1pcsfYl8vgRj8sVP7lcTnruCoktNmSAypBHvk04Oe7N27y+R9au7fEI2hsvPS+f3\/m+jLmjQUY\/1CDDn6mV5tdrvGcUh71cKxPfXkweP\/gxue8X90rjl7Xi\/vv1cUfJWpOX90D+NutsK\/\/79btJWGeFdWXIkAaKE2htbZWD9ttT9v3FbjLsoVpZ\/81VZc2Xl0+Msz5OfiGPbGU31DbI71c4Xa5c60JZeLeTpHWxcQk5GnE9jppH+Py6P5DaF8dK\/Yz+aWB23DNHmQu2nLvefGUAYRt+3GnS3dDoefDqVltH6jbb1m0mHXf9Tdr+dKHs0CgytL8LWfiIIyQ2pPcY1IwcKQ3LLpto1zVrlsy5916Z\/\/bb0rzCilK3SD8B7\/zkUzm0STy56JRo1Bfp7vt727VLTZK6nXbvy+0Lpn0cPz6f9CV6g9lb7CTL7PzT3kSK34Yfbu99+set0vXyc4JN3Ly6LbaT+i13dLMKEud6QxDXICCeCtRRUI\/2CtKlgNsfuhSrT2Qr6EP7JSSdLbSdKzNbGcWs79cLfRX0Szlh1OHXOUhv8oCOxW2jeYUOWWvHT1pJ8EhmQiQL3f9gk+dM64NN9crRlxM1CqTRtSgXLnDziHMRE0YdS09aVn641fZy7oWXy6ab9X5ih8\/0BOm9z5I\/Fsijlt3dLvLT1pj89ZYr5eEH75cPP3jP25KunRuTxi\/qZOgH9TL03Tppmlon6y+1vowdMVbmfDtbUv2DSIaVf\/bppwOK8Aa6+Hhq\/5a1VoZEapywY9hwmbrfkfL51qk9pjPHT5Ln9jpUZqywqiyzzDI0TaBl3rxEvGbJidJ84nkDPh7O84zNZ18udZtuI3o+cK58+MOfSuO+hyY975gQFo\/Ujh4tix5\/vCy0a+9zh\/Es7\/+ihx4qLlH0MuM\/saFD5bu\/u0Dqllwinur\/39Itwgs0PF\/Znyuy4IZLE8nmQ38jM3bZL5H2Rxr\/7yyZtc2uHvnVsp6aGo1KTU1tIk6k+cTzpfGY00LH1nTc6dL863NEYjEpxj\/sDJCNrQFxBWkFedRVbcLDVgAAEABJREFUkC4X0IG+Xd1I5wu\/PPoB+cqlPXIAcfohLCfQQYEe6KYgrSCPuFuXdJSBzgAdVW8NyaNMQbpUYLcPEsm6XKo+B2M\/\/TNmCbXnUyH+xa+E3Uemq7\/85S9y8MEHe3c75fI0usbQC5eQfL1wNSRPy4iXAm1dC+SWV26VG1+6Uf764o3SulRHElbZfR3Z5fS9ZLez9pP9zjtCzvzz7+XxZ1+Sv9x8m5xy+pkycuQojySEkUfGsPCtf5YZjT1y1YRO2WWFTjl9o075Zqd2mbl9u3zzowUy+3vtMm\/1DmladYSMG7+kBP378Q5be9ndzT0J\/domdQvoWKLHK4NEAi\/h\/Lz09guJNjq+7sZuaWxqinsnh0hPrF+mlhM+M+dFeXLmsx4enf5fAQ\/Fw9uW7JbLDtxR\/vqzLeRf22wgz222oUz78Z4yZf+j5LWjT5W3dviZtI8c7WnQ+PhDXqg\/57TVyrwbH\/D+vvXQ6+6Suh9sI0Nvul+G3fqgDPndlTL0xvtkxN3PSt2639MmHonkHMEb+e4qG8knJ18mQ\/70d2k67WKBWJ3cEpMl\/\/JXmXjHnTJqxx3FT7KGrL22TPzXv2Xp++6XCX+7RZa67e+y0O67Cf8al1tOJt50kyxx1VWy2JlnymFxfjv2oVekMT6W2snreHoOf+QNLxx63pU08Y435+m8DTaX4fc8L0Ou\/Efvh8hPOE\/4azrDH3hFGrb8kac3DagLaJ+QFR87ZQnU1krDNjvLiDuekCFX3S7Np1wgTadcJEOvv0uQxwfKpQT\/sDOgK3RWkAaUAeJRAfoA9PHrS16m0LaEtEGmgnShobLpDxRafip59KegnupCSDoMlAPKaU8YdQTpyRhAOXXHG8m6DJHEyVNOXaLad0kJ5IMPPii77LKLLL300rJsfNuK+IUXXiiffJK8tRRVYxVCL+5oAFvUBxxytOy13+HCX4cphOxcZHDxKmjPRasg7YJ80m590sXEJzM\/FT7Nc8ZDZ8r5T\/xOWlfvTsKjC56UK169Ui596TI597nz5aAHDpZ1Ll5brnnuOpnfscAjCfu+8pskFX+46Eb96ZjInd\/tlB9v0CF\/XqZHPhrXI90jewkflXrqYtK5SI8sWKpbpk2aIR+vNkM6RnZT5OGef9\/lbVFzU0RGZ1O\/fu1r1whYsFFMOsf0eB\/pjsVisvZqK3i48\/bbvG3yhSeMTRoTY7zqtlvl4kuvlQsvuVp+f\/m1A8qp81LTW3L7V\/d6uOWTOwT8dcptAq6Lh3\/4+l45veNxuXShL2TOqmvIgnHjpasxvgccV\/SrU4+X6RecIZ3xuP\/\/6NEL+bMktshiUrvG+lLDFnl8DAMqxDM4P0B3c7O8Nb9b6jfazPvLL893x2TFnfaSWGNDvFb4\/9pRI6VhyQlSP24JqYl7UxM16+qleeWVZfIRJ8v7PTUidXWJIo3oOUlIHnoAaWqW2qWXk\/rvbyn1m20j\/DUdtz11AG1oC4iHIr5lXztxGanbOC5v4y2kZomJgfqEti9AQZCOjAEUQHzRRKAfoIOgMZDvB\/UUlNFeQboUoD\/6cfUgXWiofEJk06+CdDbQdsgC2bQtRV10UtCf6ktImjLCKABv5PhJK3mOHiOSyUekJjmZXSqb2pdeeqkcdNBB8tJLLyWaEb\/ssstkt912k88++yyRX2kRCCOANHJHA7i7YZzlumC4QIHqgB6AdCpQB1CH9oB4lNDSPl\/Of\/Q82fHaHWWNa7dOUu3a1ePbjH05PTU9MnujDnlxtbiHL04U+7JTBrNqZ8u3m3XI\/KW6kurFYjGPICZlOokVfjlZautqpaGhIYHfnXum8G\/SpGUIkjBz5jce+VVbJxVmmWh9\/x35\/MgDPLxx\/BECeVQRM2rrNeqFiy22uDTFPZ9eIscf1ZlzA6w\/52vZumWmTFx5wwQyFe1vs3XrzERTZCvIpF8F6WzgtkNmNm1LURedFPSn+hKSdstIRxnpdNaxEDIO6itIlwNu\/6pXIfRAlgJ52g8h6Xyhctw+8pWZT3u\/HugHXJmkgb+uW6cccYgk3kjW8nL0H8U+S0Ign3\/+ebnooou88Y8bN04uuOACueOOO+SUU07x8r788kv52c9+JnPmzPHSlfDDnQonWhBp9I+Pi4W8Yl8wKp+Q\/ugXEM8WtAO0U3nEo4Sps6ZK68xp0uN8msb1RrYu3xn3LvZ7E7PRvWX1Tuka3pPUhMklKcNJvPHyG9KxQo3MWWJ+Yrt63vg2WX3XFeWD7o9kyJs1MuSVGhn6Sp0Mf65O\/u+Yw+Q\/\/3lQuru75ZOp7zuS0kV7vPEyKkDtjxepkVN2rJUTv98g16xZJ48sWSPdNSIzY7XySX0TVRLYZNPNvfj0eV8Jjw24uOetu72ysJ8ZLTMSbV7veEXAt+2zE9V3beuPu8QwVVwb79cxR7Q95xugjHNQQTpfIAsZyAfEywX6V6ADuilIK9w86mt+mrCsxX6d0VuBYlpOSDoqUH1cXXPRzd8euSAXWenaIBdQj34JSwn6VNAvuihIh4E6lLltSZcTOH5+use+5o3sOwjxZaQvVsTg4osv9qQPHTpUbrjhBtl5551l8uTJstdee4mWTZs2TR5++GEZ7P+UNOLy5kTjhMtkTFwsgLpcMISFALIUyKMPBel8gSxkuH2QLibqZok0v1vr4fujvie7r7G7rL746tJU1zig257ODulo7b8x4W3rhRceI+2ju2T+8kqxnGatIkNfqJNRD9XLQnc3yKgnGmThT0Y4Ffqjc1fvkNr62kTG9ddfLx0dvi+T95W2Lxzf2l6l09vSbu3bhl+wpnjpv756o7SuHC+P50NM567bKbO+3yYHP32YrLHHKvL7C87tk9IfNH7bJSt90OFhtQ86Zc0PO2X89E6p6e6RWKxGuLCBtmirq4mT5R5pje+4vrB+vVywWaPcu8hQaY\/X1Tp8PugXe\/3SS06b\/Znw2ICLo\/99rLw87WWvPOjnq7nTB7QZPW5UUtVfxokg4EPh4JjueUnlJI7vafE+Ms6HxqkDeSRfwTmn0LxChq5szutCys5EFn0Crevqo3lBIfXIpy0gPtjAGECU9UY\/gI7YGRBPB+opqIsMBelig77ow9WBdDGgfRAin74VpDOF20ZlZdq2mPXUG3nwwQcXs5vIy3bXmKIoO2vWLMEbh\/Cf\/vSnstRSSxFN4Ec\/+pGMGTPGS99zzz1eONh+II14n9TbmBFpDBmkXjBcLCCkWsps2imoqDIJSRcayAXIdfslXQw0Tq2ToW\/3YsuFt5Kdxv5YTljtRLlm0+tk2xW2HdBl27yZ8bwegTzGI\/LyKy\/I3PW6iCZhwwkbyjatm0jzp7VSNzdOwtpjUvd1TOTFNhn5ZP2AP2\/YuXCPtCzVIXstdk6SnEIleP5y\/uQumb1pu4y+pz5J7MTPu2STFzo9bP5Kl3wzvEY+XrQu7lnM7JLuXkjEvxV\/wkmny8hRo5L68SfOfuQs6XY8uv7yXNKQSEijIhcZxWjDOQ1KcU5rH4SMhX4VpDOF20ZlZdq2VPXQS0Gffp0pIz\/q8OsdpC9jUVCubQhJlxr0C+jX1Yt0vlB5hMiiHwXpfIAc2iMbEC8n0IF3Fw467HhZY401BA5QTn3K1Xdmq00e2vGcozb\/wQ9+oNGkUPMfffTRpPz\/Z+88ANyojoc\/0jWfewUbDLEBAwaMcWg2NfRAAiQGAiGhBjDlTwgfIZCEUBJ6qAZiEsDUQGJaQg+mF5sOLtjgSrGNwb2Xa59+ezfrp9VKWkm7Ot3dI\/npvZk3b97saMt4V9KVssAOA1o08i+SQgpH77bmesCwQyv4Yr6CXAzM9YilGGuWN32hgrV3SRzIbyydJvHK6uSl62vlmt3PdXVrq9dLQ3WDK9Ph7uVpA0bIySecIyeenPrzL11WdZQtv90c0yTWbSMya93Hcuomf03ShynwqHzZ3usTd0Urfd2+vX1cvtgov0OZO57SNS4PPPyY7HfAQb7+TeWkbz6Vp7I8yjbtW0OffYvtYJ8G+mGALwV\/rKMgF4L6Mf0X4q\/QuRoHLb40PlpkoA\/01Y5+qWPGTNwmxM64glwKmPEQbyExMR\/woX5pkcMEn4BP1gP6xYQ1QdckHq77PG3kJpLeLNPx1t7md9XJISuzjT+5ts02iautz1zzt+hWGr9D52ParCoKRtCikZ2GnSeqoNg5Af\/mTousoAeVsQeVm6PV9YkLwo7h+BN+5Xxzndwf+ZMjnS+bsEbjT\/XEpLK6C2ISr8x41ZV\/du6Jbl87I4adKXvsvJvEEo90h+21v\/znmRflrzffLn+55q8y+oFH5PmX3pT\/3PKcbNSpt05xWgrRfu13cvoUkT\/ueY7T975ULIh5VRJfVC8Vkxtk+KDh0tDAZxcbEalPsa3vHJMlB65P0X\/TIybjt0u+O4lR5zUNcux7dXLB2Fq54ulaOfO1Wtnz01qGUtj0+K3le1v0T9GnU1z7yjWyumZ1uuFWqWefBjaukH2auQq+8Kkghw2+8WmuiVwE3J9TYm3WIxYFOR1qwzxIZ1dKemL2xoMOvPpSkjU+8gxBY8NWYQ5+gH7UsA6wDjHQRglrKKzD2gqywk0kikhqBNW19jYe9QYuW7bMXaJzZ\/\/PknXr1vibdBjyyJu2VOBfFOwQxSoa\/babnRW97sRmi55xBbkUMOMh3nxjmjZtRsrUvn03cYtGHdzl\/iO0K2XVHdy+dr5ePke7MnHeRLevneGDGv8EHXGj+2b+QunYqbvwI+TbbLudcKezoqxChkw9kOEklrZbmCR7hUO++IXU9GrwqqViVlzaTYrJZmP2klgs5iINMRnco7EoNScl1K64ujou83vE5bl9qlyddvourE8Uj7Wy8QqRssST+vaJurPfwgY5fGqDnPxajVTWJBeoE+ZPkAc\/fEinZ22XrF4i97x7T1a71mig+wf7NATdRmwBe3woyFFjrqUxRLkma4CuYa6vuiAt87DDF9AvNYhLITZiBvroaUsd4gXiJGag7wW9whhzFORiw9qsacaEHAbqkxZ\/rKUgp4ObGtxYol6gdkhn11r0kReQS5cudXNVVZV6sWOQD+\/TQqncgaRo5F8TwA7BjkF8pUTGHbpEAiVGQuFABPrZwE7JZsu4WTwix2NlNEl8uWTDX3v5cM4nSWMIG3XaiMaBmAGBOJ4+ZpooPWqS70Bis7Q6cwE5YMUg2WLp9pgGI1FM\/unHf5Rh\/YYl2TsHa31jIfplv3J57KAKWdsulmTD3cuve8bkrn0rZOQBZXLfHs4s16b\/ggY56KNaV9bO6Pfu026g9va37pBvls0LZNvajHbccXv3HzDsH+C3jegVxs15yGHDcQDp\/LI+Y2ZMyGGgPmnxx1oKcr6YPtR3vr7CmkccCj41Rlpk0L5ph76UIWYgRuLWlj4gM64gNzdmLMQI+cbEXAUf6psWOReoF\/giLfVDay4kk68uuWQooG11tefzaD7zVq1a5Wo7derk9ovdoTG24wgAABAASURBVGgE\/vVA0cgtaXaEYseh6+nOTIuOHVlBVj39UsYbs1\/c6BS2RedsvfVWiGlJd8HcrPOmSXO+WzHfeUyMcvqCz2lc+nXrJ\/HEo2tX0dQhhi\/\/kvx4uPvaXk2jG5qlHb7dIKTp7Tn\/sJSR2u4pKlcRS8RzzKCjXNntJIpL336TsiFxmzImsSbJv9nuywbpuLqxEFUL8sMPr6scpL3p9caf5gpiG6YNf3bSS5j+g\/pi\/wDs2Xe1pQ\/IjCvIUZHuOPCuZ8ZCjOC1CSozV2GO+qZFDhP1aa4Xpv9svnRdWmyJR0H2wxzXeX52frpS0GnMuh20pRCXXwzEBoxp3PSDgD1giw8FuVCoISgiqSsK9VWK8yMvIHv12nDBXb8+8SzNJwvmXcquXbv6WESn4o0FisZtB+0u0JxFI1vKzgz0dWemRVZUxg5UX8otMQMxasy0gA62224bmfjxu\/LQ\/aMdrr\/mz6iTeGzMI\/LSi8\/LrtcfIlKb\/DgWQ75tvdD55jVSI+3KKiUW8y+sOrVLfeTNLO460ppsdlrqIbOqfMM\/gExbs991\/YbjQPU1m9dLQ+rNUmf4jYu+lL232tfpmy\/1sQZTzNonO+skJgpyWcLFkOmJZ9ue2V8u+dKjSRZ\/t99FSYqnpjwjbw3PfGd1+vSZSXNas6D7Mfu4Uozt9RaPXtkvBjM+jdvPzqvDVmFM\/dAiRwlrAGuYMSBHga5Bi3\/WVpCDwhxs8QP0SxFiA43NjFt1pd6aMZvb4o2bMYUx5gH9sKGe4IYUhWRruxuZejUMOXs9e\/Z0Pab7azOLF\/MzK41mHTt2bOwYrxR3YKgK6vImatHIGwu8yboDsWMVtEAek1lTYTqxAP10MA6Mm3ORWwLETJxsg1JfXyej7rjVZeKE1MfNs2bOkMsuuVgqH5svlQ\/Mk\/iUFYknt40\/00PxuGztMllTvwbXLlW1lW7f+0WYWYtmuWONnfSvn92\/PGWwY43\/Z3tTDD2KWEVc1m1DSecZaBI7V\/ndjW8sgivX1kl56pNo6b+wXvotqJP+C+ql3XcNsmJxudwSay+3xTs4LEzcoZTEf109dyATKvlyceYCcuerb5AffNu4PvZw47a17s\/6+BWLAwZsiZnDeipXp1fYS6eXJxXkgLuXBTlITGbfVRKi80ibfZi+qUcuNhwDQdcMGrO5TcxRgq4Tpp25NnGF6Rtf+AT6YK6HnA+mD3xDPn7CnkMcCr41TlpTNm3QlzLEDsRI3LRAX0HGRkGOGr0b+c4770S9VNH8R15AmoXfxx9\/7Lth+gPiBxxwgO\/4rFmNF3h8ga9RAKUWjXw2QYtG7zR2KHTmjoYcBboGLf5ZW0EOijlHfQWdWyw74lJY0xszY+hzJbamTipeXSwnLNnbnTpz0Wy3r51ODRuKsR7V3VTttPzpQ7\/Ht4c\/urUzbr4sq0j9vOOHXz0ro+ddaJoF7tcMjsuSiu987bkD6n0Urwfs8XufInWeP7\/YY3mDjHi93mH4GyId32onnT5KfgSvC3XxuWk6e8lMobiC1b8+QU2T2nOnawSN6imdYzLqf3c537ht1ETzeteh5a5j4lOBYhJUDtriwyToPPZTUHtzP0ZnyqYdY6WKN2aNm1YhdtMOubkhHmIwY0TOB\/VBy3x8K8hhYfrUtcLyHdQP6yrM0Zhokf3QMXOen11J6JqC8IsZndJk1ixNaykik68GEaSSvzjDny\/E9ahRo8T7GHvy5Mkyc2bjY64DD0z9hivzgCIS6OdSRFI0Hn\/88cKcdEUjPk3MHYwDxhwrtI8\/BV+6Fi1yIagP038h\/gqdq3HQ4ov4FGQwZbVDnytj\/vVP+eTjD51psxc37k+O0PTSvqF9U09k641SP1c5YV7qnU4meIvILzpMQZ3EBd1HOzJF5DML73D6\/BJPu8\/jAlMWvyn3f\/P7Rr3xWjUjJpVfx+Q\/ne4U8bmbiKn3UbzUp79j2a7pcT4WL8b8v7CGz53vf0YGXdb416GQlW\/uv027vm37u5+QHZ75VE4benrS+L1T7pYBA7cU825jkkEJCRSNfuGgV7zj7JcKY7rP0iL7oWPmPD+7UtJpzMRE3LToFORSw4yNmCFojNgqzFFftMhRomuY60e5nq5DyzqsryAHwbRXP0HmFduG2BTWJm5aQE9bTPbcbbAorHveeefJrKabYtLC\/4tHHX8sFpOf\/exnzjIUivwdbP2b1\/xG5BlnnOGMdejQQQ46KPsPGWviKQjBmex5oWgExikauXXMI2qPWVaRHQ\/Y6SDrhDQGzFUwwaeCHCamX9YM03dQX6wLam\/GpDpviw26SZOm0mSkftN2UrdN6ucWX3zhOVm1bpX8Y9xdKfP71G3s6nbfPPnbzQzc\/nZT4YfggSISvqucKx\/1fD1ptKquSrrV9JLLtn7KwR1saJCOn1Y4TFz0kpSVlblD2qlYUSad36+UyrfXSPf\/VUrV7MThWK+jIvs8uLF4H8XH1m14hOx9FL+0d0cZ\/sFMqfrTX+UbSfja4MrtHfLw087j1orbL3Z12ulVs8G36sx22rSZzp3GvTrvnfRnI1esXyH3vHu3mobePrRfuWyyqNHtxIr28m1dY5I65fEomzlKo8fUVy0kaXU\/Zv9UUmf4a0x79eNv2bxaYlOIhLhpAT1tqUPMQJyZYmZMwZY5CnIxMdc1YwozBtOvrkdbyBo63\/RdiL+w5nrjIU7APy3Qx442SrRgpGUdLRqpX+ijaw34X2VC3rLTTz9d9tlnH8fr3XffLTvttJPssccewiPr+fPnO\/orrrhCunfP8LVUx6rxhTcBkCgSacEsGikc8yka8ePF3PFy2fmwBfWHH1A5ypZ1gPUhyrXwzRoKMmsryEHAftCggVlN6we0l9oDe8qPj2j87UadwGcjrxx7pXyx5AtVNbbrGqRvQ5\/GfuJ1j+8NTbwm\/\/\/9r96XYUcNkd+cO0Ku+vOl8sJzz4j5k1K19bUy8ZinkiclpOoF7eWZhbcn7jBeLP\/+9krp0W6ThDb3\/8cThWGnjyuk\/aTGQnOf674nfo\/iy9ZvOGT9HsVPm\/mZjLw581\/FoTD6tqpBvP9tuaJBtLhqP\/JB77Aj8x4N+\/5ucunBlzmyvtz21u0yd+lcFdO21x2b\/s5oukm\/fLVWfvRe4y3auRXt5KTV5TK9TpxiNt2cIHrdVm1n3\/wvAXNu\/\/OPEzB1ufbJGXPM4wM5DIJ8YcZvHY2FlnFiVEyZvtrQL3XYBmIkZqAP9IE+YAf0mxviAOIwY0TOB3wozMc30A8L\/AH+zLWQi4muTcu6xKQge2EMHfZAPywoFhV8UihSpwB9dKVPbhFuuBrlNi8na37\/kcfXFIw6UQtH7jxef\/31Mnz4cB0K3PLGgE4Is2hUn9qy4wFyph2PMQVb5ijIxYa1WdOMCTkM1Cct\/lhLQY6KxM09ef\/E\/0os1njHrL5dvazZrE7ebveePDH5yZRlq2eWy5Cddnb1fbpsIgdstr8ra2fxrmvktYVvyXPP\/Ff+cvkf5ZD995T33hkvS1YtlZ+P\/oW88NnzarqhnbRKJq96Xb5ZP1Nmr50gy+sWbBjLo9c+EWvFwsbDcrbPo3hZ1+h0xdpV0rfrpo2C8Xrqxb+U5fWrpLZdg2y05abSsXdXp2+YOEXixK6Nd\/FMfdUFf3NE3s8ZM1I\/R7r11hu+EPOTHY6UgRslF\/tXvny1Mz\/bSz5FpNfnnzv2kbVr1hRcROKX7QX67L+0Xii6wasPKuMXsGctoF9sWBd0XWIClb0tY8Ac8I6Xoky8QGzEDPTRKcilBrERE\/EC\/aBgrzAHXwpyVJhrsH5U65h+WUdBrzHQImcDO8DO9IOcD2bRyPxZicfT0FqLRrZRabxSqRRhy+9B3nXXXTJhwgS599575eqrr3ba8ePHy9FHH13QyrxZOPC+kejCxm\/H052QlvWwUZCbGzMWjTHfmJiv4EN90yIXg\/XLv5Ntrxso95Y9IguHr5PFh9XIql1rZX3f1Ltqkrj7SAFpxlVbWysTRo+XWG2q\/ZrBdbLwR2tl2W41smrHGjnl0V\/J0NuGysSFE0wXTr\/95DIpXxFz+vm81LVvkHWJwtc7t+KrmJx\/+hm+j+LjSxutH5\/0uLz4+dhGwXhdvPlKWXrYeocpg2fJF3t86\/RrutW7Vl8s\/kKe99wo7VBZLT2rewUuyMriZfKngy9xfdLhtyRpi8GC774VaahxljL3R0cR4EXn0GLO\/gv09Y4kLbIJRSSYulz6rAHM0bXpK1G0rKPgn\/UV5CBgj53pB7nU0PhoiU3jpt8SIF4gVt0G+n4wrjDOPAW5mLAu65nxIIeF+qXFJ+spyPlgzle\/Qf1orUHLHIpF6hBAbivEi72hnTp1kn333VeOO+44p\/X72Z58YuKNe\/jhh52p+qY6QgQv7HjqVnc8dIqOlVpLfMREzEA\/KNgD9vhRkEuZzuMrJd5YZ7hhvvn6qyIr6qXjuApXl9SpiklN38Rdza3qpbbXhsLLtClfIlI9vcxV9a8aLH0qt3TlIJ2y1TFZtVOdrO2XeB5rTKhYEZdvN13i+yieMTX1K13reol0fKdMurxVIR0+KJOqr8okxvaXi\/PfrNkz5OY3bnX65ku7xe3k\/ntGycibrpKRN18j9\/wj8xdqmLtz351lQEXqF5IYM+l6bOo3ui87ptI0CdT\/X3ny514\/\/2yK83lO9kUcsH8CfT8YUxhnnoLsB0UkeMcoIsGrDyqzLrZmPMhhoX5p8cl6CnI+mPPVbz5+wp5DLAq+NU5alWlNG+RShtjBG7PKtMSPjYLcnJhxEB8UGg8+QP2Ya6iu0Baf+GAdoO8HdYXCuFk00kfX1ih6ARllgocOHep+u8l8o4Ovmd2SHQzU0tz5VFfKLfECMbIdQN8PxhTGmQf0S52qteXS9eUKqVycuotPnfqpE37VwjLp\/GaZYCs5\/Fc5Jy6dx1dIrCEm3bp1l6eef1keevMBOanPtdKjvG9gT9sO3E6u+em1UlvfWEDqo\/g1W9bKun6phWt14vF2bXfjrmniEX7DWkNuWnnl0DpZ36leqr8ql04flEuPp6sktl6kvkJk1My7fR\/F141fJe+Mf1OmTvlUpn46SRYvSv25oib3TrMoMX7xhb+RBU995cjZXnqM+LX0PPd3SWaZisjF7WICSxLt0uqYVPzhD9Kw+YYvQuGotq7xc5H02S+BPvssLdBXkLFRkINCEQlee4pI8OqDyGYcZoxB5mLj\/fzj6CHXCOCLcfVPixwW6o91ICy\/ufhhXYV5xKQgezHHmOcdL1WZuImNmIE+oAf6pQZxAXGZMSMHgTkK9vhSkKPA9G+urXUELetSKHKzCuija8vEW+PG8+aadyP1zc93W3WHosWH7my0KtMyDvRLHWIH4jRjpq8who2CHDXDHvT5830BF+1dvbHsv9X+cstPbpbBM7eR8mX+u\/fQYXvKJZddKSedepacfvi5cungK+SnWwzPukq39t2k07j4\/kQSAAAQAElEQVRy6fxehcTXNvr+zQW\/kx49Gn8sn29q79XlmKx+1ODDdhPlomcukrW7NOT1KJ6fCur2emWgR\/Erd6yTxYevk89XTtPl3TbXR\/Hr16+TEaeeINzJLV8Tk+opZa4vb+cvl1\/iVSXJvz+6QkCVFx5VLnD9j8sFbjm8Qu48olL+MvNGmbn7N2I+iu\/fP\/WOL\/sqvqLahykigTVM8i0i8UHMQJ+4afPh1I9\/70zDFzhCRC\/4B9wTM9CPGtYBXYcYwJEDvKgtPiDAlGYzIT7wBqDb4NWXmqxxsg2QKT7GFeyYqyAXC13zrNOOF60bKBSpKYB+sWJpCes0XgVbQqQ5xqh3I81CMhcXujPTMk93LFpkL+gBPXOAfqnjFzM6pZjxe++osHZ54nFuzyeqJBujdh0pr5\/3uow6+m9y6LaHyi5Ddmd6Crwv5RXtZdPNtpAdBg2WPn02kkkfvifz\/jNdejxZKd3GVkqn8WVSPTHuFEYdPyqTLab0kdfPfk2eP+E5qZqfXCxdfOH5ssvgbV0uefhX8vnFn8mkCydKmP\/po\/irDrvS8c8ad+5+m5Sviof6KH73oXvIwO229w2dn\/FhYMwj\/5RvvplH16F6WlziaX4CaPasGbJ06VLHLtMLRSNksnHGyp1X2Wzz78meeyf\/qUfeW2i0iPaVIhLMVQopIvHDMUfLNgD9dKQbVx\/p5oWtZz3Ab7qYGCsE\/Cr4YT0FOVfMuabfXP1EYa\/x0OJfY9UWHWNAv9Txxm3GyzYo6NWWFrnYUDAqrE3tMGvWLLFFI9nwJ+6vbj1aLSTZInPnQPaiOzMtY+zICnIQTHv1E2ResW2ITWFt4qYF9LQ5UpC5X\/GYzeGWW20tB\/\/wcLn2xr9Jz16p30g258+ZM8\/9ggjbOmPaZLnw\/DPl2quukJdefEFmzpjmPJIuWxGTqm\/KpcOMCunwWbm0+6JchvUfJr0795bly7IXQhSTEuJ\/lWvKAj+Kj69IfZydKRS\/R\/E3jRwld9\/3sPTrv4Xv1HfeeU9G3XFr0tiFv71Erh1+XZLOFObN21Bsmvp8++3bt5dbbrtTKioqnPeU\/RXwx3vrhTFgPGz8ishCCkmNnTi9MSMrjJcSxE08YcWnfmjxi38FOQxMf7pOGH5z9cHaCnM1LlpkE3SAzpyDXMp4YyZ24kWvIBcbrQloWZtikaIRqB3QWdJnIJ5+qHWNsEPwLwq2ip0F6LMjK8i6M9Mi54vON33n6yvMed54iBNYgxboY0dbDLIVj9fdcKvzOUM+awhPv\/CK8BufDzz8qFz25ytl7733dMIkZnAEnxe2DV59ZazceP3VKRZ9+mwihx52eMrvS9aMfdq5w3jUkYemzPFTUEQO22VHv6FAuvKVcamcF3celQ+c2j\/jo\/iTKhs\/K1m5oFy6vFQp1Z+VZV9jXYNkehSPg1NOHUGTxEbde0i3rp2SdBRzh\/34SDl8ux\/LoD4+dy4TNe2cOXOkrq5O4rF40tx8hdWrV8vNN\/7VKR7xwXuqIHthDB37BtAPE28Rie9CikjmEzMQr2LqGUMuJYgJiMmMGTkIOocWe3wpyFHBGvhmXaBfDFgLWIsYFORsmLbqI9uczOPRjpZSjFz7FbbaLBrpo7MEy0A4Z\/RgazW7Ff+ioJDUnYSdSIMyD0jVFdqaPpvzAGJthW3SuGiRvajenOO1CUvOVjyyTqfOnZ3PGfJZQ+ieKGTQmxAzoCPuhQsX0XXp23fD79bccetNrp4OP0g+5sln5bH\/Pi+XXH6l7LjTENQuL65tLNIa+AFKV9vYeemlB2Xu3PEOaLCJxWLub1Sig1wexX965RSZNHKKvP\/EJNlz6D5M92XA706V3csbhDXrGupEErVjhynlaR\/Fd36jXLo9Vyk9n22X8iieu7kUvsphu\/3IfVT+eeKRPOw2cPfEdn6dFAvF3JmnnSinnnCcdBxbJsMm7yg7f7St+5GDimVxp3hctWqVVMQr5OaBf3b4xy43Cty\/5+0C\/9z3TjF5YK87BLBhjn6EoWJB4ynrrTdekXlzZou+50lB+QjYAUMTJzZ+iYp+WFBEQhj+iE9J5y\/IcZNubtR68gysw3bQZgIbUBvmgsrFaFkPWItYgH7Y4FfBN2sC\/XzQuabPfPxEMUdjosU\/sZqg0zH6UcK1XmEdagBqAaCPzpJ7BhrPxrnPa5Ez+Ks1wF+s0Q3gw7KgchQtBw1+OViAftSwjsJaxKAgZ8O0xU82+3zGw7oIEp+SLY4VK1aI+fk97H\/7uz\/Ipptu+Pb0Z1P8C4xYLOYUa8xR3nrrQ+0miqvxcv6Fv5K6bcqkdpuYrO5fk5Y1\/etk3aZ1UtupXirbVckvTjxF\/njpNTLhk4ny70cecnhszCOub+3oo\/hZh+0gd\/WrlV\/vUCdLDqyRJUfVypIjamTREWtl2V41snrLGmmoiAk\/TK6P4im+OsU7SLt27dSd1Fc3ODEe+bvDZN1W9Q41mzU4d1xdI6Mz9+uvpa6q3pmj2zehZrJ8\/uVUmT7tc4cvvpyVNI7d+6s\/kXdXfiRvLn7H4dXv3hJ46ZvXBV6c+6ooHyz8xFixsXv22WdL165dG4Wm17v\/fkdTL3jDfo110P0F21wwi8hc70J6YyJWE+LAhrYlQOzEScxAX0FW0GGrIDcXZgzEF0Yc+FHwp2vQIhcKfgA\/5jrIxUbXp2Vt4lKQTdAjYwv0w8YsGvFNwQi2aCQbhdPqC0gKRoV0sfOYmI+1GY8KDhbAPwcL0A8TfCr4ZT0FOR90vuk3Hz\/eOemKx\/EnPO419ZU1HloMNE5te\/bsgTqF+sSjVK9y+vRpjqq+vl7eGf+2PPHYvx3Z76XOM\/+aa0bJCy+8IStXrpY333xf7vrXGFk1pFbWfD9RmA1JFFppwGbF7rWy9KAaWfaTGplR\/aXcfMuV8ti\/HxT+FCE889STKSFMnz5VPrnhGDlqjxr5+4AGmd+3Qeq7NLh2DeUxqe3VIOu2aJCVu9TKkn3XS02XxjuoGN1+5z2ybNkyug617epldSLGdbvEZP2ucYe1e8WktkeDU0RyR9IxbHqprKpKFJ3izGGeUt8+JlWJwpRH2hJPHZ\/Uebo8v\/xVeeLb5xwe+epJgQdnPSpw\/4x\/i\/JCophsWs5tunXrJjt9+5Ur0+EfAtzZpJ8Luo8wZ2LibiTQD4tcikjWVlhfY6NFNlEd9qZe+x+c9JR2S6YlZiAg4laQ0SvIpQRxEY8ZL3JQdB4tc\/CnIEeB6V\/XjWIdr0\/WUhjTOGiRM4ENYGP6QM4XLRpp8UGxqNd85Ahosy4Tp\/rWt+1aMNKydbrz0CKbDG367Uh2MnY4MMfD7nOwAH45YGgLBT+AH3wryGGBT3yxDtDPl3TFY5ALIGuDrk1coHK6ljt3jHXp2lX47Ub6ynnnnC5\/vOgCOeLQA+SC885WtdvG4xsOEwrIdevWCajB2WdfKttsc4Acd9yvZfnyFaoO3K5av0aeXfC8zN19iXN3Tyfud8ChYj6Kb4g3yHtlr8hV29XL6kShqHaZ2vruIssOqJE1iTuexP6LY4en3EX1m79u5waJl8WdR\/G77jRQdhuynfAXfPputrmfudxz\/yNyy+2j5caRd8vNidbXKE\/lt5dfJNC5oT7Fw4rly1N0QRXsN4C9uU8hF4oWkdqa\/lhLQU8MCnImgtpl8lHMMXM7zXXZDlMuxT4xArGxHbSZwEbBjrkKcrFgTdYyY0EOG69\/1oV81mEeMBe\/tLnAdVthHtdzrvdAH50l\/AzEw3fZ\/B7ZaYiCFuhng53MvBvJzphtTiHj5sGSzwHDHIU48Af0owL\/gH\/Wps2VfIpH\/i4z6wHrEYOCHBTmwy9POjVpCp\/je+3Vl2TJksWO3ltg8hnDf57Fn3Nxht0XikjFVRbQqevUIMv2Xi\/15Q1y+JFHy5Cdd0vytnrbWlmQuLuYpAwocMezoUvMsY7FUh\/FOwPGS32PuNT2j0tlZaUD33geuvMOslmaApKp+p7MnfMlYt6sTtxlnfebMwS+uehc18\/yeLnb185GG28sokKeLXEzlX0D6IeBWTziV8E3ayrIuZDuGMrFR9S26baVbWZtcxy5lCFmSBezV48tNOc2sT4QA\/HRhgG+FPyxhoJcKPjCh7kGsh9coxXGuYZzvQf66CzRZqBVFpCkjJ2INhf0bqRZSOYyP1dbDhZgXpADRm1omcNcBblYsCZrEQfQD0K6C5955zEWS90ly8vLnS9LsC4EWausrCzJjC\/R6NyB2+8kJ558WtI4Ao9ff\/6LE+WRR\/+L6IKvt6aVy5j\/q3FwB3w6FFxedfkScb4dvW\/XvWXE0BGyc\/VOUr4w5vsD4PWdYzL05P3k4j\/+ydlmfRS\/vludrNk29Q5cbE1MOrxfLl3HVshftr1MHv7lP+WsYWd5Q3Dkld+vFR4vI3A3kjYdXV4vF0n8f3m\/Nc7nGVd+b53wGcnDzj8o8Xi7Xtp\/HHfo8HG5tPssLv\/+7z9lXc1amTxpovxt5A3i\/a\/zslrZfkaN7DSj1mGXmbUCO31VJ5ssqpPuKxuk3frGR\/Ff9orLZT8pk0v3rZRHv18mH20el3fLqxJ3X9sluf3+zrtKvOnu8Hcrv5WHPnwoiWc\/fSbJ3issWLXAtZ9Y87HAsvVLnW93s1\/X1NW4417fQeSXpr3k+tK12QdB5TBbYg7TX66+WF9hLtsJ9E3QATrsaVsCZszErRA7YwpyqaAxmbHmGpvOpWWu+qRFDhv8An51TfqKLRo1E83fxkMIodW50EKSDTN3VuQo4GABfHsPGGSFcewU5ObCjMGML108QYpH\/EydOk1G3f2wAz\/VA0f+5Mh0btPq\/++8C5yf+mE+HH5k41+aIW6K1GF77S9X\/\/V2+evNt8tfrvmrjH7gEXn+pTeFeXzrmzkTrkoUTU2Pq\/cbWOuu5VdIvvvnernplzH505HrXDvtVH1RLnw7+pyhZ8n\/+8H58vB5\/5J\/\/nKM\/KHfH+R7tamPhD+omSC19RvWa4g1yIqhderObcvnx6TbS+VS\/XWZ8C3vu24ZKU\/c9rC8dPV\/pMubFSkFal0vkXUDGos0Csh1iW2rqakRv\/+W7lPjfJZz\/ffjsnpIvazdWZzPR9bsViarBtc5OvTc2VybKGzHLHlCBl83WH559c+FO7pen5vOq5cfvF8rh3xY5\/Djj+tlkyUNMnWTmMzrUSaLO8ZkbWXMnbauPC7rE3dbJ29ZJi8Oq5DX90s8iu+efLoacfavXfs5S+fKX8ZemcT\/e\/q38tGcj1wbb+fbFd8l2TO\/W9+uTuGO7UefTEgZxyYof3v977hx\/LHfgaOI6OXUj3+fUrBGtJTrlmNWQck2KsiZwI5xcz5yqUKc3tjYBvDqS03WGNkGyBYfNgq2zFeQi4GuRxx6HaZlbW7ycJPI3mkkG81H8hm5+eIoyZXZQdlRCY4dF+hHBQcMvjlgFGT0CnIpYcZFzH6xZSsemQc61\/SpujBb9c+j6o6dujs\/pzItIAAAEABJREFUQr7NttsJdzpZp+PYbQXoB4HiEbs9t66TARs3FmjIXqrfP969wHPnbKcdd5EX\/viCDOs3LMn0uxXz5dVpr7i62k710lCd7Ldd4o7cwRX7SbxmwyFM4aaP4vnGdccJFa4P7azfNiYVF22lYqgtX+BZkygul+6\/Xro9m7q2LlaXCPm+fcrlqV0qhEJR9Zla87Oc2J10yumyw6Ad6Wbk6pevknqfz05mnJQYZB9JNAX9v2PHDk7xWJCTHCdTRDLFPJ6Q05P7CL4VZpMrBTkXzHmmz1x8RGmrMdGyjsZLi4we6Jc6xAzE6RczOgUbbBXkYsP1FvRXUigWuSYDN3mKHY9dLzUDiVN5qtJqNmSAHZUd1iwkN4yG2+Pg9XrkAPbqSlHWONkG0BjTFY+jh1zjFlPYMl9BLgbmehSMSlRrV5SJ7PHtUU5RoWvz49rHDDoqZclXZrzq6Hh8XtstuXhk4Mj+w+XkX54lPzws9e6sPop\/4qZnhEfc2Lu0F2mob4isiGQd57Oc+9ZIfCFSKm9vH5cvNsrv1MMdzyNPPVbOOOv\/Uh37aCZ986k8leVRts80R7XDDgOdtqW9sG8RM8ch0Id0xyJj2cCPgi1rKMiFYvpinUL9FTqfGAA\/GhstsoIMyNgC\/VLHGzNxg8bNOKhczJaCUWFds2ikj85SOhnI7yxeOvEXLRItJFnQ3MGRC4EDV8EPB66JOcZ4KaNxEyNxey9YFI0KNmpPi9xcUNRFuTbfXuZx+KDNGlfxFqp7b7Vv44DxOvvNJ4XfETzpqfvkhxunfvbx7INGyODBg+TI4cdKukfxm2\/WT3598IbHvOq+rma90+VOZPkv+zp986VD3z4SX2tqGvvxRfVSMblBDp8bE75Y5JAoRsXnDl99R5G6no3zeF1dHZfvEo+gJ\/SLyfjtUu9Odkisd9Cn9XLqW7VyxdO1cuZrtbLrp3VMTeHD6uRH\/CkGHsW1r1wjq2tWe7T5iZt23FQO2vwgl+OGHCvp+MGWP8hvkQyzvMeU11Q\/T8wxBYwzB+jnCscx6Dx8gspht+qbNSFs\/5n8sZ6CHbEA\/UxgA9gwn7YlwjZAsWPX6ykta1MoctMG6KOzlGYGbAGZ4\/vCTm3ejdSdPhc3nGQU5nHQKsgm6JFNe+RShpj1UZrGSeFInzEFubmhmPOLYdzGj\/upQ9WxNnSu6pTid071BtUnXTb0tVc9\/CCnwOx\/\/nHy\/T+fLYMuPVt2vfoC2eScY2XNIUOcsR+88wu5b9FTOsVt62r9P\/eoBu26dpSyXh1UdNuKWXFpNykm447YVmKxWCPxWGI8Jnv12yvRJv\/fvG\/6Zb9yefSQKnlu98pko4REIbqyql7Gbh+XZ3aMS\/tEfdtvYYMcNbVeTn6tRiprkgvoT76ZIA9++FBiZrD\/L1m9RO55955gxlmshvTaSc7Y4UyH24+\/Ta445Iq0nLrbyVm8RT\/sPQ6DrKjnGlrs9XilRY4a1gHW0RjoRwH+FfyzroKchaRh5qEw\/SGXChoXLTERr4KsevrFgGsn6FpcV7m+2qJRM1L6bYspIE8\/\/XThdx2ffvrpZs+q3o3UHd08CNIFx8GpYKMHLi1yJrABbPBBW8p473aYxWOpxb3yoM98Q+KupFlE\/uz21LtmvhPzVG7WedOkmQvaJR41N2lmdjJLMZHNV8UknqwS87\/Dr9pRAF15ZWrBVl+X+kUfbE3a9zZuHzYN1HZv6nibRDH5m33PS\/ksp3Ny4Q5lk31D4k6lo2uSG5tEcZioQWOx1BHG+y9okIM+qqWbxOj37kuSswm3v3WHfLNsXjazpPHJk6cmyQh8K37HHben63wEw+mU6Iv3OCRMzh9A3wSdgp5tVJCbA13fjCuMONQfLf50HVrkQsAH4AP\/QL+5YH2FGIhNQVbQ0TdtkcOGa6WCb66hFI3AdRWdpeVkwP+sXWLxz507V15++WUnqoaGDFdOx6J4L+z8\/KuJFc2DAlkxD0gOUkXHc2mZi73pE7lUIC6\/i5beBWE81FgjdkYROfrBxwVYiiIS6OfCFUdnt15Zm\/yItUNltXR+eZJ0SuCd3WWr7Rw9Y4raaOGospQlqjNXaOzU1yaKtsZu2tfyqqqUsZrN66WhLEXtKCgA\/T7LKYni0jHgxewjJ2hoiAn\/S3TT\/n+7Lxuk4+rk454vGq2pWZt2jt\/ATa\/f5KdO0k2fPtMpDLPtq6V+LCZtlCF442Y7ARPGFORSgZiIhTiBfq4wT2EuPhXksDF9s27Y\/rP5Y01QOzMe1Xlb04a54LXJR9ZrIy3zuW5SMAJ9dJaWmYGSLiD5mZHXX39dTj755JLNLv9q4kAwC0kOPIXAzQMTuRBMX+YahfgsZK7GQKuFotcfn8sibvTYAf1SgbuQ4BcPRSR8doMIYEMRCfThn+PKBIZdXi6\/GJV6p\/KRcTF5fkJMJn8dk7WJx7PMMVm2dpnwmNXUdS7v4oobdert9unMWjSLJgkKyZTiMWFxWuynidfk\/5eVlScrAkqxiris28a\/+Dxj0iXi91nO+lhj4Ve5tk7ar0md23dRvfRbUCd9F9RLr8Tdxp6LNtx51bDKEi6GTE\/9POSXSzL\/YPnv9rtIXTjtU1OekY\/mfuL0M72wr4Lfl2i+XblIJn0z2SHWq0FgxtIZ8sQb\/3XQsS8Wf5FpibzG\/P5xls6Rny3HoZ892wp+Y6WiIz4gnlzOH9gC8wAfQL8Y6FrEAFGuiX+FdVhbQQ6KOQd\/QeeZdhSLCnoKRa6TQB+dpeVnoCQLyOXLl8s+++wjAwcOlFNOOUVmzpxZ8pnWQpKDg58dAPNADHsDTN\/5HuT5xsR6Cj6Ixa945IIF2AB2QN+cj1wK+BWR5mNsYqSIBPrAHfG\/v1opwF9r4VvT6E1mfFcmF4+plF\/cWSmH3lApT3wQl1qjHpq5aLZp7vTLVsaclpce1d1oXPjTh0Hvvo1a9k93nnbq\/5vm69FqkKGtGRyXujXrfC38PsspifuLkvhvy9l1Ur1uwzYlVM7\/272RuNP5RjtZ+2a1NLzZTr77uFquj3WQJ6VKljQ4Js5LV88dSJRfLs5cQO69xR5y4IADMXW5+qW\/uD\/rw91Gd6CpM2DAlk09\/+b5qc\/K0fcfncTvx10kio6d\/cTZrgO+DKW4ygg7fsUjHyMxjzmOQyAMU49cygSJWbeHlm1hjoJcbMy1NaawYsCfgk9dixa5ENSH6T+bP1s0ZstQ6xovyQKSb63OmTOnRWaaAtK8G8kBFeWG5HOQ5xOPnkRomc+6it8FyywcsTfReejUH\/1SgCISKBwhXUwUkTDx2nQW\/vrFq2JyxZMVct2UU4V1YPbi1H8gtW9o7zrYeqPU322cMC\/7nTQc1CxbQZNEfGmD1Fw3I0mXi7Bs5te+5nxpx\/tZTj3B7H7iabK4q0qN08tWxGSZlMlKn88\/zoxXyD1lHeUzabxb2mVV4xzzdfaS1LyZ4\/Qv3O+3NC6TvvlURv3vLucxtauMoLN27boIvGZ36Xcs6iw97mi9OuRSOxaJyQ\/iB8aI2Qt6xhXkUkDj0XjzjUnn0+JD\/dIihwk+AZ+6Hn0TrnEKeq5\/s2bNEq6FyJbWm4HkM3qJbGf37t1l3LhxLo8++miJRBYsDL0byYHEDA4u2qjgAAf8c5AD\/TDAF6gv1gGV\/S5YmYpHnUerfvAP6JoTYlCIg\/j2+Db1NxoZK5Qx\/\/qnfPLxh7Jq3Sr5x7i7Utz1qdvY1e2++TC3r53b375Du2772tB\/CqiiZvUaWfPdEhWdNlYWky637Op8rrPmuhlS+1DTP9QST5fbfR4XqLl6htTcmFqcVc2ISWXiMXz8m1qRxP8dh56XhSsXJ2vqE46TNa4UqzVuMbra5M7Ysmrpe91dMuiyW5IHEtKilcnbllCl\/L9f935y5BY\/TdI\/OO0+GTBwS8l2tzFpUo7CurVr5IXnX5B3330\/x5n5m2c67tiXM3nWcXP\/z2RfimNsg1KK8WlMxEg\/11yb9vhQ8BU1upbGwDVNYW2KRYpG4PqHzhJhBkrEdbxE4kgJo3fv3qJsvPGGi2mKYQkrOJA4oAjRPNiQo0APcnxzoNPmA3MV5qtfWuRs+BWV6ebgExg310QuBromLesRi4LMHULaQth5l13lkEN\/lOLixReekyvHXilfLPF8Xm5dg\/Rt6OPa7\/G9oW5fO+9\/9b68PO1lFZNaisiXdrtfVsz+JkmPUNmlM438evXV0vHvOzt956WhQTp+WuFQWVspfo\/iK1aUSef3K6Xba5XS\/X+VUjU7cfow6sP\/7vQ3WVO\/xnGnLzHjsbX3s5x1xrfLzzz713LLbXfK0T\/7uU512zde\/Z8sXucpTBOjG3XslXhN\/\/9p0xq\/EPPTrX4qfClJLZesXiL3vHu3ijm13dp3k1033zUtg3sPlrIFIitmLJW\/XHaRfPrbX7n+O\/l8IcodzNIJckyls8lUWJrLmvs9xwOY46XSJy7QeIibPjqgX+oQMxBnppgZU7BlDtAvNlzD+GgW6Npc34ACUnW2bTsZSFwB2s7GNteWcoCZdyM5EKOMRU8w5okn23pqS4stPhTkXOFiBkHnmWtpDEHn5mqHf4W5ujYtsheKSPDqg8r7H3iI\/Onyq2Szzb\/nTKlvVy9rNquTh+f\/W56Y\/KSjM1+qZ5bLkJ02FHd9umwiw33+Ws3ZT5wjQ0\/ZWX42\/MfywnPPyMqVKx03S1YtlTPGjJB1C5Y4svlS\/VFcahN3HWtvmCm1f5st5RsbPzhpGmbpxxOFYaePK6T9pDLhv38MulJ8P8u5vvEUs2LtKulc2QlTF\/70Id\/q3n7oENn\/J4fJJtv0k5+ddpLsekDyb0pOmjRB2nWvdOdpZ8BGW2vXt9166y2F93TY93eTPx74pySb2966XeYunZukCyIcuf3h8tDxD6blwDV7S7c3q6TTRxUp7tjnUpQlqCBnQGilEjNxKMRFfIop0zftkEsZtoH4zJi1T8sYNgpyMeFapbAuhSLXM0Dmp\/VoLW0zA41n97a57UXdar0baRaSUQZgnnD0RORdD73CmM6hRQ4CdzcgnS1FJKQb9+p1bTMur00+svqjZT7rKMhBoIiEILamzeX\/u1y2vW6gfLzLNFk4fJ0sPqxGVu1aK2v6+DwHTtx9pIA059M\/uNf+4vfId8k2q2TC4Ony2+d+J3v9Zjc56KaDZehtQ+XtL95mWhLtJ5dJ+UerpeG9pdLw1Rpp+GyVxBb7xJA0K7PQPlHsVixsPI3M9vksp6xrnP\/4pMdlxqLPGwXjtbZ7vby+yTuy\/537uTzf5WWp6VbvWi347lsZP\/0tV9ZO\/cLGR+C8pzNmpH4RSe1of7LDkTJwo4F0Xa58+Wq3X2jnqy+\/kMsuuUjuvzf14wj41ruPxIpcLDIdm9li4PjAhpiBfjFhTYV1iUdB9sMcZ66fTanpiFlj0pjRKTpWzNYsGlmX69Ysz+cakRmjiAT6lraVgcYzf0vb5hYcrxaSbMAG2MEAABAASURBVIL3IEUXNnoS4sTkhbV0nBY5X7JdqCgiIYh\/YgFsiZk2X5gPOh+\/oHI+LUWksurAifm4SDun8\/hKiXv+UAxfKvv79bdKx3Gpd7UcR1UxqembuKu5Vb18tf4rR+V9KU\/cjKye3ni30DsWVG6QBlm3qfH18aaJFV\/F5PzTz\/D9LGd8aZNRomloiCVek\/+\/YkfPxupwuXZEatrXyYuzkx\/Xtyuvkp7Vvdwvw2y1Vf8NE3x6ZfEy+dPBlySN8FuSSYochbVr18pPfnywcBz\/\/Jgj5aUXX8jogf0O2B8ho7ExmIutMS2ULvECzooRB2sorMnaCnJQmIOt6Qu5lNDYaImLmIG+6ugXC\/ZjhTXNu41ct9B5oYgE9BSRQN\/SNjIQbxubWXpbyUHHv+qIzDxokYsBJyoIcy2KSMjkkyISMtnoGPEBJ1NQfbYWWwVbfCjIpUjV2nLp+nKFVC5OPSTffP1V+fqrL6VqYZl0frNM4isa77wF3Y7KOXHpPL5CYmYBt20HiW2e2+PrmMSkY7fOsrZfchFZsSIu3266RPw+y8mYxhnTjtHWdxHp+E6ZdHmrQjp8UCZVX5VJzKgpG2INsnqHDXcjpem\/Dkvby1NPPiIjb7pKRt58jfz+d\/+vaSR9s3PfnWVAReo32tPPyDxSV1cn3B3NbCXOj76bNuyLyOY+imyiY7SmPpd+tmMxF1\/EDMQDucz12npl\/CmMsY6CnC+mD\/zn6yfsecQC6teMEx0yLTZAPyr02kPLGmbRSB9dELieAba2iCQLbYN429jM0txK\/lXHQacHqh7EYUXLyUfBJycmoI+eNgq4cEEm3xSRkMlGx8yY08WNXmEecxTkUqT9+nay\/1b7yy0\/uVkGz9xGypf5H45Tp37qhl+5oFx2nL6t\/HSL4ZLtP77wsemnPaTzexUSX9vou1u37vLU8y\/L2w+Mk\/Lfbimx3u2yuXHHtx24nbx895vSpV1nR6ef5VyzZa2s65da5PEovra7UezGYtKw1pAdLyIrh9bJ+k71Uv1VuXT6oFx6PF0lsfUi9RUiy4bVOHdXm0zdpm78Knn+2adl6pRPZeqnk2Txosy\/bbkoMX7xhb+RBU995frIp8NnOecu\/UZg5vxZUtuuISNVZ\/gXtua+yX6rsdAHZLXx+41VxpsDYmJdYgT6+cJ80Pn4BpXDavEJrAVh+c3FD+sqzCMeBdmLOcY873ghMtcZBT9cf7gOAX10+YIP5toikiy0fuKtfxNLfws5aMO6G8nJRmHL9UREiywizhcL6Jt2yGFDEQmZ\/FJEQiYbxogf6BO3tvQBmXEFuTkoT9yN6\/lElQThqZP\/K6OO\/pscuu2hssuQ3dOGO3TYnnLSqWe5nH3Ob+Xan10tH\/76A7lixz9J53cqpHpiXKqnlEnHj8pkiyl95PWzX5Px546T9t9VJvn9zQW\/kx49ejo6vq39\/E0vyMC3NpetXt\/wrW9n0Oflw3YTZbvrt5e52y1O+izn+r6pRaE0fZbzqsOulM8v\/szh099Oll5vtvf9LOeawXWy8EdrZdluNbIq8Vh75Y51svjwdVLbO9V3ez7LuSLmE6G\/av36dTLi1BOEO7nla2JOnvwtRf5y+SXS0JC6ptrzWU79vOYxjx4jSw9bn5FXJr6sU31b9lcG2IeBPjqgny\/Zjrt8\/TKP2IC+xkw\/CNgr2ONHQY4S1sG\/uT5yVOg6tKzB+gpyENQeHxBkTjobs2jEhmIPuP4ghwU+ISx\/1k\/pZsAWkCXy3ujdSLOQDBoaJxaFOXrSoUX2gzFgzJyLHDZczCCTX4pIyGTDmBmzyugAuaXwl2v+Kps1fSs7U8y8N+UV7WXoHnvLDoMGS58+G8krY5+RE48\/Rg75wZ5y2+XXS+W8uHSYUSEdPiuXdl+Uy7D+w6R3597CnwJdsmSxmP9dfOH5ssvgbV0oJv\/zzIvy5DP\/M80K7vt9lpMCTlbUB\/osZ22v1DuaBFXu+Szn7kP3kIHbbc9QCtOmzXR0Yx75p3zzzTynz0v1tLjEa\/wL0NmzZsinkydhFgp1seTH\/eqU91VBZ+6\/6NGVOhoz8UK6eBlTsGGeglxMzHWJKey18angW9ejRc4XnT9x4qfu536D+NKikRZ7ri+zZs0SQLbYDBSSAVtAFpK9COZqIYlrDnqg74d5omKckwzQDwr2gD3+aKOCIhIy+aeIBK8NsSmMmTGjR1fqbLnV1nLwDw+Xa2\/8m\/TstWnGcOfMmedeKNjWGdMmy4XnnynXXnWF82WNmTOmZZzP4LfzU38HEr0JxaQpF9qvXFOW9rOc+iheP8tZtbY8p+XMz3Lqo\/ibRo6Su+97WPr138LX1zvvvCej7rg1aezC314i1w6\/LklnCi\/+71lTDK3PfqrglPdVMWX62NHmQ7ZjLB+f6eaY8RMzYEurIKsdLXJzo3GYMRYSk+kH30ohPr1zTZ+s5x1XmWuGgo47jBSMwPUFncVmIIwM2AIyjCxG4IODnX8t4to8GXDiUBjTkwotcr7ofNN3vr6yzeMCB5nsKCLBjIcYFeaafezQ5QL+lYDzApldd8OtzucM+awhPP3CK\/L2exPkgYcflcv+fKXsvfeejh9iBoSFCxfRuPTtu4no9r36yli58fqr3THt9OmziRx62OHy4yOS\/9JKzdinnTuMRx15qJomtePnJv\/+IUXksF12TLLJRShfGXfugvJZy4FT+6f9LOfQxKP4Sy670nkUf\/rh58qJXU6U6s\/Ksi7VrX036TSuPOmznOajeByccuoImiQ26t5DunXtlKRr3769HPbjI+Xw7X4sg\/r43LlMPL1+\/pmnhMfeZfHCT4+rV69x\/iGg77O+p7RJgRkCY3zuMYp901gm1C4xA051W+mjA\/qlBnEBcREz0A8K9gpz8AX0o0TXMNfWawQta5tFI310FpuBsDNQ+Bky7Ih8\/MViMVcbi23ou8pW2uFfi95Ckk3lBKIgh4Xp0zw5heXf64ciEkz96CHXmKJwIQViSxowBB3LJWYuzoYL8crmWK79Tp07O58z5PEwdE8UMl4fGjN64qZNxx233pQ0RME45sln5bH\/Pi+XXH6l7LjTkKTxF9c2Pv71+yzfgy+95NhqEYlNLBaTWCzm6PUl6Gc5R+06Uj69copMGjlF3n9ikuw5dB91kdLyKH7TzbZwHsXH4\/XyxMMPSYcp5dLjyUrpNrZSOo0v8\/0s5\/MnPCdV88vE\/M\/7KP6w3X7kfM5SP29Ju9vA3WXu3K\/NabJ69Wo587QT5dQTjpOOY8tk2OQdZeePtnU\/s1qxLO7YfPvtt9KuojrFJ34\/GFshXtDDGfETXV89n6iS3Tcf6qzP+w2OkOGF\/RAymJTc0MSmx6q0JRdcgIB4XwDTbNvAuII98xTkYsGa\/EUYsEVj0Kxbu7Az0CIKyE033dT5zAbF1OGHHx52Dkre39ChQ53tJ1DzhIEcBZycAN+cLGmjhKJRSbcOF1VIN068wDgxA30\/0vlJp\/fzUYiO2CCIjxUrViR9fo85v\/3dH2TTTfvSdfhsyoZvaTuKppdYLCZde\/RokhqbD996q7GTeKWIvPmhh6SystIhoQr0fx7F\/+LEU+SPl14jEz6ZKP9+5CGHx8Y8kjJfH8Xr9uqj+Hvv\/rtbtMYaYlK2IiZV35T7fpZz+bKlku0\/7qKC127u18kFJOPTp30uJl9+MRt1EuvWJv9JxqRBj2D+SPjCRYuTRnv32Vh0v5QM\/7HvQQaTQEPef5AFmpSHEe+nwnS20Qt60wa5lCF+4vPGrDIt49gpyMWGghF0XZ5UcW20dxo1I7YtVgbixVrIrlN4BjhJcLLAEycQoB8VnCTxzYkT6IcJPkF9sh6o7NdykQW\/MXTMB\/qmb2TINJfxqCAWhTWI0aRnz+RCDxuor6ujSWL69MbPP9bX18s749+WJx77d9K4KXw7b8OXR9CPuuYaeeOFF2T1ypXy\/ptvyv874QTnyzaMeenQoaOwhjLk+7skHsM\/Jvc99G\/ZZ9\/95ZmnHpXH\/v2gjLz5rw7PPJX6Zxnxqdu5aOE830fx+O\/Ro6dsudUAzF0+\/2xqxkfxrqHR8RaRlVVVxmhjl8fYVe3aCdD3I\/Eku9E4wyuF4+yb\/5X0iLpnj+4ZZqQOsT9C6kjpaXT\/pSU6fV9pkb2gB\/Q6h34pQ7xAjMQM9NEpyMWGc73C2hSLXA+AGwzoLDYDxc6ALSCLnfEC1+NkwUmDEwiuOKnQRoV50tSTaSFr4UPBj\/qnRQbuogD9dHDRhXTj6k\/XwhbS2as+iI3aZmt1bVpsiUlBzgR37hjv0rWr8IUR+sp555wuf7zoAjni0APkgvPOVrXbxj2f3ePb2KAGF\/3qV3LANtvIr487TlW+ReQpp40QfCkff\/SBnHj80bL30CEy4lcnJO4+fuTOp7PfAYeK32c5GQPvo3h0gP9FixbKzBnTEV1mNBXKPGZ3lU2dlx7\/l8yd9IELNrFYzLmruetOA2W3IdsJf8Gn72abN81obA48+Icy9rXx8sob78ott4+WG0fe7TA2oTMZMGCbxgk+rxSNkM\/7qu7Yz0DlQluOFyjUj3c+26gwpvsvLXIQ1Nb0E2Rec9iUUoyc2xVywTmfcz\/QR9caGD16tPC7kZdeemlr2JzWug2+2xX31VplyWeAEwgnEgI1TzLIUcBFAPI5weocWmLDj4KcDi6IkG4cPRdhoO8l2xr4Bu+8QuUZM2Y7d6XwozHQIucC+YJfnnRq0jQ+x\/faqy+J\/kSPt8CkmKrZ7+6kOQgUkUA\/CNttv4Mcd\/wJEuS\/w488Wk465ZS0pn6P4onTnMCdSFPWfiwWS\/ltxrfefU+HnfahUSOdx\/D6OL6iokKG7ryDbOYpID98\/z1ZuXKlM4f3BJYtXSI333iDA4\/j\/R7FM4H3AugDc4F+UNhXIZM9+yRksjHHcrE152Xqs52gNmwnqJxry1xgnukXubkhHoVYiNMEnTmOHBV6LqdlDT3Pc66nj641sX79ennkkcaPvvAPvta0bW1hW2wB2cLfZU4s5mPtqDeHEytrZDuh6jgt9sxTkHOBCyRkmpPposy6meZ6xzL58trGYqmH0Lbbbu187i2XdcvKkr8kot\/CZr2B2+8kJ558Gt0kePT681+cKI88+t8kPb7KvxsvNQfc55A06BGqEo94KzttLPX73OQZaRTP\/c1v5Zbb7kz7m5Vdu3WX2++8Ry7+45+cCbk8ivcWkNyJdJz4vPBnA83i95pb75AXXnlNVq5aJW++856MvOvepLuoejH66eGHJN3Bpei+9A8XOnc7KVgnT5oojzx0j\/MoXh\/Hm4\/i2X8VwuI9VZCDwj4FmezZxwGbFFuUaTDjS2MSSK1+aJmg20mLHAb4AtaAMHzm44O1FeYTk4JsYuqZY46F1adgBPXHeR1aY9HINnI8T548Wc4991yZOXMmKksLzEDq1a8FbkRbD9l8rM1JCKLMSaYTKidYYH21o0UuFC6ukM4PF10rIKTXAAAQAElEQVRIN+7Vq20mn945psx2Tp06TUbd\/bADP9UD2+8wyDQL1P+\/8y5wfuqH+XD4kcOdeeRu8OBBMmyv\/eXqv94uf735duFHyEc\/8Ig8\/9Kbwjy+9c0cCiyltvfeznxeKCRplfrhrwjE9rxa1ux3v6w9KHEHoPv2OpzS7rr7MLn4kqvk8itvlDP\/7wI57henyI23\/k0e\/+8L8swLrwifjUyZ1KTgUTx58nsU37FjR\/nBfgcmFXhN05zGLCi54KBk+2jh7N\/9UbYZuq8cd\/rZ8v7Hn6Byi0iKQxSxWEwuveJqui7vvjMu66N4YgYm8R4oyLkw8sP7sn7Dn\/0PcvFr2hIbssZLPyjMUZiDLwU5KlgD3+bayFGia9GyDjEoyNlQW+ZDNvts45ynFWy5EUDRCMitlREjRsiAAQPkiCOOkLFjx7bWzWwT2xVvE1tZ+Ea2CA\/8a5WTEMGaJybkKOCEil9OpgoyeqAfBVxsIZ1vCkMwx9PZe+10Tjq9bicttmynghwVrMGj6o6dujs\/Qr7NtttJeXm57HLtSpdc167beKhI+43STvvVgxt+w5Bi7qCDD5QTTjxRzj3vN8JvOvbu00disVja+QxwJ5WWfOXzKJ67lPdVJH+lhSJSwbeXC6Q26ZH3bkOHyfDKZB\/eOSrzKH7IzrupGGnLPglhLML+gR\/yDPTTwbiCDXMV5GJhrkk8Ua2Lb8C\/rkmLnA86F5+Qiw89L9Myj3M2BSNwIwBda4dtbe3b2Fa2zxaQreyd5iTEAWoWklFsIidOiMJ3UJ9cfCGdPUUg6Hg6W2zSjTGX7VSQuYAoyMXCXFMLx7DWjvk8ipeycjn1ue+lfRy\/4oBBomgcZZ5H8eiJm5ZH8T887Ei6SWR6FK\/+vEVkkoMmYePE4\/iR7eJyYFmToqkhxjOqGuTq6nrZNM0Zr6vxKJ54FVzw3tNmg\/1ozLRnk8zqGxp\/k9NUsq+BqTP7+DHloH1vzN64kUH9mfaqa46WOFiX2IB+oeBHEZG0+3C+6xAzMJ91aNNBsahgYxaN9NG1JcaMGSPjxo1z6dGjR1va\/Fa1rWlOp61qG9vkxmghycabJy\/kfOFEqeCDE6gX9NjQFotMF2Ni4IIM9NPZMs5vUWKjoNNtMbdTx5urpagLfe14uaz50fPOI2Dn7t7wt0WaHmsHKVYp0uCkp+6TFzrVu\/xg5GVOkUn+eBR\/5PBjsz6KNx\/H66NotpciEugrr7QvSxSG5fKv6jJ5pKxWdoinvyP6\/XKRf7SvT3oU\/3+\/uSjjo3jiZi32A6Dvhf0E0Nft3UPWnfM9l4bvd0Ut7HeKowjpBZ9eV8QM6IlZQUavIJcKZkxmvLnEp\/Nomac+aZGjQH2zJugaes6lRUehyD\/sgT66tkq3bt2kd+\/eLnzhra3moqVvty0gW\/o7mCV+Tljm3Ug9oWWZ5g5zUlRQcsJUkL0whs6cgxw1XEgh0zpc5CGdHX\/xxltEsj2QyW8xxyjm\/NYbfdiXfupQdebaFIu5OMce+p9\/nHz\/z2fLoEvPll2vvkA2OedYWXPIEKfIlD0HiUsOzofGGmSj9HWjdForws\/uwFe3\/Es27t3HfRT\/8+OPl2yP4nn\/gZDYr2mBfQnop4N9DdKNe\/XZ\/Hnt\/WRiBO+YboNXX0oyMQIx+W0DehNsFPTMVZCLgbke51fQdTn3cg5u60Wj5sO2rSsDtoBsXe+n79bo3UhOZhiYJzhkP8yTMuPmSRI5E6at10+meWGMcbGGTL68F2mzaKSINOd6bc2xUupzV3L0g4+7IVW8fLLbD7OjRSQ\/ou1HrmtR3Cm5zs3FfsezjhOHHbd3HmfmMldtdb9mnwDV+7Xsg+A3FpbO9K\/HGS3+NVZt0ekY\/VKHuImRmIG+gqygw1ZBLjacT+Gs0453lqZYpGgEzr2OsuW+2MhtBtJmwBaQaVPT+gY4mXFSY8s44QF9RU\/KtOj0pEyLnCvMA+apT\/rFgIsrBFnLWzQGmdNcNh9c3DHt0hSR8vMPxCkkEy1FJKSd0DQQf2J\/cajqLMx3aBrLtfErKtPqEncGc\/VfsD13OfN0QtEI5nTzHx\/o2eeAfq54fQeZz3EF2HKsKcgmqscWzLFS7WvMxEfMCjKY48jFhHOnwrpm0UgfncVmoLVnwBaQrf0d9tk+ikjzbiQnQk7OmOpJmRY5DNQXa0AYPoP64GIO2HOxB\/q5kM+FPRf\/GW19BikiwWfIUVFIOo+0E0UkCr8i0ikYKRxDKBpZIywmjvpXWK7S+6GIhPQW7gjvveIqPR32KfYx8AxFKuqxxPEFQRZTO+ZCkDnNaeMXI9sAxY6L86TC2hSKnEuBPjqLzUBbyoAtINvSu21sq96N1BMfj1+iPCnjGwjB76KAPgpYC7jI4z\/fu434AHyUChSRYMbjFI4JBUVkovG9o0jx6NxlbCowHbuAL971Ak4LbMZjZopI5y5q06yTazJ8wLHJJuwmW9Go65n7dDH3D4pV1gaNJWjLHMC+mDGzXhCIScGeWBVkcww5asyikbUoGEHPnegsNgN+GWjtOltAtvZ3OMv2cRLkZIiZ90SJLmz0QhD1RcDrX9flwgu5bpcWnvjNdW7U9hSNils4ehelWDTxjqeRKRhN0piFqqaIBIrIDzpWOL6DFpH7r64TYJ7iOPB78bkLGbRwZB8C3Oq+RZ\/9A+jnAuums9d\/\/KQbz1dP3MwlXqDfXLC+QgzEpiArpg571Yfd6rmQFt88seE8CcgWmwGbARFbQNq9wMkAJ0ZOkgh60qQfFVwI8M1FAOgXCn4UfLGGgmzCxR9MXba++jLXyDYnqnGNgZY1NDb6YaBFYxi+0vp4e5IIpDGgiGSIQpBikiIS0JlQMELneFywhcbx4K8UcJBpBvuL4mdnvgf6vvjZeXXZ1sWvd05YMr4Bf8QM9IsF6wHrEYeCnAnsGGcu0C8UznsKvvQf15wbeWKDzmIzYDOwIQO2gNyQizbf4yTJyZITp3kijSoxXAQA\/\/leBJin4Ad\/CnI2MhUE3rl6occ\/Y+a6yFGj69GyFnEoyFBI4adzafFVNCgiwViQx9iG6HyD2q8wLKRopHjlPQVzLW8\/l32EubwntLxPQL8QssVXiG+dS8yAHEbM+EkH\/hVsWBfo5wJzgDmmP+Sg6HmOljmc+zgHAn10FpsBmwH\/DERaQPovabWlngFOnObdSD25RhV3rhcBvVjQEhPzFeR8yKdIYE3W0jjoRwH+Ad+sqSCnI1sRqI+8sVPS+SqaPlFEUjhCujUpIk3S2fnqE\/4pGmGXM\/pLtsIs133CXNN8j3jvwBynjy5bDNgVE+JmPWID+mGALwV\/rKMgF4LphzWy+eJ8pmDL+Y6CEeijs9gM2Axkz4AtILPnqE1a6N1Is5CMMhHei4DfhQAdaBzmHNUV2mYrGrwXfGJgTeIC+mGALwV\/rAP0g5KuMESvvlgjqL+o7IhBYQ1i08fXyAVhFI344f0D+unItg+km+end7Zlx+2dId1GbfVztc5gDi\/El4N5zqZNMTvzNFZHyPFF59IyVf3SIoeN+mU98Po3i0bGOLfZopFMlAaxWKw0ArFRBM6ALSADp6ptGmohydZ7T8DowoaLAOCXi4AJOsYU5KjIdJH2FiBmPBpvPnHpXFrmq19a5HyhYDRRP+qX9UD1xWhZT2E9YlGQuUvo4Ah5vGjhmJjK+6UkxLT\/5z2HtAZ5DrCd3qlsq1cXRI4ivnTrEiMw7rcN6L1gpzDGfAU5asy1NA7zvMUdRopG4NwWdTzWf\/YM8HexeT+uuuqq7MbWoqQyYAvIkno7jGBKrMsBzr\/YCcs8ISOHDSd+r0\/zwuAdi0rOdLGmIPGua8botw1ee2TsFGT1QYscNawDrGPGgRw26p8W36yrIPtCIeg70KjkcTc4ErZKQsF7BIluxv\/zPkNGozwG2U5gqm4nLXKQuLAziSJG03+6vsbMtoCfHXrQMeaAysVsOT\/xs2SKWTTSL2Ysdi2bgdacAVtAtuZ3N+Rt41\/s3kIyrCW4+Cj45OKjIJtjyMUi00U7XRFA3MSXLmbV02KHvYLcHJjra1xhxYE\/UH\/mWqrL2BpFodcOX+goInUN3hdAnwneW8hkk+sYMSjMJT6gr+T66JoYQec3R8s2AGuzfdrSB2TGFeRiQtGosC6FIucqoI+umNi1bAbaQgZsAdkW3uWQt1ELSdyaJ23kXOHiA8zTiw8tsoIMyGpLv1hkuninK1SIF4iRmE3QAeNAv1TQeDTefOPS+bT4wK+CnBdaSJptwpH6pTBL934kzJz\/814qjiKEF7ZRwZ3GQ4tski0+0zbsOE3fhfbZXnywjQpysfGefygYwRaNxX4n7HptMQO2gGyL73pI28yJOp\/H2lx8FEIJegHCDntzLnIq4Wu4mKfzmktRwDYo6fw1t96ML5dcqy0t26B+aJGjgvyD6d\/749u8f2DaFNJnGxX8sI0Ksh\/eGP1sSlGn20lLfOZ2ogP0xUKLRlrWpFjkXATIFpsBm4HiZMAWkMXJc6tdRe9GchJnI\/WkTt+Ei4yCXi9CtMhBwR6wxx9tschUgJjFAXEpxEa8QN\/UI5cyxAzESNy0XtArjGGvIEcJOYd0a1BEQqb3Ld1cP71uJy3jup20yJnIFKd3HjF7dc0hs52ga7Od4CebdjoeZst5RcEv5xsKRqCPztJKMmA3o8VkwBaQLeatKu1AOYlzMidKPdFzUVHQc\/FRkAsBP8w3\/SM3JxQJxEMMxKcggymrHfpSh7iJkZi9oGdcQY4ScqxkWseMh5gz2WYbYz6onelbdZla4s007jdGEWmu6WcThY41FfzrttIi+6Fj5jw\/u1x1eh6hZa6eYzjP0EdnsRmwGWi+DNgCsvly3+pX5luQXFyUsDfY9Bv2xStdrNnuZvE5PEg3H73GXayYWTMHfE2J2TuADrz6KGSKMMjmm\/cH1E7jyzXXak+LL\/woyEEJErPpi9jNdXR90ybsPmso+Nb1aZGDgC1ga\/pCzhUKRmAehSIFI9BHZ7EZsBkojQzYArI03ocWH8UWW2whoBuiJ3wuBKD6KFouXIBvLl60UcLdIci0BoWDF6+9GXMx4vauH0QmLgV7Ygb66GmjRHOYbQ0KL\/CzI15gLFPMjCnYMkdBzhViDzqH2MG0Z21kMybkMFCftPhjLQU5X0wf6juIL84RCvZ8tlrPIcgWmwGbgSAZKK6NLSCLm+9WtRoFo8KGccJXkLljwIWAvnlxQI4CLl745cIF9MMCfwo+s91lxMYLBYUJ48QM9PFP29wQh0IsxKcgAzKtaYccFpqnbP4ouiCbHePEDGbM2qc1bbBDzhfiDzo3U\/zEAfgiRqCfD8xVmI9fBTlM8Is\/cz1kEz0n0KLnfKHnDz5bjc5iM2AzULoZsAVk6b43JRtZpqLRGzQXAi4KZiHptQlT5sIF+Mx08WI8GzqfFlv8Klz0TRjPFYoMhYIUWAty9VWoPWsq+NLtpEX2gzFgjLm0Y5FnNwAAEABJREFUhaL5yOSn0Lx7Y0ZWMq0bZCxI\/KYftsWU0\/XN+HLNNfagvk1fqouiNddhfQpFE9Y0i0b66Cw2AzYDLSMDtoBsGe9TSUVJQagEDUwLSez1IkI\/KrwXr6DrcKFTmKN+aJHTEbQQSDdf9RSRkGshovNzbc1tZS7bCfSDovZeX0HnYxdke8kxYJ8PGh8t8824kQshSPyF+Ne5Zsy6HTpmtowp6JmnIBcTPgsNuiaFop4\/6KvetjYDNgMtKwOtrIBsWclvi9Fy4TDvRlJMRpkHLpr4Ny+myF6848wDr10muZDiJp1fLUy0TWeXi163lZZ5bKeCnA\/mfPxCED9Btou8QhB\/XhviUBjTOGlVpjVtkIMSJP6gvoLaETtgT9y0QF9BxkZBLjYc26DrcuzTv\/XWW5M+L43OYjNgM9DyMmALyJb3nrX4iPVupFlIRrlR5kWUC6yuRV9Bp3a0yPmQb6ETdC0tWMw2yFzdTlrs2UYFOSxMn7qW13fQ2MkleOdnk1lXwVZjokX2gh7Qm\/OQvZix0\/eO5yLns22mf7+Y0SmmbbH6FIwKa3KHkcIRkGmBPh+FoXWxHZsBm4EWlQFbQLaot6t1BauFJFtlXnSQo4ALK37NIgGdwlgYFFoY5BoDhYwXfOh20iLrdtIiR4muwdrAWhoj\/UyQP8hk4x1jDYUx1leQg2Da40vjNdsgftLZsE0m6eyC6IkPvLZ+Oq9N2LIeu7T4NotG+ui8UEQCRSR4x61sM2AzUPoZsAVk6b9HxYqw2dbhQmLejdQLUVgBcVFV8EmhQAvoacPGLBToh+0\/mz+KHj5LaYLOJJuPQsfJMxAD62bzR54gm505zvsHqmM9UDloS3wK8ULQudns2CbIZpdtnO1UsGU7vaDHhjZKOEYV1qFQ5DgG+uiCgD12FJFA32IzYDPQMjJgC8iW8T5FGmVDQ4PU1tZGukY253o3Ui8+XJyyzck2zoUU1E4vtshm37RhLAooIDKRz5rZfosym08tmDK12XxkGle\/mWzMnGSyM8d4vxT0+l7SImdCY\/K2mebkOmZuE\/1c55v2up206NlGBdkLY+iwB\/phwnEJ+ORYpQAE+ujygfmQz1w7RzNgW5uB4mfAFpDFz3nJrPjiiy\/KMcccI1tuuaVsvfXWTv\/GG2+Ur776qtli5EJUyN1ILpoKG8EFVUH2whg6cw5ysaHQyES6eCgiId14oXpvoeUne9dQG68emVgVthddEPT9wTd3B03QBSXIWoXY5LJN6dbRbaXFhn1UQc6GaYsPyDYn0zgFo4IdxycFH8cqcljgMyxf1o\/NgM1A9BmwBWT0OS7JFW6\/\/XY588wz5cMPP3Tjo3\/HHXfIcccdJ3PnznX1xe7o3UguVKzNxYs2HVwgFWz0AkqLnA3sADvTD3JQorajMAEtvmjNNb2yORZ131u8ZVov38JP52XyXawxM9e8Jyb5xqD7HS0+2B8V5Hww56vfoH445hTmUCxS4AHHJzqLzYDNQNvOgC0g2+D7\/95778lNN93kbHnfvn3lhhtukCeffFIuu+wyRzd\/\/nz5+c9\/LsuXL3fk5nrhQsUFi\/XNixkyF0QFWS+WtMj5wFxgLr5pSwFiUYiHGL2gp7ABs6Chz5gl3Axongv1ar6v+NL3lX5Y4BNf3rXQmegxRoveLBrpo7PYDLTCDNhNyjMDtoDMM3Etedott9zihN+hQwe5\/\/77Zfjw4TJ48GA56aSTRMfmzJkjL730kpTCfxSR5t1IvcBxYVTCjBOf+Mt2wcUmSrzrExf4rYkeGGMerUIRaaJ622bPgJk37ZNnIM+Q3UuyBXMURvClIEeB6Z+1dQ2OJQUdhSLHG9BHZ7EZsBmwGfDLgC0g\/bLSinVLliyRd955x9nCn\/3sZ9K\/f3+nry9HHHGE9OjRwxGfffZZpy2FF70bqYWk+ZctoojPe8E1L7pRrKc+WUdBp3HQImdD7Uwf3jlaCJmt16aocgkuprnJFFqQXOt8fT9o0TFXQS4WuqZZNLI2x5UtGsmExWbAZiBoBmwBGTRTrcSOzznqpuy3337aTWpV\/+qrrybpS0HQQpJYvBdBdGGjF1z86sWffpjgV8GvrkmLnCvMA+aZfpHToQWTtunsWrs+1+0nz0BeyDWtgqygw05BLjZ6vNCyNncYKRqB4wqdxWbAZsBmIGgGKCCD2lq7VpCB2bNnu1uxzTbbuH2zM2DAAFdcuXKl2y+lDhc97poQExdEoB8VXPjxbRYEyPmifmjxgX8FOQxMf7pOUL9aSGmbaZ7aeFudw+cFtV+qrcaeb3zkmrnkWUEGxoB+seG4UFjbLBrpo2up1NTUtNTQbdw2A60iA7aAbBVvY\/CNWLZsmWvcuXNnt292unXr5oo88naFEutw14RCUi+EXCijDJEiAFiDIoE2V5gHzMOXghwVrIFv1gX6uUKB5Z2DDrx6lSkcQeV82zB8pFub+CHdeFB9urxq7oP6yd0udQbHgcIoxwfHCdBH11KZOnWqjBgxQgYNGiT8A\/iAAw6Qyy+\/XN5\/\/\/2Wukk2bpuBFpsBW0C22Lcuv8CXLl3qTqyqqnL7Zqd9+\/auWKp3IN0AEx0uilwcE10xL5zIUUBRABQNkG0NbBRsmQv0iwXrAesRC22+UHCB33x8K4yzJqSzxyYIFJGAn2xk82fOz2abaVy3kxY7ttMEnY7Rjxpz39djguOCftRrF8P\/W2+9JT\/60Y9k7NixsmrVKmdJnqg88MADcuyxxwrjjtK+2AzYDBQlA7aALDDNLW16dXV11pD15Ixhp06daFoEXCzNx9pRB02xwBoUCUBfQVbQYasgNxcagxlb0Fi08PLaqy9axnQNWmRF52urerPVMbPFD2DHGkA\/HeZcv366eUH0rK1gT1wKsgl6ZNMeOUy0aKTFL\/s\/x0FrKRrZJuDJyVlnnUXX4ZJLLnF+eoyfINMvAp544okyefJkZ9y+2AzYDESfAVtARp\/jklqhV69ebjzr1693+2bHvEvZtWtXc6jk++ZjbS6qEGXQFAnAGloo0CKjV5BLCeIiHmIF+rnAHIV5+FOQg5BrcWf6Z+0ga4Rlw3qAP42DFjkT2AA2zAf6hcA+reCHYpGiEdj\/0bU2HnzwQfeuI3\/s4NRTT3V+eoyfILvrrrvczX3ooYfcfoCONbEZsBkoIAO2gCwgeS1xas+ePd2w0\/21mcWLF7s2HTt2dPstqcNFlbsxxGxebJHDhIIAvD61aPDqS0kmRiAmv21A7wU7UD3zQeVitLoecUBUa+JbYQ3WBfq5wjxgHj5pc0H3YVrmsX9TMAJ9dK2Z5557ztk8\/uzqoYce6vT1ZYsttnB+yxaZnx5bt24dXYvNgM1AxBmwBWTECS4195xsNaaPP\/5Yu0mt\/oA4H1BPGghbiNgfd2O4wJqFZFhLUgSA+qM4UNAxBvRLHeImRuIF+iboFPTYK8jNgbk+sYUVA74UfOo6tMiFon7MNdL5pFhUsKFQZH8G+ujaAnyR77PPPnM29Yc\/\/KHTel\/23HNPR8XHb6ZPn+707YvNgM1AtBmwBWS0+S057\/zFGf58IYGNGjVKvI+x+QzRzJkzGZYDDzzQaVv6ixaSbId5QUbOBb3o0zKPYkBBVkyd2upYqbbemIlbIWYdp0UuFTQeM9ZcY9O5tMzFp4IcNqZvXdNcw7uP8g+gtlY0mvn44osvXHG77bZz+2bH\/IfxokWLzCHbjygD1q3NgC0g29g+EIvFhL9Aw2ZTKPIh9OXLlyMK32g844wznD5\/5vCggw5y+q3lhYswF2O2x3uRRucHF3iFcb340yJnQm3M+Znsm3uMOL0xsA3g1ZeSTHxATGwD0M8ENgp2zFeQi4GuRxy6P9KyNncY2V+BfwCha6uYvwTRpUsX3zSYP0lmC0jfFFmlzUDoGbAFZOgpLX2Hp59+uuyzzz5OoHfffbfstNNOssceewiPrOfPn+\/or7jiCunevbvTb00vXIy5KJuFpHf7uKArjOmFnhY5F5gDzDF9IhdGOLM1Jlo8EquCjB7olzreuP3iZVtAx8w5qitWS7EIZ512vLOkWTTSd5T2RZYuXepmwSwUXWWiY\/66hFlwJobs\/20GbAYiyoAtICNKbCm75fcfeXxNwahxauHIncfrr7\/e\/VC6jre2VgtJtouLOFBYADotLGiRCwU\/gB9dg35zQQyg6xMbqEyLDPSxBfqljjdm4laInXEFuZiwnymsS6HIP2iAPjpLcgbM4jB5ZIO0Zs0aV2ipX\/xzN8B2Wn8GWskW2gKylbyRuW4GJ2V+\/mLChAly7733ytVXX+2048ePl6OPPjpXdy3Wngu33o3kThBQXES1QerbLGiiWsvrV9ekZYxYFOR0mDY6N51tKevN7Sh2nGbRyNrsd2CLRrKRmR49ergG6e4umncpTXt3ou3YDNgMhJ4BW0CGntKW5ZAfCt93333luOOOE9q2+K93vRupF3Mu9lG+i2YhE3VBhn+FbdK1aZFzQeeY\/nKZH7WtxkXLWsSrIKteRBAjh\/1IYTH+oULRCMiWYBno1q2bazhnzhy3b3b4prbKtoDUTNjWZiDaDNgCMtr8Wu8tKAMUkHpxNy\/8UW2CFjcUNhDWOvhS8Knr0CIXAj4AH6xB29wQBxAHsSnICjr62AH9KND9hhb\/uk+xX\/EPFXSW3DLQp08fd8IHH3zg9s2O\/hlDPoLD38g2x2zfZsBmIJoMNE8BGc22WK82A6FkgIs9d4twpoUA\/agIq7ihMAKNE7+gcpgtfoH1IEzfQXyxpoI9sQD9dDAOjDOXNgzYRxT8mUUjfXSW\/DPAZ7b1lyPGjBkj5s\/64JUfDn\/00UfpCr8cUVFR4fTti82AzUC0GbAFZLT5td5baAa4W0QhSQFgFgdRbQ6FDeA\/l+IGW4W5+FCQo4a1WMOMATkKdA1a\/LO2ghwUnYMfCDrPtNN9ghY9+wn7C9BH11ppju0aPny4u+wf\/vAHmTdvniPzE2QjRowQfkAcRVv6\/Dbba7EZaM4M2AKyObNv1y75DFAMmHcjtWCIKnCKG3xT2AB9L+gVxpijIBcbc23iCnN9\/Cn41bVokQtBfZj+s\/nj\/Qe1Y9+wRaNmI7p2t912k\/PPP99Z4J133pG99trL+SkyfoLsjTfecPQUmfwcmSPYF5sBm4HIM2ALyMhTHPYC1l+xM6B3IykWWNssIJDDhsIG8KvFjdmiZ1xBLgWIhzg0Vvr5oPNpmY9fBTlMTL+6ntc\/77fCGP+ooGgE9g10lugzcO6558o555zjLmR+oebkk0+Wa6+91h2zHZsBm4HoM2ALyOhzbFdoJRmgWKBoYHPMggK5GGixQ1uM9fJZg9iAuRRkQD8I2ILa4gdUjrLVdVgf9P2lZV2zaKSPzlL8DFxwwQXC38V+\/PHHnZ8eu\/POO2XcuHFy6aWXSnl5efEDynVFa28z0IoyYAvIVvRm2k0pTgYoIs27kVpkhLU6BQyoP29xo\/pSbokZiNHcFuCIxfAAAAnISURBVGQTxhT0zFGQiwm\/AaqwLoUi7zXQR2dp\/gxUVlbKkCFDnJ8eO\/jgg6V3797NH5SNwGagDWbAFpBt8E1vw5sc2qbr3UizkCzEuRZRtPjRIorWlOljA\/RLHY2feIF4aRVkbBTkYsM\/AEDXpWAEWzRqRmxrM2AzYDOQmgFbQKbmxGpsBgJnQAtJJlCEAP0gaBFFi70WUbTIfjAGjOk8+qWMxkuMGjM6BX2x4X1SWJt\/CFA0ArLFZsBmoNQyYOMptQzYArLU3hEbT4vMAIUHRQjBm4UJsgkFlIJeiyha5KCovekr6Nxi2pnxETOwPnraYqLvCy3rcoeR9w34hwA6i82AzYDNgM1AsAzYAjJYnqyVzUDWDFCEUIxQmGCshQp9CiagDxRSQD9fmA\/MN30jR0FQn8SiMIcYgT5o37RBHwW8Bwr+eW94j4A+OovNgM2AzYDNQO4ZsAVk7jmzM2wGMmaAwoQCBSOzeKFwUhgLC3ziqxgFGev4oWvTMk5MCrIXc0zneG3ylTXntPjQ94P3hD46i82AzYDNQBvKQCSbagvISNJqndoMiFCw6GNtvt0bZU68BVnYRZlf7KyhMK4x0CIHAVsw\/QSZ52dDwQg6Ru55D2zRqBmxrc2AzYDNQHgZsAVkeLm0nmwGUjJgPtamuIEUoxAVFGOAS4oy2jDBp4Jf1lKQ8wUfzDV9I2eDfCrYUixSNAK5R1cS2CBsBmwGbAZaWQZsAdnK3lC7OaWZAQob7ogRnVnwIEdBvgVZuli8hR3+IZ19Pnr8AXNZj9YPzR8t4+SWghHoo7PYDNgM2AzYDESbgbZSQEabRevdZiBABrgjRpFjFpIBpuVtQjEGOKAgA\/pBwV5hDr4U5KhgDXyba1MsKoxRKJJLoI\/OYjNgM2AzYDNQvAzYArJ4ubYr2Qw4GdBCEsEsipCjgIIM8E1RRpsOxhVsmKcgFwtdk8+OkiNdl4IRbNGoGWkLrd1GmwGbgVLMgC0gS\/FdsTG1iQxQCJl3I81CKYoEUJTh1ywQTRk9MnYKcrEhDwpra47oW2wGbAZsBmwGSiMDtoAsjfehpKOwwUWXAb0bqUUShVN0q4mYhSEFI7Ce6mmRiw3brbA2dxgpsEFzhH6LLbYQoG+xGbAZsBmwGWi+DNgCsvlyb1e2GXAzYBZJZiHlGoTUoWCEkNwV5Ea3kxZHZtFIH50JxSSgo4gE+habAZuBtBmwAzYDkWXAFpCRpdY6thnIPQMUSObdSC2ucve0YQYFo4KWu4wm6Mxx5KhgexTWoFBkm4E+umxgC9jZIpIsWGwGbAZsBoqfAVtAFj\/ndsW2lIE8tlXvRmpBRcGVqxstCGmZ6y0Y0Sk6hqz29MOEbQD1SZFMEajbqPpcWuZjb4tIsmCxGbAZsBkobgZsAVncfNvVbAYCZ4DiSoskii\/INJniT8FOC0Na5Gyonekj25xM48SrYKfbwzZRJKMrFHxBoX7sfJsBmwGbAW8GrJw5A7aAzJwfO2oz0OwZoEDijh2BUJDRmngLPgpBMG2C9pkH2Hv9ossG8SnYmkUjfXQWmwGbAZsBm4GWnwFbQLb899BuQRvIAHfsKCQpwrRAMws8ij4ljHSYvlgnk0+NhxY7YiRWoI8uP+wsmwGbAZsBm4FSzYAtIEv1nbFx2Qx4MsBn\/W699VZXy49sA8Weqwy5o74pIsF0T8EIqqNgBFs0akZsazNgM2Az0HozkLGAbL2bbbfMZqBlZICiUSFiCjQl02NtbMOCIhLwR8GoIFMsajzIFpsBmwGbAZuBtpEBW0C2jffZbmULy4BZNBK6X5Gmj7UZN4s65DBR39ztxK9ZNNJHZym5DNiAbAZsBmwGIs2ALSAjTa91bjOQewYoHpmlRSMtcjoYN+9GUvClsw2qx4fCHApF1gH66Cw2AzYDNgM2A203A7aAjOq9t35tBvLMAEUa5DJd70aahWQu89W2rReNo0ePdv5U4qWXXqopsa3NgM2AzYDNgE8GbAHpkxSrshloqRnQQpL4zWIQOR1qR4sNRSgFbFu707h+\/Xp55JFHSIHU1tY6rX1pmxmwW20zYDOQPQO2gMyeI2thM9DiMkABSCFI4BSGQF9BVtBRLDIHKELRtRXq6upk8uTJcu6558rMmTPbymbb7bQZsBmwGSgoA7aALCh9dnI0GbBew8gAhSAFIcUh\/rRgpEVGzzjQR9fWGDFihAwYMECOOOIIGTt2bFvbfLu9NgM2AzYDeWcgnvdMO9FmwGagRWSA4lDvRtKnYAT6LWIDIgySPETo3rq2GWh7GbBb3GYyYAvINvNW2w1tyxnw3o1sy7kwt33MmDEybtw4lx49epjDtm8zYDNgM2AzkCYDtoBMkxirthkIKwOnn366883ep59+OiyXmfzYsRwy0K1bN+ndu7dLRUVFDrOtqc2AzYDNQNvNgC0g2+57b7e8CBmYO3euvPzyy85KDQ0NTmtfbAZsBmwGbAZsBlIz0LI0toBsWe+XjbaFZGDdunXy+uuvy8knn9xCIm59Ya5Zs0b8aH1barfIZsBmwGag+BmwBWTxc25XbMUZWL58ueyzzz4ycOBAOeWUU+zPwjTTe82d3+233178yPQbj80Url3WZsBmwGagxWXAFpAt7i2zAZdyBihO5syZU8ohtonY7McF2sTbbDfSZsBmoBkzUGIFZDNmwi5tMxBCBrp37+5+o5dv9z766KMheLUucs0A36a+4YYbxI+ysrJc3Vl7mwGbAZsBmwFPBmwB6UmIFW0GCs2A+a3ejTfeuFB3dn4eGaiurpbhw4f7EovF8vBop2TNgDWwGbAZaFMZsAVkm3q77cbaDNgM2AyEmwE+LvDJJ5\/Ixx9\/7LBgwYKsC2Cj9rRZJ\/gYrFixQm688Ub5xz\/+4TManmrChAnOOjxRCM+r9WQz0PIzYAvIHN\/DSZMmOSdJTnp8YSLI9GXLlrlzmMfn5ILMM21efPFF5yTG3+w19UY\/lO6oUaPk1ltvlZqamlD8tSYndXV1vt\/q5RvXrWk77bbYDOSSgfXr18uFF14oRx11lAO\/e5rpHMcYXzBT+zvvvDOX5VzbkSNHyh133CHz5893dVF0Onfu7KzDX24Kes6PIg7r02ag1DJgC8gc3hEKiN\/\/\/vfOSZKT33XXXRdo9iWXXOLOufbaayUezy3t8+bNkzPPPNM5ifHZrkCL5mk0c+ZMp4C899578\/TQeqfxQ+B+3+rlYth6t9pumc1A5gxUVVXJzTff7BpNnDhRRo8e7crezl133SVTpkxx1B06dJArr7zS6efyMnXqVLnnnnucKb\/61a+cNqqX\/v37Ox+FWLRokdx0000RLWPd2gy0vAz8fwAAAP\/\/CudJ3wAAAAZJREFUAwAIHB32zVgioAAAAABJRU5ErkJggg==","height":205,"width":328}}
%---
%[output:4c577f48]
%   data: {"dataType":"text","outputData":{"text":"Minimum robot-to-obstacle clearance: 5.0 cm (must stay above 0)\n","truncated":false}}
%---
%[output:41c405f3]
%   data: {"dataType":"text","outputData":{"text":"Five-block animation finished.\n","truncated":false}}
%---
