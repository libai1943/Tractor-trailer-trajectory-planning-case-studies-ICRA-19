function draw_trajectories(NE, NCNC)

% Plot the obtained optimal trajectories for the tractor/trailers.
colorpool = [237,28,36; 0,162,232; 34,177,76; 255,127,39]./255;
% Load the optimization results.
load phy.txt
load x.txt
load y.txt
load t.txt
load tf.txt;
t = tf(1,1);

load AX.txt
load BX.txt
load CX.txt
load DX.txt

load AY.txt
load BY.txt
load CY.txt
load DY.txt

phy = reshape(phy,length(phy),1); % Get the steering angle profile
x = reshape(x,length(x)./NCNC,NCNC); % Get the x coordination profile for each of the NCNC parts of the tractor-trailer vehicle
y = reshape(y,length(y)./NCNC,NCNC); % Get the y coordination profile
ox = AX; ox = reshape(ox, length(ox)./NCNC, NCNC); AX = ox;
ox = BX; ox = reshape(ox, length(ox)./NCNC, NCNC); BX = ox;
ox = CX; ox = reshape(ox, length(ox)./NCNC, NCNC); CX = ox;
ox = DX; ox = reshape(ox, length(ox)./NCNC, NCNC); DX = ox;
ox = AY; ox = reshape(ox, length(ox)./NCNC, NCNC); AY = ox;
ox = BY; ox = reshape(ox, length(ox)./NCNC, NCNC); BY = ox;
ox = CY; ox = reshape(ox, length(ox)./NCNC, NCNC); CY = ox;
ox = DY; ox = reshape(ox, length(ox)./NCNC, NCNC); DY = ox;

hold on
axis equal
grid on
box on

xlabel('x axis / m');
ylabel('y axis / m');
title('Optimized Trajectories of the Tractor-Trailer Vehicle');

set(0,'DefaultLineLineWidth',1);
for ii = 1 : NCNC
    for jjx = 1 : NE
        APP = [AX(jjx,ii), AY(jjx,ii)];
        CPP = [CX(jjx,ii), CY(jjx,ii)];
        BPP = [BX(jjx,ii), BY(jjx,ii)];
        DPP = [DX(jjx,ii), DY(jjx,ii)];
        PP_ = [APP(1),BPP(1),CPP(1),DPP(1),APP(1);APP(2),BPP(2),CPP(2),DPP(2),APP(2)];
        plot(PP_(1,:),PP_(2,:),'Color',colorpool(ii,:));
    end
end

set(0,'DefaultLineLineWidth',2);
for ii = 1 : NCNC
    xt = x(:,ii);
    yt = y(:,ii);
    plot(xt,yt,'Color',colorpool(ii,:));
end

load Current_vertex;
for ii = 1 : (length(Current_vertex)./8)
    APP = [Current_vertex((ii-1)*8+1,4),Current_vertex((ii-1)*8+2,4)];
    CPP = [Current_vertex((ii-1)*8+5,4),Current_vertex((ii-1)*8+6,4)];
    BPP = [Current_vertex((ii-1)*8+3,4),Current_vertex((ii-1)*8+4,4)];
    DPP = [Current_vertex((ii-1)*8+7,4),Current_vertex((ii-1)*8+8,4)];
    PPX = [APP(1),BPP(1),CPP(1),DPP(1),APP(1);APP(2),BPP(2),CPP(2),DPP(2),APP(2)];
    fill(PPX(1,:),PPX(2,:),[0.5,0.5,0.5]);
end

axis tight;