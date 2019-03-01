function generate_video

car_n = 0.25;
car_l = 1.5;
car_m = 0.25;
car_b = 2;
car_b2 = 1;

NCNC = 4;
global colorpool
colorpool = [237,28,36; 0,162,232; 34,177,76; 255,127,39]./255;
%colorpool = [255,174,201; 34,177,76; 0,162,232; 255,128,64]./255;
tf = 1;

load phy.txt
load x.txt
load y.txt
load t.txt
load tf.txt;
tf = tf(1,1);

load AX.txt
load BX.txt
load CX.txt
load DX.txt

load AY.txt
load BY.txt
load CY.txt
load DY.txt

phy = reshape(phy,length(phy),1);
x = reshape(x,length(x)./NCNC,NCNC);
y = reshape(y,length(y)./NCNC,NCNC);
theta = reshape(t,length(t)./NCNC,NCNC);

beisu = 4;
number_of_frame = round(tf*24 / beisu);

phy = smoother(phy,number_of_frame,tf);
x = smoother(x,number_of_frame,tf);
y = smoother(y,number_of_frame,tf);
theta = smoother(theta,number_of_frame,tf);

oot = AX; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,tf); AX = oot;
oot = BX; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,tf); BX = oot;
oot = CX; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,tf); CX = oot;
oot = DX; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,tf); DX = oot;

oot = AY; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,tf); AY = oot;
oot = BY; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,tf); BY = oot;
oot = CY; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,tf); CY = oot;
oot = DY; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,tf); DY = oot;

hold on
for ii = 1:NCNC
    xt = x(:,ii);
    yt = y(:,ii);
    %set(0,'DefaultLineLineWidth',2);
    %plot(xt,yt);
end
awu = length(xt);

vidObj = VideoWriter('Result');
vidObj.Quality = 100;
vidObj.FrameRate = 24;%round(15*(7*(awu)/(2*ymym + awu))/10)*(11./8.65)*(6./7.30870629);

open(vidObj);

for jjx = 1 : awu
    
    axis equal
    box on
    grid on
    axis([-20,20,-20,20]);
    
    set(gcf,'outerposition',get(0,'screensize'));
    
    for ii = 1:NCNC
        x00 = x(jjx,ii);
        y00 = y(jjx,ii);
        t00 = theta(jjx,ii);
        APP = [AX(jjx,ii), AY(jjx,ii)];
        CPP = [CX(jjx,ii), CY(jjx,ii)];
        BPP = [BX(jjx,ii), BY(jjx,ii)];
        DPP = [DX(jjx,ii), DY(jjx,ii)];
        
        PPX = [APP(1),BPP(1),CPP(1),DPP(1),APP(1);APP(2),BPP(2),CPP(2),DPP(2),APP(2)];
        
        plot(PPX(1,:),PPX(2,:),'Color',colorpool(ii,:));
    end
    ii = 1;
    xx = x(jjx,ii);
    yy = y(jjx,ii);
    
    aa = theta(jjx,1).*180./pi;
    AA = [xx - sind(aa)*car_b2 + car_l * cosd(aa), yy + cosd(aa)*car_b2 + car_l * sind(aa)];
    BB = [xx + sind(aa)*car_b2 + car_l * cosd(aa), yy - cosd(aa)*car_b2 + car_l * sind(aa)];
    CC = [xx + sind(aa)*car_b2, yy - cosd(aa)*car_b2];
    DD = [xx - sind(aa)*car_b2, yy + cosd(aa)*car_b2];
    
    xxw = CC(1);
    yyw = CC(2);
    aaw = aa;
    draw_RCT
    h5 = get(gca, 'children');
    xxw = DD(1);
    yyw = DD(2);
    aaw = aa;
    draw_RCT
    h6 = get(gca, 'children');
    
    phaai = phy(jjx,1).*180./pi;
    
    xxw = AA(1);
    yyw = AA(2);
    aaw = aa+phaai;
    draw_RCT
    h7 = get(gca, 'children');
    xxw = BB(1);
    yyw = BB(2);
    aaw = aa+phaai;
    draw_RCT
    h8 = get(gca, 'children');
    
    for ii = 1:NCNC
        xt = x(1:jjx,ii);
        yt = y(1:jjx,ii);
        set(0,'DefaultLineLineWidth',2);
        %plot(xt,yt,'Color',colorpool(ii,:));
        hold on
    end
    
    h1 = get(gca, 'children');
    set(0,'DefaultLineLineWidth',2);
    for ii = 1: (NCNC-1)
        plot([x(jjx,ii),x(jjx,ii+1)],[y(jjx,ii),y(jjx,ii+1)],'k')
    end
    h3 = get(gca, 'children');
    
    grid on
    box on
    
    load Current_vertex;
    for ii = 1 :(length(Current_vertex)./8)
        APP = [Current_vertex((ii-1)*8+1,4),Current_vertex((ii-1)*8+2,4)];
        CPP = [Current_vertex((ii-1)*8+5,4),Current_vertex((ii-1)*8+6,4)];
        BPP = [Current_vertex((ii-1)*8+3,4),Current_vertex((ii-1)*8+4,4)];
        DPP = [Current_vertex((ii-1)*8+7,4),Current_vertex((ii-1)*8+8,4)];
        PPX = [APP(1),BPP(1),CPP(1),DPP(1),APP(1);APP(2),BPP(2),CPP(2),DPP(2),APP(2)];
        fill(PPX(1,:),PPX(2,:),[0.5,0.5,0.5]);
    end
    
    load Terminal_config;
    xce = Terminal_config(1,2);
    yce = Terminal_config(2,2);
    
    APP = [xce - 8, yce + 1.3];
    CPP = [xce + 8, yce - 1.3];
    BPP = [xce + 8, yce + 1.3];
    DPP = [xce - 8, yce - 1.3];
    set(0,'DefaultLineLineWidth',1.5);
    PPX = [APP(1),BPP(1),CPP(1),DPP(1),APP(1);APP(2),BPP(2),CPP(2),DPP(2),APP(2)];
    plot(PPX(1,:),PPX(2,:),'k--');
    
    h99 = get(gca, 'children');
    
    M(jjx) = getframe;
    writeVideo(vidObj, M(jjx));
    delete(h3);
    delete(h99);
end

close(vidObj);