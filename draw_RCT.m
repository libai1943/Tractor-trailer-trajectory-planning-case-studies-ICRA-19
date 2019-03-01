lll = 0.3;
stda = 10;
AAw = [xxw + cosd(aaw + stda)*lll, yyw + sind(aaw + stda)*lll];
BBw = [xxw + cosd(-aaw + stda)*lll, yyw - sind(-aaw + stda)*lll];
CCw = [xxw - cosd(-aaw + stda)*lll, yyw + sind(-aaw + stda)*lll];
DDw = [xxw - cosd(aaw + stda)*lll, yyw - sind(aaw + stda)*lll];

PCDP = [AAw(1),BBw(1),DDw(1),CCw(1),AAw(1);AAw(2),BBw(2),DDw(2),CCw(2),AAw(2)];
plot(PCDP(1,:),PCDP(2,:),'k');