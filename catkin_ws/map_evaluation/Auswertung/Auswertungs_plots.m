clear;
cOrange = [245,159,0]./255;
cRed = [121,16,17]./255;
cBlue = [0,154,198]./255;
uLGreen = [182,200,40]./255;
cGrey = [191,191,191]./255;
uDGreen = [0,117,86]./255;
cBGreen = [204,227,221]./255;
cPurple = [127,13,93]./255;


[fpr,tpr,mr,algorithm,ground_truth] = importfile('rocs.csv',2,0);

[algorithm,I] = sort(algorithm);
fpr = fpr(I);
tpr = tpr(I);
mr = mr(I);
ground_truth = ground_truth(I);

coords = [fpr,tpr];
text = [algorithm,ground_truth];

hokuyo_1 = find(not(cellfun('isempty',strfind(algorithm,'hokuyo_1'))));
hokuyo_2 = find(not(cellfun('isempty',strfind(algorithm,'hokuyo_2'))));
sick_1 = find(not(cellfun('isempty',strfind(algorithm,'sick_1'))));
sick_2 = find(not(cellfun('isempty',strfind(algorithm,'sick_2'))));

gmapping = find(not(cellfun('isempty',strfind(algorithm,'gmapping'))));
hector_mapping = find(not(cellfun('isempty',strfind(algorithm,'hector_mapping'))));
octomapping = find(not(cellfun('isempty',strfind(algorithm,'octopmapping'))));

res_0025 = [find(not(cellfun('isempty',strfind(algorithm,'resolution_0.025')))); ...
    find(not(cellfun('isempty',strfind(algorithm,'delta_0.025'))))];
res_005 = [find(not(cellfun('isempty',strfind(algorithm,'resolution_0.05')))); ...
    find(not(cellfun('isempty',strfind(algorithm,'delta_0.05'))))];
res_01 = [find(not(cellfun('isempty',strfind(algorithm,'resolution_0.1')))); ...
    find(not(cellfun('isempty',strfind(algorithm,'delta_0.1'))))];

% gmapping
particles_20 = find(not(cellfun('isempty',strfind(algorithm,'particles_20')))); % default 30
particles_50 = find(not(cellfun('isempty',strfind(algorithm,'particles_50'))));
particles_75 = find(not(cellfun('isempty',strfind(algorithm,'particles_75'))));

minimumScore_00 = find(not(cellfun('isempty',strfind(algorithm,'minimumScore_0.0_')))); % default
minimumScore_001 = find(not(cellfun('isempty',strfind(algorithm,'minimumScore_0.01'))));
minimumScore_03 = find(not(cellfun('isempty',strfind(algorithm,'minimumScore_0.3'))));

update_interval_8 = find(not(cellfun('isempty',strfind(algorithm,'update_interval_8'))));
update_interval_5 = find(not(cellfun('isempty',strfind(algorithm,'update_interval_5.0')))); % default
update_interval_2 = find(not(cellfun('isempty',strfind(algorithm,'update_interval_2.0'))));

% let's try pca
len = size(fpr,1);
data = [tpr,fpr,zeros(len,1)];
index=3;

data(res_0025,index) = ones(size(res_0025,1),1).*0.025;
data(res_005,index) = ones(size(res_005,1),1).*0.05;
data(res_01,index) = ones(size(res_01,1),1).*0.1;
% index = index+1;
[coeff,score,latend]=pca(data(gmapping,:));
vrange = [0.025,0.1];
xrange = [0.0,0.12];

fig_num = 1;
figure(fig_num)
scatter3(data(gmapping,1),data(gmapping,2),data(gmapping,3),...
    'MarkerFaceColor',cBlue,...
    'MarkerEdgeColor','k');
xlabel('TPR');
ylabel('FPR');
zlabel('Resolution');
m1=coeff(1,1)/coeff(3,1);
m2=coeff(2,1)/coeff(3,1);
f1 = fittype(strcat(num2str(m1),'*x+a'));
f2 = fittype(strcat(num2str(m2),'*x+a'));
fitobj1 = fit(data(gmapping,3),data(gmapping,1),f1);
fitobj2 = fit(data(gmapping,3),data(gmapping,2),f2);
hold on;
plot3(vrange.*m1+fitobj1.a,vrange.*m2+fitobj2.a,vrange,'-','Color',cOrange,'LineWidth',3);
hold off;
view(-45,50)
set(gca,'xcolor',uDGreen);
set(gca,'ycolor',cRed);
print('Resolution_Cloud','-dsvg');
fig_num = fig_num+1;

figure(fig_num)
[hAx,hLine1,hLine2] = plotyy(data(gmapping,3),data(gmapping,1),data(gmapping,3),data(gmapping,2));
hLine1.LineStyle = 'none';
hLine1.Marker = '.';
hLine1.MarkerFaceColor = uDGreen;
hLine1.MarkerEdgeColor = uDGreen;
hLine2.LineStyle = 'none';
hLine2.Marker = 'x';
hLine2.MarkerFaceColor = cRed;
hLine2.MarkerEdgeColor = cRed;
hAx(1).YColor = uDGreen;
hAx(2).YColor = cRed;

ylim1 = [0,1];
% ylim2 = ylim(hAx(2));
ylim2 = [0,0.16];
ylim(hAx(1),ylim1);
ylim(hAx(2),ylim2);
xlim(hAx(1),xrange);
xlim(hAx(2),xrange);
hAx(2).YTick = linspace(ylim2(1),ylim2(2),10);
hAx(1).YTickMode = 'auto';
hAx(2).YTickMode = 'auto';
hAx(1).YTickLabelMode = 'auto';
hAx(2).YTickLabelMode = 'auto';
hAx(1).Box = 'off';
hold on;
[hAx,hLine1,hLine2] = plotyy(xrange,xrange.*m1+fitobj1.a,xrange,xrange.*m2+fitobj2.a);

ylim(hAx(1),ylim1);
ylim(hAx(2),ylim2);
xlim(hAx(1),xrange);
xlim(hAx(2),xrange);
hLine1.LineStyle = '-';
hLine1.Color = uDGreen;
hLine2.LineStyle = '-';
hLine2.Color = cRed;
hAx(1).YColor = uDGreen;
hAx(2).YColor = cRed;
ylabel(hAx(1),'TPR');
ylabel(hAx(2),'FPR');
xlabel('Resolution');
hAx(1).YTickMode = 'auto';
hAx(2).YTickMode = 'auto';
hAx(1).YTickLabelMode = 'auto';
hAx(2).YTickLabelMode = 'auto';
hAx(1).Box = 'off';
hold off;
print('Resolution_2D','-dsvg');
fig_num = fig_num+1;


data(particles_20,index) = ones(size(particles_20,1),1).*20;
data(particles_50,index) = ones(size(particles_50,1),1).*50;
data(particles_75,index) = ones(size(particles_75,1),1).*75;
% index = index+1;
[coeff,score,latend]=pca(data(gmapping,:));
vrange = [20,75];
xrange = [15,80];

figure(fig_num)
scatter3(data(gmapping,1),data(gmapping,2),data(gmapping,3),...
    'MarkerFaceColor',cBlue,...
    'MarkerEdgeColor','k');
xlabel('TPR');
ylabel('FPR');
zlabel('Particles');
m1=coeff(1,1)/coeff(3,1);
m2=coeff(2,1)/coeff(3,1);
f1 = fittype(strcat(num2str(m1),'*x+a'));
f2 = fittype(strcat(num2str(m2),'*x+a'));
fitobj1 = fit(data(gmapping,3),data(gmapping,1),f1);
fitobj2 = fit(data(gmapping,3),data(gmapping,2),f2);
hold on;
plot3(vrange.*m1+fitobj1.a,vrange.*m2+fitobj2.a,vrange,'-','Color',cOrange,'LineWidth',3);
hold off;
view(-45,50)
set(gca,'xcolor',uDGreen);
set(gca,'ycolor',cRed);
print('Particles_Cloud','-dsvg');
fig_num = fig_num+1;

figure(fig_num)
[hAx,hLine1,hLine2] = plotyy(data(gmapping,3),data(gmapping,1),data(gmapping,3),data(gmapping,2));
hLine1.LineStyle = 'none';
hLine1.Marker = '.';
hLine1.MarkerFaceColor = uDGreen;
hLine1.MarkerEdgeColor = uDGreen;
hLine2.LineStyle = 'none';
hLine2.Marker = 'x';
hLine2.MarkerFaceColor = cRed;
hLine2.MarkerEdgeColor = cRed;
hAx(1).YColor = uDGreen;
hAx(2).YColor = cRed;

ylim1 = [0,1];
% ylim2 = ylim(hAx(2));
ylim2 = [0,0.16];
ylim(hAx(1),ylim1);
ylim(hAx(2),ylim2);
xlim(hAx(1),xrange);
xlim(hAx(2),xrange);
hAx(2).YTick = linspace(ylim2(1),ylim2(2),10);
hAx(1).YTickMode = 'auto';
hAx(2).YTickMode = 'auto';
hAx(1).YTickLabelMode = 'auto';
hAx(2).YTickLabelMode = 'auto';
hAx(1).Box = 'off';
hold on;
[hAx,hLine1,hLine2] = plotyy(xrange,xrange.*m1+fitobj1.a,xrange,xrange.*m2+fitobj2.a);

ylim(hAx(1),ylim1);
ylim(hAx(2),ylim2);
xlim(hAx(1),xrange);
xlim(hAx(2),xrange);
hLine1.LineStyle = '-';
hLine1.Color = uDGreen;
hLine2.LineStyle = '-';
hLine2.Color = cRed;
hAx(1).YColor = uDGreen;
hAx(2).YColor = cRed;
ylabel(hAx(1),'TPR');
ylabel(hAx(2),'FPR');
xlabel('Particles');
hAx(1).YTickMode = 'auto';
hAx(2).YTickMode = 'auto';
hAx(1).YTickLabelMode = 'auto';
hAx(2).YTickLabelMode = 'auto';
hAx(1).Box = 'off';
hold off;
print('Particles_2D','-dsvg');
fig_num = fig_num+1;




data(minimumScore_00,index) = ones(size(minimumScore_00,1),1).*0;
data(minimumScore_001,index) = ones(size(minimumScore_001,1),1).*0.01;
data(minimumScore_03,index) = ones(size(minimumScore_03,1),1).*0.3;
% % index = index+1;
[coeff,score,latend]=pca(data(gmapping,:));
vrange = [0,0.3];
xrange = [-0.05,0.35];

figure(fig_num)
scatter3(data(gmapping,1),data(gmapping,2),data(gmapping,3),...
    'MarkerFaceColor',cBlue,...
    'MarkerEdgeColor','k');
xlabel('TPR');
ylabel('FPR');
zlabel('Minimum Score');
m1=coeff(1,1)/coeff(3,1);
m2=coeff(2,1)/coeff(3,1);
f1 = fittype(strcat(num2str(m1),'*x+a'));
f2 = fittype(strcat(num2str(m2),'*x+a'));
fitobj1 = fit(data(gmapping,3),data(gmapping,1),f1);
fitobj2 = fit(data(gmapping,3),data(gmapping,2),f2);
hold on;
plot3(vrange.*m1+fitobj1.a,vrange.*m2+fitobj2.a,vrange,'-','Color',cOrange,'LineWidth',3);
hold off;
view(-45,50)
set(gca,'xcolor',uDGreen);
set(gca,'ycolor',cRed);
print('MinimumScore_Cloud','-dsvg');
fig_num = fig_num+1;

figure(fig_num)
[hAx,hLine1,hLine2] = plotyy(data(gmapping,3),data(gmapping,1),data(gmapping,3),data(gmapping,2));
hLine1.LineStyle = 'none';
hLine1.Marker = '.';
hLine1.MarkerFaceColor = uDGreen;
hLine1.MarkerEdgeColor = uDGreen;
hLine2.LineStyle = 'none';
hLine2.Marker = 'x';
hLine2.MarkerFaceColor = cRed;
hLine2.MarkerEdgeColor = cRed;
hAx(1).YColor = uDGreen;
hAx(2).YColor = cRed;

ylim1 = [0,1];
% ylim2 = ylim(hAx(2));
ylim2 = [0,0.16];
ylim(hAx(1),ylim1);
ylim(hAx(2),ylim2);
xlim(hAx(1),xrange);
xlim(hAx(2),xrange);
hAx(2).YTick = linspace(ylim2(1),ylim2(2),10);
hAx(1).YTickMode = 'auto';
hAx(2).YTickMode = 'auto';
hAx(1).YTickLabelMode = 'auto';
hAx(2).YTickLabelMode = 'auto';
hAx(1).Box = 'off';
hold on;
[hAx,hLine1,hLine2] = plotyy(xrange,xrange.*m1+fitobj1.a,xrange,xrange.*m2+fitobj2.a);

ylim(hAx(1),ylim1);
ylim(hAx(2),ylim2);
xlim(hAx(1),xrange);
xlim(hAx(2),xrange);
hLine1.LineStyle = '-';
hLine1.Color = uDGreen;
hLine2.LineStyle = '-';
hLine2.Color = cRed;
hAx(1).YColor = uDGreen;
hAx(2).YColor = cRed;
ylabel(hAx(1),'TPR');
ylabel(hAx(2),'FPR');
xlabel('Minimum Score');
hAx(1).YTickMode = 'auto';
hAx(2).YTickMode = 'auto';
hAx(1).YTickLabelMode = 'auto';
hAx(2).YTickLabelMode = 'auto';
hAx(1).Box = 'off';
hold off;
print('MinimumScore_2D','-dsvg');
fig_num = fig_num+1;


data(update_interval_8,index) = ones(size(update_interval_8,1),1).*8;
data(update_interval_5,index) = ones(size(update_interval_5,1),1).*5;
data(update_interval_2,index) = ones(size(update_interval_2,1),1).*2;
[coeff,score,latend]=pca(data(gmapping,:));
vrange = [2,8];
xrange = [1,9];

figure(fig_num)
scatter3(data(gmapping,1),data(gmapping,2),data(gmapping,3),...
    'MarkerFaceColor',cBlue,...
    'MarkerEdgeColor','k');
xlabel('TPR');
ylabel('FPR');
zlabel('Map Update Interval');
m1=coeff(1,1)/coeff(3,1);
m2=coeff(2,1)/coeff(3,1);
f1 = fittype(strcat(num2str(m1),'*x+a'));
f2 = fittype(strcat(num2str(m2),'*x+a'));
fitobj1 = fit(data(gmapping,3),data(gmapping,1),f1);
fitobj2 = fit(data(gmapping,3),data(gmapping,2),f2);
hold on;
plot3(vrange.*m1+fitobj1.a,vrange.*m2+fitobj2.a,vrange,'-','Color',cOrange,'LineWidth',3);
hold off;
view(-45,50)
set(gca,'xcolor',uDGreen);
set(gca,'ycolor',cRed);
print('UpdateInterval_Cloud','-dsvg');
fig_num = fig_num+1;

figure(fig_num)
[hAx,hLine1,hLine2] = plotyy(data(gmapping,3),data(gmapping,1),data(gmapping,3),data(gmapping,2));
hLine1.LineStyle = 'none';
hLine1.Marker = '.';
hLine1.MarkerFaceColor = uDGreen;
hLine1.MarkerEdgeColor = uDGreen;
hLine2.LineStyle = 'none';
hLine2.Marker = 'x';
hLine2.MarkerFaceColor = cRed;
hLine2.MarkerEdgeColor = cRed;
hAx(1).YColor = uDGreen;
hAx(2).YColor = cRed;

ylim1 = [0,1];
% ylim2 = ylim(hAx(2));
ylim2 = [0,0.16];
ylim(hAx(1),ylim1);
ylim(hAx(2),ylim2);
xlim(hAx(1),xrange);
xlim(hAx(2),xrange);
hAx(2).YTick = linspace(ylim2(1),ylim2(2),10);
hAx(1).YTickMode = 'auto';
hAx(2).YTickMode = 'auto';
hAx(1).YTickLabelMode = 'auto';
hAx(2).YTickLabelMode = 'auto';
hAx(1).Box = 'off';
hold on;
[hAx,hLine1,hLine2] = plotyy(xrange,xrange.*m1+fitobj1.a,xrange,xrange.*m2+fitobj2.a);

ylim(hAx(1),ylim1);
ylim(hAx(2),ylim2);
xlim(hAx(1),xrange);
xlim(hAx(2),xrange);
hLine1.LineStyle = '-';
hLine1.Color = uDGreen;
hLine2.LineStyle = '-';
hLine2.Color = cRed;
hAx(1).YColor = uDGreen;
hAx(2).YColor = cRed;
ylabel(hAx(1),'TPR');
ylabel(hAx(2),'FPR');
xlabel('Map Update Interval');
hAx(1).YTickMode = 'auto';
hAx(2).YTickMode = 'auto';
hAx(1).YTickLabelMode = 'auto';
hAx(2).YTickLabelMode = 'auto';
hAx(1).Box = 'off';
hold off;
print('UpdateInterval_2D','-dsvg');
fig_num = fig_num+1;




% Mean doesn't help
% particles_20_fpr_m = sum(fpr(particles_20))/size((particles_20),1);
% particles_20_fpr_v = sum((fpr(particles_20)-particles_20_fpr_m).^2)/size((particles_20),1);
% particles_50_fpr_m = sum(fpr(particles_50))/size((particles_50),1);
% particles_50_fpr_v = sum((fpr(particles_50)-particles_50_fpr_m).^2)/size((particles_50),1);
% particles_75_fpr_m = sum(fpr(particles_75))/size((particles_75),1);
% particles_75_fpr_v = sum((fpr(particles_75)-particles_75_fpr_m).^2)/size((particles_75),1);
% 
% particles_20_tpr_m = sum(tpr(particles_20))/size((particles_20),1);
% particles_20_tpr_v = sum((tpr(particles_20)-particles_20_tpr_m).^2)/size((particles_20),1);
% particles_50_tpr_m = sum(tpr(particles_50))/size((particles_50),1);
% particles_50_tpr_v = sum((tpr(particles_50)-particles_50_tpr_m).^2)/size((particles_50),1);
% particles_75_tpr_m = sum(tpr(particles_75))/size((particles_75),1);
% particles_75_tpr_v = sum((tpr(particles_75)-particles_75_tpr_m).^2)/size(tpr(particles_75),1);
% 
% particles_fpr_m = [particles_20_fpr_m,particles_50_fpr_m,particles_75_fpr_m];
% particles_tpr_m = [particles_20_tpr_m,particles_50_tpr_m,particles_75_tpr_m];
% plot(particles_fpr_m,particles_tpr_m,'x-');
% % hold on;
% % plot(fpr(update_interval_8),tpr(update_interval_8),'ro');
% % plot(fpr(update_interval_5),tpr(update_interval_5),'go');
% % plot(fpr(update_interval_2),tpr(update_interval_2),'bo');
% % hold off;
% ylim([0,1]);
% xlim([0,0.16]);


% abstract Art xD
% figure(1)
% plot(fpr(gmapping),tpr(gmapping),'rx');
% hold on;
% particles_fpr = [fpr(particles_20),fpr(particles_50),fpr(particles_75)]';
% particles_tpr = [tpr(particles_20),tpr(particles_50),tpr(particles_75)]';
% plot(particles_fpr,particles_tpr,'-');
% hold off;
% ylim([0,1]);
% xlim([0,0.16]);
% 
% figure(2)
% plot(fpr(gmapping),tpr(gmapping),'rx');
% hold on;
% score_fpr = [fpr(minimumScore_00),fpr(minimumScore_001),fpr(minimumScore_03)]';
% score_tpr = [tpr(minimumScore_00),tpr(minimumScore_001),tpr(minimumScore_03)]';
% plot(score_fpr,score_tpr,'-');
% hold off;
% ylim([0,1]);
% xlim([0,0.16]);
% 
% figure(3)
% plot(fpr(gmapping),tpr(gmapping),'rx');
% hold on;
% update_fpr = [fpr(update_interval_8),fpr(update_interval_5),fpr(update_interval_2)]';
% update_tpr = [tpr(update_interval_8),tpr(update_interval_5),tpr(update_interval_2)]';
% plot(update_fpr,update_tpr,'-');
% hold off;
% ylim([0,1]);
% xlim([0,0.16]);
% 
% figure(4)
% plot(fpr(gmapping),tpr(gmapping),'rx');
% hold on;
% res_fpr = [fpr(intersect(res_0025,gmapping)),fpr(intersect(res_005,gmapping)),fpr(intersect(res_01,gmapping))]';
% res_tpr = [tpr(intersect(res_0025,gmapping)),tpr(intersect(res_005,gmapping)),tpr(intersect(res_01,gmapping))]';
% plot(res_fpr,res_tpr);
% hold off;
% ylim([0,1]);
% xlim([0,0.16]);





% just random stuff following

% hector mapping
% update_angle_05
% update_angle_09
% update_angle_13
% 
% update_distance_01
% update_distance_04
% update_distance_07


% mapping = gmapping;
% tide_file = sick_2;
% resolution = res_01;
% 
% figure(1)
% plot(fpr(tide_file),tpr(tide_file),'bx');
% hold on;
% plot(fpr(intersect(tide_file,mapping)),tpr(intersect(tide_file,mapping)),'ro','Markersize',8);
% % plot(fpr(intersect(tide_file,resolution)),tpr(intersect(tide_file,resolution)),'ro','Markersize',8);
% 
% hold off;
% ylim([0,1]);
% xlim([0,0.16]);
% title('Intersect');


% figure(2)
% % plot(fpr(res_0025),tpr(res_0025),'rx');
% plot(fpr(intersect(tide_file,mapping)),tpr(intersect(tide_file,mapping)),'bx');
% hold on;
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_20),res_0025)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_20),res_0025)),'-');
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_20),res_005)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_20),res_005)),'-');
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_20),res_01)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_20),res_01)),'-');
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_50),res_0025)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_50),res_0025)),'-'); 
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_50),res_005)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_50),res_005)),'-');
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_50),res_01)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_50),res_01)),'-');
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_75),res_0025)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_75),res_0025)),'-');
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_75),res_005)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_75),res_005)),'-');
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_75),res_01)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_75),res_01)),'-');
% plot(fpr(intersect(intersect(tide_file,mapping),minimumScore_00)),tpr(intersect(intersect(tide_file,mapping),minimumScore_00)),'ro','Markersize',8);
% plot(fpr(intersect(intersect(tide_file,mapping),minimumScore_001)),tpr(intersect(intersect(tide_file,mapping),minimumScore_001)),'go','Markersize',8);
% plot(fpr(intersect(intersect(tide_file,mapping),minimumScore_03)),tpr(intersect(intersect(tide_file,mapping),minimumScore_03)),'bo','Markersize',8);
% hold off;
% ylim([0,1]);
% xlim([0,0.16]);
% title('gmapping');
% % set(gcf,'Position',[0,0,1000,800]);
% print('test','-dsvg');

% figure(2)
% % plot(fpr(res_0025),tpr(res_0025),'rx');
% plot(fpr(intersect(tide_file,mapping)),tpr(intersect(tide_file,mapping)),'bx');
% hold on;
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_20),minimumScore_00)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_20),minimumScore_00)),'-','Color',[1,0.5,0]);
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_20),minimumScore_001)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_20),minimumScore_001)),'-');
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_20),minimumScore_03)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_20),minimumScore_03)),'-');
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_50),minimumScore_00)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_50),minimumScore_00)),'-','Color',[1,0,1]);
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_50),minimumScore_001)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_50),minimumScore_001)),'-');
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_50),minimumScore_03)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_50),minimumScore_03)),'-');
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_75),minimumScore_00)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_75),minimumScore_00)),'c-');
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_75),minimumScore_001)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_75),minimumScore_001)),'-');
% plot(fpr(intersect(intersect(intersect(tide_file,mapping),particles_75),minimumScore_03)),tpr(intersect(intersect(intersect(tide_file,mapping),particles_75),minimumScore_03)),'-');
% plot(fpr(intersect(intersect(tide_file,mapping),res_0025)),tpr(intersect(intersect(tide_file,mapping),res_0025)),'ro','Markersize',8);
% plot(fpr(intersect(intersect(tide_file,mapping),res_005)),tpr(intersect(intersect(tide_file,mapping),res_005)),'go','Markersize',8);
% plot(fpr(intersect(intersect(tide_file,mapping),res_01)),tpr(intersect(intersect(tide_file,mapping),res_01)),'bo','Markersize',8);
% hold off;
% ylim([0,1]);
% xlim([0,0.2]);
% title('gmapping');

% figure(3)
% plot(fpr(sick_1),tpr(sick_1),'rx');
% ylim([0,1]);
% xlim([0,0.2]);
% title('sick 1');
% figure(4)
% plot(fpr(sick_2),tpr(sick_2),'rx');
% ylim([0,1]);
% xlim([0,0.2]);
% title('sick 2');
