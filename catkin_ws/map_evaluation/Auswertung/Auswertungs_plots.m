clear;

[fpr,tpr,mr,algorithm,ground_truth] = importfile('rocs.csv',2,0);

coords = [fpr,tpr];
text = [algorithm,ground_truth];

hokuyo_1 = find(not(cellfun('isempty',strfind(algorithm,'hokuyo_1'))));
hokuyo_2 = find(not(cellfun('isempty',strfind(algorithm,'hokuyo_2'))));
sick_1 = find(not(cellfun('isempty',strfind(algorithm,'sick_1'))));
sick_2 = find(not(cellfun('isempty',strfind(algorithm,'sick_2'))));

hokuyo_1_gmapping = find(not(cellfun('isempty',strfind(algorithm(hokuyo_1),'hector'))));
hokuyo_1_fpr = fpr(hokuyo_1);
hokuyo_1_tpr = tpr(hokuyo_1);

figure(5)
plot(hokuyo_1_fpr(hokuyo_1_gmapping),hokuyo_1_tpr(hokuyo_1_gmapping),'x');
ylim([0,1]);
xlim([0,0.2]);
title('hokuyo 1 hector');

figure(1)
plot(fpr(hokuyo_1),tpr(hokuyo_1),'bx');
ylim([0,1]);
xlim([0,0.2]);
title('hokuyo 1');
% figure(2)
% plot(fpr(hokuyo_2),tpr(hokuyo_2),'rx');
% ylim([0,1]);
% xlim([0,0.2]);
% title('hokuyo 2');
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
