% all statistics can be found under assets/saved/forComparison

compTime.CMPC.mean = [13 23 33 54 82 117 163 227 293 391];
compTime.CMPC.min = [10 18 24 36 54 69 105 138 194 247];
compTime.CMPC.max = [18 53 72 127 139 253 266 443 452 718];

compTime.DMPC.mean = [12 25 37 48 68 80 95 107 120 140];
compTime.DMPC.min = [10 20 30 41 54 67 78 94 109 117];
compTime.DMPC.max = [22 48 55 66 104 101 122 129 158 177];

Ts = 200; % samlpe time [ms]

FontSize = 28;
MarkerSize = 15;
LineWidth = 2.0;
figure('Name','camparison','position',[0 0 2000 1000])

subplot(1,2,1)
hold on
grid on
plot(compTime.CMPC.mean,'Color',[0.4660 0.6740 0.1880],'Marker','*','MarkerSize',MarkerSize,'LineStyle','-','LineWidth',LineWidth)
plot(compTime.CMPC.max,'Color',[0.4660 0.6740 0.1880],'Marker','s','MarkerSize',MarkerSize,'LineStyle','--','LineWidth',LineWidth)
plot(compTime.DMPC.mean,'Color',[0 0.4470 0.7410],'Marker','*','MarkerSize',MarkerSize,'LineStyle','-','LineWidth',LineWidth)
plot(compTime.DMPC.max,'Color',[0 0.4470 0.7410],'Marker','s','MarkerSize',MarkerSize,'LineStyle','--','LineWidth',LineWidth)
plot(Ts*ones(1,length(compTime.CMPC.mean)),'r','LineWidth',LineWidth)

title('Comparison of computational times')
xlabel('number of vehicles','FontSize',FontSize)
ylabel('computation time [ms]','FontSize',FontSize)
xlim([1 10])
legend({'centralized MPC (mean)','centralized MPC (max)','distributed MPC (mean)','distributed MPC (max)','step size of the high-level controller'},'FontSize',FontSize,'location','northwest')
set(gca,'FontSize',FontSize)


objectiveVal.CMPC.v = [19.5 43.9 78.2 124.2 185.9 255.2 342.3 611.7 1174.2 2295.8];
objectiveVal.CMPC.d = [0.0 2.2 10.7 26.1 44.2 56.2 62.9 85.0 164.3 725.7];
objectiveVal.CMPC.total = objectiveVal.CMPC.v + objectiveVal.CMPC.d;

objectiveVal.DMPC.v = [19.5 42.6 84.2 159.1 255.2 347.4 429.2 588.3 798.9 885.8];
objectiveVal.DMPC.d = [0.0 2.3 13.2 38.7 67.7 87.6 98.7 130.6 178.3 189.5];
objectiveVal.DMPC.total = objectiveVal.DMPC.v + objectiveVal.DMPC.d;
subplot(1,2,2)
hold on
grid on
plot(objectiveVal.CMPC.total,'Color',[0.4660 0.6740 0.1880],'Marker','*','MarkerSize',MarkerSize,'LineStyle','-','LineWidth',LineWidth)
plot(objectiveVal.DMPC.total,'Color',[0 0.4470 0.7410],'Marker','*','MarkerSize',MarkerSize,'LineStyle','-','LineWidth',LineWidth)

title('Comparison of objective values')
xlabel('number of vehicles','FontSize',FontSize)
ylabel('objective value','FontSize',FontSize)
xlim([1 10])
legend({'centralized MPC','distributed MPC'},'FontSize',FontSize,'location','northwest')
set(gca,'FontSize',FontSize)
% save as png
saveas(gcf,'camparison.png');
