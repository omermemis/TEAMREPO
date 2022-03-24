load('saved/data/data.mat')
load('saved/data/sys.mat')
compare(data,sys);
f = gcf;
grid on
title('Model Identification')
exportgraphics(f,'saved/asset/modelIdentification.png','Resolution',300)