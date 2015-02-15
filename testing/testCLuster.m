load('../cluster.mat');
eva = evalclusters(meas,'kmeans','CalinskiHarabasz','KList',[1:10])
