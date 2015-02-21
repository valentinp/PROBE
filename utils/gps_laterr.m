pts_gps = trans_rtk;
pts_odo = trans_cam;

npts_odo = size(pts_odo,2);
npts_gps = size(pts_gps,2);

dists = zeros(npts_odo,npts_gps-1);

for i = 1:npts_odo
    for j = 1:npts_gps-1
        fprintf('i = %d, j = %d\n',i,j);
        dists(i,j) = perpDistToLine(pts_gps(:,j),pts_gps(:,j+1),pts_odo(:,i));
    end
end

[mindists,inds] = min(abs(dists),[],2);


gps_lat_err = zeros(1,npts_odo);

for k = 1:npts_odo
    gps_lat_err(k) = dists(k,inds(k));
end