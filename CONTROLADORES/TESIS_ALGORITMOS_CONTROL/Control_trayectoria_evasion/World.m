function [world,scanMsg] = World(lidarSub,pose)
    th = pose(3)-pi/2;
    scanMsg = receive(lidarSub);
    data=readCartesian(scanMsg)*[0 1; -1 0];
%     data_aux=readCartesian(scanMsg)*[0 1; -1 0];
%     [i,j]=find(data_aux(:,2)>0);
%     data=data_aux(i,:);
    world=data*[cos(th) sin(th);-sin(th) cos(th)] + repmat(pose(1:2),[numel(data(:,1)),1]);  
end

