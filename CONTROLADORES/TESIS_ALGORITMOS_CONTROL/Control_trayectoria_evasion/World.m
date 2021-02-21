function reduccion = World(lidarSub,pose)
    th = pose(3)-pi/2;
    scanMsg = receive(lidarSub);
    data=readCartesian(scanMsg)*[0 1; -1 0];
    world=data*[cos(th) sin(th);-sin(th) cos(th)] + repmat(pose(1:2),[numel(data(:,1)),1]);
    world_t=world';
    reduccion=[world_t(1,1:50),world_t(1,end-80,end);...
               world_t(2,1:50),world_t(2,end-80,end)];
end

