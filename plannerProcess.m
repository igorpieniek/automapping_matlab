function poses = plannerProcess(planner,start, target)
% Funkcja planuj¹ca trasê 

if ~isa(planner, 'plannerHybridAStar')
    error('planner musi byc obiektem plannerHybridAStar')
elseif ~(nargin == 3)
    error('Za ma³o argumentów wejsciowych!');
elseif ~(nargout==1)
    error('Za ma³o argumentów wyjœciowych!');
elseif ~(length(start(1,:))==3)
    error('punkt startowy musi byæ w formacie [x y k¹t]')
elseif ~(length(target(1,:))==3)
    error('punkt koncowy musi byæ w formacie [x y k¹t]')
end

route = plan(planner, start, target);
route = route.States;

% Get poses from the route.
rsConn = reedsSheppConnection('MinTurningRadius', planner.MinTurningRadius);
startPoses = route(1:end-1,:);
endPoses = route(2:end,:);

rsPathSegs = connect(rsConn, startPoses, endPoses);
poses = [];
for i = 1:numel(rsPathSegs)
    lengths = 0:0.1:rsPathSegs{i}.Length;
    [pose, ~] = interpolate(rsPathSegs{i}, lengths);
    poses = [poses; pose];
end