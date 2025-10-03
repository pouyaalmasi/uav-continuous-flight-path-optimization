clc;
clear;
close all;
tic
%% Global Variable
global Big_Value H V GSD DW DH generated_nodes points

% Big value
Big_Value = 1e12;

% parameters of camera
focal_length = 4.3;  %mm
S_w = 6.2;  %mm
imW = 4056;  %pixels
imH = 3040; %pixels

% flight parameters
H = 10;  %m
V = 5; %m/s


% GSD
GSD = (S_w * H * 100)/(focal_length * imW);
DW = GSD * imW / 100; %m
DH = GSD * imH / 100; %m


%% Meshing
width = 20;
height = 20;

pgon = polyshape([0 0 width height],[height 0 0 width]);
tr = triangulation(pgon);
model = createpde;
tnodes = tr.Points';
telements = tr.ConnectivityList';

geometry_results = geometryFromMesh(model,tnodes,telements);
mesh_result = generateMesh(model);
generated_nodes = mesh_result.Nodes';

%% meshing results

%  pdegplot(model)
%  figure
%  pdemesh(model)
%  figure
%  pdemesh(model)

%% Problem Definition

CostFunction = @(x) flight_cost(x);        % Cost Function

nVar = 9;                      % Number of Decision Variables
interval_points = 40;
number_points = interval_points * interval_points;

X = linspace(0,width,interval_points);
Y = linspace(0,height,interval_points);
index_points = 1;
points = zeros(number_points,2);
for i = 1:interval_points
    for j = 1:interval_points
        points(index_points,:) = [X(i),Y(j)];
        index_points = index_points + 1;
    end
end
%% if condition
%to check the points (if they are out of the area)
        

VarSize = [1 nVar];                         % Size of Decision Variables Matrix

VarMin = 1;                     % Lower Bound of Variables
VarMax = number_points;         % Upper Bound of Variables

%% PSO Parameters

MaxIt = 200;      % Maximum Number of Iterations

nPop = 50000;        % Population Size (Swarm Size)

% % Constriction Coefficients
phi1 = 2.05;
phi2 = 2.05;
phi = phi1+phi2;
chi = 2/(phi-2+sqrt(phi^2-4*phi));
w = chi;          % Inertia Weight
wdamp = 1;    % Inertia Weight Damping Ratio
c1 = chi*phi1;    % Personal Learning Coefficient
c2 = chi*phi2;    % Global Learning Coefficient

% Velocity Limits
VelMax = 0.1*(VarMax-VarMin);
VelMin = -VelMax;

%% Initialization

empty_particle.Position = [];
empty_particle.Cost = [];
empty_particle.Velocity = [];
empty_particle.Best.Position = [];
empty_particle.Best.Cost = [];

particle = repmat(empty_particle, nPop, 1);

GlobalBest.Cost = inf;

for i = 1:nPop
    
    % Initialize Position
    particle(i).Position = floor(unifrnd(VarMin, VarMax, VarSize));
        
    % Initialize Velocity
    particle(i).Velocity = zeros(VarSize);
    
    % Evaluation
    [particle(i).Cost,particle(i).Position,temp] = CostFunction(particle(i).Position);
    
    % Update Personal Best
    particle(i).Best.Position = particle(i).Position;
    particle(i).Best.Cost = particle(i).Cost;
    
    % Update Global Best
    if particle(i).Best.Cost<GlobalBest.Cost
        
        GlobalBest = particle(i).Best;
        
    end
    
end

BestCost = zeros(MaxIt, 1);

%% PSO Main Loop

for it = 1:MaxIt
    
    for i = 1:nPop
        
        % Update Velocity
        particle(i).Velocity = w*particle(i).Velocity ...
            +c1*rand(VarSize).*(particle(i).Best.Position-particle(i).Position) ...
            +c2*rand(VarSize).*(GlobalBest.Position-particle(i).Position);
        
        % Apply Velocity Limits
        particle(i).Velocity = max(particle(i).Velocity, VelMin);
        particle(i).Velocity = min(particle(i).Velocity, VelMax);
        
        % Update Position
        particle(i).Position = floor(particle(i).Position + particle(i).Velocity);
        
        % Velocity Mirror Effect
        IsOutside = (particle(i).Position<VarMin | particle(i).Position>VarMax);
        particle(i).Velocity(IsOutside) = -particle(i).Velocity(IsOutside);
        
        % Apply Position Limits
        particle(i).Position = max(particle(i).Position, VarMin);
        particle(i).Position = min(particle(i).Position, VarMax);
        
        % Evaluation
        [particle(i).Cost,particle(i).Position,temp] = CostFunction(particle(i).Position);
        
        % Update Personal Best
        if particle(i).Cost<particle(i).Best.Cost
            
            particle(i).Best.Position = particle(i).Position;
            particle(i).Best.Cost = particle(i).Cost;
            
            % Update Global Best
            if particle(i).Best.Cost<GlobalBest.Cost
                
                GlobalBest = particle(i).Best;
                
                
            end
            
        end
        
    end
    
    BestCost(it) = GlobalBest.Cost;
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);    
    [temp_cost,temp_position,temp] = CostFunction(GlobalBest.Position);
    disp(temp)
    disp(temp_cost)
    w = w*wdamp;

    
end
%% Coordinates
p = zeros(nVar, 2);
for i = 1:nVar
    p(i,:) = points(GlobalBest.Position(i),:);
end
plot(p(:,1),p(:,2),'ko-')
axis([0,width,0,height])
disp (p)
BestSol = GlobalBest;

%% Results

figure;
plot(BestCost, 'LineWidth', 2);
semilogy(BestCost, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Best Cost');
axis([1,200,10,100])
grid on;
toc