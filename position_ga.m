% MTSPO_GA Open Multiple Traveling Salesmen Problem (M-TSP) Genetic Algorithm (GA)
%   Finds a (near) optimal solution to a variation of the M-TSP by setting
%   up a GA to search for the shortest route (least distance needed for the
%   salesmen to travel to each city exactly once without returning to their
%   starting location)
%
% Summary:
%     1. Each salesman travels to a unique set of cities (although none of
%        them close their loops by returning to their starting points)
%     2. Each city is visited by exactly one salesman
%
% Input:
%     USERCONFIG (structure) with zero or more of the following fields:
%     - XY (float) is an Nx2 matrix of city locations, where N is the number of cities
%     - DMAT (float) is an NxN matrix of city-to-city distances or costs
%     - NSALESMEN (scalar integer) is the number of salesmen to visit the cities
%     - MINTOUR (scalar integer) is the minimum tour length for any of the salesmen
%     - POPSIZE (scalar integer) is the size of the population (should be divisible by 8)
%     - NUMITER (scalar integer) is the number of desired iterations for the algorithm to run
%     - SHOWPROG (scalar logical) shows the GA progress if true
%     - SHOWRESULT (scalar logical) shows the GA results if true
%     - SHOWWAITBAR (scalar logical) shows a waitbar if true
%
% Input Notes:
%     1. Rather than passing in a structure containing these fields, any/all of
%        these inputs can be passed in as parameter/value pairs in any order instead.
%     2. Field/parameter names are case insensitive but must match exactly otherwise.
%
% Output:
%     RESULTSTRUCT (structure) with the following fields:
%         (in addition to a record of the algorithm configuration)
%     - OPTROUTE (integer array) is the best route found by the algorithm
%     - OPTBREAK (integer array) is the list of route break points (these specify the indices
%         into the route used to obtain the individual salesman routes)
%     - MINDIST (scalar float) is the total distance traveled by the salesmen
%
% Route/Breakpoint Details:
%     If there are 10 cities and 3 salesmen, a possible route/break
%     combination might be: rte = [5 6 9 1 4 2 8 10 3 7], brks = [3 7]
%     Taken together, these represent the solution [5 6 9][1 4 2 8][10 3 7],
%     which designates the routes for the 3 salesmen as follows:
%         . Salesman 1 travels from city 5 to 6 to 9
%         . Salesman 2 travels from city 1 to 4 to 2 to 8
%         . Salesman 3 travels from city 10 to 3 to 7
%
% Usage:
%     mtspo_ga
%       -or-
%     mtspo_ga(userConfig)
%       -or-
%     resultStruct = mtspo_ga;
%       -or-
%     resultStruct = mtspo_ga(userConfig);
%       -or-
%     [...] = mtspo_ga('Param1',Value1,'Param2',Value2, ...);
%
% Example:
%     % Let the function create an example problem to solve
%     mtspo_ga;
%
% Example:
%     % Request the output structure from the solver
%     resultStruct = mtspo_ga;
%
% Example:
%     % Pass a random set of user-defined XY points to the solver
%     userConfig = struct('xy',10*rand(35,2));
%     resultStruct = mtspo_ga(userConfig);
%
% Example:
%     % Pass a more interesting set of XY points to the solver
%     n = 50;
%     phi = (sqrt(5)-1)/2;
%     theta = 2*pi*phi*(0:n-1);
%     rho = (1:n).^phi;
%     [x,y] = pol2cart(theta(:),rho(:));
%     xy = 10*([x y]-min([x;y]))/(max([x;y])-min([x;y]));
%     userConfig = struct('xy',xy);
%     resultStruct = mtspo_ga(userConfig);
%
% Example:
%     % Pass a random set of 3D (XYZ) points to the solver
%     xyz = 10*rand(35,3);
%     userConfig = struct('xy',xyz);
%     resultStruct = mtspo_ga(userConfig);
%
% Example:
%     % Change the defaults for GA population size and number of iterations
%     userConfig = struct('popSize',200,'numIter',1e4);
%     resultStruct = mtspo_ga(userConfig);
%
% Example:
%     % Turn off the plots but show a waitbar
%     userConfig = struct('showProg',false,'showResult',false,'showWaitbar',true);
%     resultStruct = mtspo_ga(userConfig);
%
% See also: mtsp_ga, mtspf_ga, mtspof_ga, mtspofs_ga, mtspv_ga, distmat
%
% Author: Joseph Kirk
% Email: jdkirk630@gmail.com
% Release: 2.0
% Release Date: 05/01/2014
function varargout = position_ga(varargin)
    
    % Initialize default configuration
    defaultConfig.xy          = 10*rand(40,2);
    defaultConfig.dmat        = [];
    defaultConfig.nSalesmen   = 5;
    defaultConfig.minTour     = 2;
    defaultConfig.popSize     = 80;
    defaultConfig.numIter     = 5e3;
    defaultConfig.showProg    = true;
    defaultConfig.showResult  = false;
    defaultConfig.showWaitbar = false;
    defaultConfig.batteryLife = 5;
    
    % Interpret user configuration inputs
    if ~nargin
        userConfig = struct();
    elseif isstruct(varargin{1})
        userConfig = varargin{1};
    else
        try
            userConfig = struct(varargin{:});
        catch
            error('Expected inputs are either a structure or parameter/value pairs');
        end
    end
    
    % Override default configuration with user inputs
    configStruct = get_config(defaultConfig,userConfig);
    
    % Extract configuration
    xy          = configStruct.xy;
    dmat        = configStruct.dmat;
    nSalesmen   = configStruct.nSalesmen;
    minTour     = configStruct.minTour;
    popSize     = configStruct.popSize;
    numIter     = configStruct.numIter;
    showProg    = configStruct.showProg;
    showResult  = configStruct.showResult;
    showWaitbar = configStruct.showWaitbar;
    batteryLife = configStruct.batteryLife;
    if isempty(dmat)
        nPoints = size(xy,1);
        a = meshgrid(1:nPoints);
        dmat = reshape(sqrt(sum((xy(a,:)-xy(a',:)).^2,2)),nPoints,nPoints);
    end
    
    % Verify Inputs
    [N,dims] = size(xy);
    [nr,nc] = size(dmat);
    if N ~= nr || N ~= nc
        error('Invalid XY or DMAT inputs!')
    end
    n = N;
    
    % Sanity Checks
    nSalesmen   = max(1,min(n,round(real(nSalesmen(1)))));
    minTour     = max(1,min(floor(n/nSalesmen),round(real(minTour(1)))));
    popSize     = max(8,8*ceil(popSize(1)/8));
    numIter     = max(1,round(real(numIter(1))));
    showProg    = logical(showProg(1));
    showResult  = logical(showResult(1));
    showWaitbar = logical(showWaitbar(1));
    
    % Initializations for Route Break Point Selection
    nBreaks = nSalesmen-1;
    dof = n - minTour*nSalesmen;          % degrees of freedom
    addto = ones(1,dof+1);
    for k = 2:nBreaks
        addto = cumsum(addto);
    end
    cumProb = cumsum(addto)/sum(addto);
    
    % Initialize the Populations
    popRoute = zeros(popSize,n);         % population of routes
    popBreak = zeros(popSize,nBreaks);   % population of breaks
    indexStations = cell(popSize,1);
    popRoute(1,:) = (1:n);
    popBreak(1,:) = rand_breaks();
    [numStations(1) indexStations{1}] = find_stations(popRoute(1,:),popBreak(1,:));
    for k = 2:popSize
        popRoute(k,:) = randperm(n);
        popBreak(k,:) = rand_breaks();
        [numStations(k) indexStations{k}] = find_stations(popRoute(k,:),popBreak(k,:));
    end
    
    % Select the Colors for the Plotted Routes
    pclr = ~get(0,'DefaultAxesColor');
    clr = [1 0 0; 0 0 1; 0.67 0 1; 0 1 0; 1 0.5 0];
    if nSalesmen > 5
        clr = hsv(nSalesmen);
    end
    
    % Run the GA
    globalMin = Inf;
    totalDist = zeros(1,popSize);
    distHistory = zeros(1,numIter);
    tmpPopRoute = zeros(8,n);
    tmpPopBreak = zeros(8,nBreaks);
    newPopRoute = zeros(popSize,n);
    newPopBreak = zeros(popSize,nBreaks);
    if showProg
        figure('Name','MTSPO_GA | Current Best Solution','Numbertitle','off');
        hAx = gca;
    end
    if showWaitbar
        hWait = waitbar(0,'Searching for near-optimal solution ...');
    end
    for iter = 1:numIter
        % Evaluate Members of the Population
        for p = 1:popSize
            ds = 0;
            pRoute = popRoute(p,:);
            pBreak = popBreak(p,:);
          
            pStation = indexStations{p};
            rng = [[1 pBreak+1];[pBreak n]]';
            for s = 1:nSalesmen
                d = 0;
                
                for k = rng(s,1):rng(s,2)-1
                    d = d + dmat(pRoute(k),pRoute(k+1));
                end

                temp(s)=d;
            end
            

            for j = 1:numStations(p)-1

                ds = ds + dmat(pStation(j),pStation(j+1));
            end
            % ds = ds + dmat(pStation(end),pStation(1)); % efficient for one station scenario
            totalDist(p) = sum(temp);
            totalTime(p) = max(temp);
            fun(p) = 1*sum(temp)+.8*ds;
        end
        
        % Find the Best Route in the Population
        [minFun,index] = min(fun);
        distHistory(iter) = minFun;
        if minFun < globalMin
            globalMin = minFun;
            optRoute = popRoute(index,:);
            optBreak = popBreak(index,:);
            optStations = indexStations{index};
            minDist = totalDist(index);
            minTime = totalTime(index);
            rng = [[1 optBreak+1];[optBreak n]]';
            if showProg
                % Plot the Best Route
                for s = 1:nSalesmen
                    rte = optRoute(rng(s,1):rng(s,2));
                    if dims > 2, plot3(hAx,xy(rte,1),xy(rte,2),xy(rte,3),'.-','linewidth',3,'Color',clr(s,:));
                    else plot(hAx,xy(rte,1),xy(rte,2),'.-','linewidth',2,'Color',clr(s,:)); end
                    hold(hAx,'on');
                end
                obfun = 1/4*(minDist/N)+1/4*(minTime/batteryLife)+1/4*(nSalesmen/1)+1/4*(numStations(index)/1);
                plot(xy(optStations,1),xy(optStations,2),'kx', 'linewidth', 2,'MarkerSize',10,'MarkerFaceColor','k')
                xlabel('X(mile)')
                ylabel('Y(mile)')
                % title(hAx,sprintf('Total Distance = %1.2f, Time = %1.2f, N_w = %d, N_c = %d, Fun = %1.2f',minDist,minTime,nSalesmen,numStations(index),obfun ));
                hold(hAx,'off');
                drawnow;
            end
        end
        
        % Genetic Algorithm Operators
        randomOrder = randperm(popSize);
        for p = 8:8:popSize
            rtes = popRoute(randomOrder(p-7:p),:);
            brks = popBreak(randomOrder(p-7:p),:);
            sson = indexStations(randomOrder(p-7:p));
            dists = fun(randomOrder(p-7:p));
            [ignore,idx] = min(dists); %#ok
            bestOf8Route = rtes(idx,:);
            bestOf8Break = brks(idx,:);
            bestOf8Station = sson(idx);
            routeInsertionPoints = sort(ceil(n*rand(1,2)));
            I = routeInsertionPoints(1);
            J = routeInsertionPoints(2);
            for k = 1:8 % Generate New Solutions
                tmpPopRoute(k,:) = bestOf8Route;
                tmpPopBreak(k,:) = bestOf8Break;
                tmpindexStations{k} = bestOf8Station;
                switch k
                    case 2 % Flip
                        tmpPopRoute(k,I:J) = tmpPopRoute(k,J:-1:I);
                        [tmpnumStations(k) tmpindexStations{k}] = find_stations(tmpPopRoute(k,:),tmpPopBreak(k,:));
                    case 3 % Swap
                        tmpPopRoute(k,[I J]) = tmpPopRoute(k,[J I]);
                        [tmpnumStations(k) tmpindexStations{k}] = find_stations(tmpPopRoute(k,:),tmpPopBreak(k,:));
                    case 4 % Slide
                        tmpPopRoute(k,I:J) = tmpPopRoute(k,[I+1:J I]);
                        [tmpnumStations(k) tmpindexStations{k}] = find_stations(tmpPopRoute(k,:),tmpPopBreak(k,:));
                    case 5 % Modify Breaks
                        tmpPopBreak(k,:) = rand_breaks();
                        [tmpnumStations(k) tmpindexStations{k}] = find_stations(tmpPopRoute(k,:),tmpPopBreak(k,:));
                    case 6 % Flip, Modify Breaks
                        tmpPopRoute(k,I:J) = tmpPopRoute(k,J:-1:I);
                        tmpPopBreak(k,:) = rand_breaks();
                        [tmpnumStations(k) tmpindexStations{k}] = find_stations(tmpPopRoute(k,:),tmpPopBreak(k,:));
                    case 7 % Swap, Modify Breaks
                        tmpPopRoute(k,[I J]) = tmpPopRoute(k,[J I]);
                        tmpPopBreak(k,:) = rand_breaks();
                        [tmpnumStations(k) tmpindexStations{k}] = find_stations(tmpPopRoute(k,:),tmpPopBreak(k,:));
                    case 8 % Slide, Modify Breaks
                        tmpPopRoute(k,I:J) = tmpPopRoute(k,[I+1:J I]);
                        tmpPopBreak(k,:) = rand_breaks();
                        [tmpnumStations(k) tmpindexStations{k}] = find_stations(tmpPopRoute(k,:),tmpPopBreak(k,:));
                    otherwise 
                        [tmpnumStations(k) tmpindexStations{k}] = find_stations(tmpPopRoute(k,:),tmpPopBreak(k,:));
                end
            end
            newPopRoute(p-7:p,:) = tmpPopRoute;
            newPopBreak(p-7:p,:) = tmpPopBreak;
            newindexStations(p-7:p) = tmpindexStations;
            newnumStations(p-7:p) = tmpnumStations;
        end
        popRoute = newPopRoute;
        popBreak = newPopBreak;
        indexStations = newindexStations;
        numStations = newnumStations;
        
        % Update the waitbar
        if showWaitbar && ~mod(iter,ceil(numIter/325))
            waitbar(iter/numIter,hWait);
        end
        
    end
    if showWaitbar
        close(hWait);
    end
    
    if showResult
        % Plots
        figure('Name','MTSPO_GA | Results','Numbertitle','off');
        subplot(2,2,1);
        if dims > 2, plot3(xy(:,1),xy(:,2),xy(:,3),'.','Color',pclr);
        else plot(xy(:,1),xy(:,2),'.','Color',pclr); end
        title('City Locations');
        subplot(2,2,2);
        imagesc(dmat(optRoute,optRoute));
        title('Distance Matrix');
        subplot(2,2,3);
        rng = [[1 optBreak+1];[optBreak n]]';
        for s = 1:nSalesmen
            rte = optRoute(rng(s,1):rng(s,2));
            if dims > 2, plot3(xy(rte,1),xy(rte,2),xy(rte,3),'.-','Color',clr(s,:));
            else plot(xy(rte,1),xy(rte,2),'.-','Color',clr(s,:)); end
            title(sprintf('Total Distance = %1.4f',minDist));
            hold on;
        end
        subplot(2,2,4);
        plot(distHistory,'b','LineWidth',2);
        title('Best Solution History');
        set(gca,'XLim',[0 numIter+1],'YLim',[0 1.1*max([1 distHistory])]);
    end
    
    % Return Output
    if nargout
        resultStruct = struct( ...
            'xy',          xy, ...
            'dmat',        dmat, ...
            'nSalesmen',   nSalesmen, ...
            'minTour',     minTour, ...
            'popSize',     popSize, ...
            'numIter',     numIter, ...
            'showProg',    showProg, ...
            'showResult',  showResult, ...
            'showWaitbar', showWaitbar, ...
            'optRoute',    optRoute, ...
            'optBreak',    optBreak, ...
            'optStations',    optStations, ...
            'minTime',     minTime, ...
            'minDist',     minDist);
        
        varargout = {resultStruct};
    end
    
    % Generate Random Set of Break Points
    function breaks = rand_breaks()
        if minTour == 1 % No Constraints on Breaks
            tmpBreaks = randperm(n-1);
            breaks = sort(tmpBreaks(1:nBreaks));
        else % Force Breaks to be at Least the Minimum Tour Length
            nAdjust = find(rand < cumProb,1)-1;
            spaces = ceil(nBreaks*rand(1,nAdjust));
            adjust = zeros(1,nBreaks);
            for kk = 1:nBreaks
                adjust(kk) = sum(spaces == kk);
            end
            breaks = minTour*(1:nBreaks) + cumsum(adjust);
        end
    end
    


function [numStations indexStations] = find_stations(route,breaks) %Bing added new function
        % Calculate length of each route
        lbreaks=diff([1 breaks n]);
        
        nStations(1)= floor(lbreaks(1)/batteryLife-0.000001);
        
        templm=ones(1,nStations(1))*batteryLife;
        if isempty(templm) == 1
            indexStations=[];
        else
            templm(1)=0+batteryLife;
            indexStations=cumsum(templm);
            
        end
        for i = 2 : length(lbreaks)
            % count how many stations for each salesmen
            
            nStations(i)= floor(lbreaks(i)/batteryLife-0.000001);
            
            templm=ones(1,nStations(i))*batteryLife;
            if isempty(templm) == 1
                indexStations=indexStations;
            else
                
                templm(1)=breaks(i-1)+batteryLife;
                indexStations=[indexStations cumsum(templm)];
                               
            end
        end
        indexStations = route(indexStations);
        numStations = sum(nStations);
       
    end

end

% Subfunction to override the default configuration with user inputs
function config = get_config(defaultConfig,userConfig)
    
    % Initialize the configuration structure as the default
    config = defaultConfig;
    
    % Extract the field names of the default configuration structure
    defaultFields = fieldnames(defaultConfig);
    
    % Extract the field names of the user configuration structure
    userFields = fieldnames(userConfig);
    nUserFields = length(userFields);
    
    % Override any default configuration fields with user values
    for i = 1:nUserFields
        userField = userFields{i};
        isField = strcmpi(defaultFields,userField);
        if nnz(isField) == 1
            thisField = defaultFields{isField};
            config.(thisField) = userConfig.(userField);
        end
    end
    
end

