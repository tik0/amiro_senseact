function [ ] = createFeatureVector( distances, pulsewidth)
%CREATEFEATUREVECTOR Summary of this function goes here
%   Detailed explanation goes here

%% Define Constants
deltaAngle = 90; % millidegree
rangeAngle = 90000; % millidegree
startAngle = -45000; % millidegree
angles = startAngle + deltaAngle : deltaAngle : startAngle + rangeAngle;

pulseWidthMax = 27604; % Maximum measured pulsewidth to staradized the pulsewith as a feature vector

muPW = 1.055613362500000e+04;
stdDevPW = 1.115634702856000e+04;

dataDepth = 16;
distancesSurface = zeros(dataDepth,numel(angles));
XSurface = zeros(dataDepth,numel(angles));
YSurface = 1:numel(angles);YSurface = repmat(YSurface,dataDepth,1);
YSurface = [1:dataDepth]';YSurface = repmat(YSurface,1,numel(angles));

%% Start the calculation
        % Distances plot
        subplot(4,2,1)
        
        distances(distances == hex2dec('7FFFFFFF')) = nan; % To noisy
        distances(distances == -hex2dec('80000000')) = nan; % To near
        distances(distances <= 40000) = nan; % To close (closer than 4m)
        invalidDataMask = isnan(distances);
        while (any(isnan(distances)))
            distances(isnan(distances)) = distances(circshift(isnan(distances),1,1));
        end
        scatter(angles, distances);
        title('Raw Distances')
        xlabel('Angle [m°]')
        ylabel('Distance [1/10 mm]')
        % Distances without angular dependency
        subplot(4,2,3)
        distancesNoAngleDep = distances' .* cosd(angles./1000);
        scatter(angles, distancesNoAngleDep);
        title('Distances without Angle Dependencies')
        xlabel('Angle [m°]')
        ylabel('Distance [1/10 mm]')
        
        % surface in eucledean space
        subplot(4,2,5)
        distancesSurface = circshift(distancesSurface,1);
        XSurface = circshift(XSurface,1);
%         YSurface = circshift(YSurface,1);
        [X,~] = pol2cart(fliplr(angles .* pi ./ 180 ./1000 + pi/2), distances');
        Y = distancesNoAngleDep;
        
        distancesSurface(1,:) = distancesNoAngleDep;
        XSurface(1,:) = X;
        distancesSurfaceSmoothed = zeros(size(distancesSurface));
        for idx = 1:dataDepth
            distancesSurfaceSmoothed(idx,:) = smooth(distancesSurface(idx,:),10)';
        end
        h = surf(XSurface, YSurface, distancesSurfaceSmoothed);
        set(h, 'edgecolor','none');
        view(0,90);
        title('Distances in Eucledian Space, stacked together')
        xlabel('Angle [m°]')
        ylabel('Index of Measurement')
        

        subplot(4,2,7)
        varianceOverDistances = sqrt(var(distancesSurface));
        plot(angles, varianceOverDistances)
        title('Variance over Stacked Distances')
        xlabel('Angle [m°]')
        ylabel('Index of Measurement')


       %% Pulsewidth
        subplot(4,2,2)
        % Prune invalid data, marked by invalid distances
        pulsewidth(invalidDataMask) = nan;
        while any(isnan(pulsewidth))
            pulsewidth(isnan(pulsewidth)) = pulsewidth(circshift(isnan(pulsewidth),1,1));
        end
        % Plot the pulsewidth
        scatter(angles, pulsewidth);
        xlabel('Angle [m°]')
        ylabel('Pulsewidth')
        % Pulsewidth descriminated
        pulsewidthNorm = pulsewidth ./ pulseWidthMax;
        
        % Plot sliding window variance
        subplot(4,2,4)
        pulsewidthFeature =  slidingFckt(pulsewidthNorm, 21, 'boxcar');
        scatter(angles, pulsewidthFeature);
        xlabel('Angle [m°]')
        ylabel('Pulsewidth Feature')
        title('Boxcar Varianze of Pulsewidth')
        
        % Plot sliding window variance
        subplot(4,2,6)
        pulsewidthFeature =  slidingFckt(pulsewidthNorm, 21, 'gauss');
        scatter(angles, pulsewidthFeature);
        xlabel('Winkel [m°]')
        ylabel('Pulsewidth Feature')
        title('Gaussian Varianze of Pulsewidth')
        
        % Plot the product of the features
        subplot(4,2,8)
        plot(angles, sqrt(var(distancesSurface)).*slidingFckt(pulsewidthNorm, 21, 'boxcar')')
        xlabel('Winkel [m°]')
        ylabel('Pulsewidth Feature')
        title('Product of Features')
        
        % Discrimination between acre and stock 
%         subplot(4,2,6)
%         colorMap = zeros(size(pulsewidthNormPruned));
%         colorMap(pulsewidthFeature <= 0.001) = 1;
%         colorMap(pulsewidthFeature > 0.001) = 2;
%         plot(angles, colorMap);
        getframe;

end

