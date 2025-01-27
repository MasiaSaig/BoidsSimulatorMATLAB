classdef Boid < handle
    properties
        position = [0, 0];      % boids position in world
        velocity = [0, 0];
        color_r = 0;
        color_g = 0;
        color_b = 0;
    end

    properties (Access = private)
        neighbors = [];         % boids that are close to the boid, to apply cohension
        neighborsAlignment = [] % boids that are close to the boid, to apply alignment
        close_neighbors = [];   % boids that are too close.
        idx_neighbor = 0;
        idx_neighborAlignment = 0;
        idx_closeNeighbor = 0; 
    end

    methods (Static)
      function out = maxSpeed(newSpeed)
         persistent static_maxSpeed;
         if nargin
            static_maxSpeed = newSpeed;
         end
         out = static_maxSpeed;
      end

      function out = width(newWidth)
         persistent static_width;
         if nargin
            Boid.leftBorderSide(newWidth*0.15)
            Boid.rightBorderSide(newWidth*0.85);
            static_width = newWidth;
         end
         out = static_width;
      end

      function out = height(newHeight)
         persistent static_height;
         if nargin
            Boid.topBorderSide(newHeight*0.85);
            Boid.bottomBorderSide(newHeight*0.15);
            static_height = newHeight;
         end
         out = static_height;
      end
      
      function out = borderAvoidanceMultiplier(data)
          persistent static_borderAvoidance;
         if nargin
            static_borderAvoidance = data;
         end
         out = static_borderAvoidance;
      end
      function out = cohensionMultiplier(data)
         persistent static_cohension;
         if nargin
            static_cohension = 5*data;
         end
         out = static_cohension;
      end
      function out = separationMultiplier(data)
         persistent static_separation;
         if nargin
            static_separation = 2.5*data;
         end
         out = static_separation;
      end
      function out = alignmentMultiplier(data)
         persistent static_alignment;
         if nargin
            static_alignment = 5*data;
         end
         out = static_alignment;
      end
      function out = predatorAvoidanceMultiplier(data)
         persistent static_predatorAvoidance;
         if nargin
            static_predatorAvoidance = 5*data;
         end
         out = static_predatorAvoidance;
      end

      function out = borderStep(data)
            persistent static_borderStep;
            if nargin
               static_borderStep = data;
            end
            out = static_borderStep;
        end
   end

    methods (Static, Access = private)
        function out = leftBorderSide(data)
            persistent static_leftBorderSide;
            if nargin
               static_leftBorderSide = data;
            end
            out = static_leftBorderSide;
        end
        function out = rightBorderSide(data)
            persistent static_rightBorderSide;
            if nargin
               static_rightBorderSide = data;
            end
            out = static_rightBorderSide;
        end
        function out = topBorderSide(data)
            persistent static_topBorderSide;
            if nargin
               static_topBorderSide = data;
            end
            out = static_topBorderSide;
        end
        function out = bottomBorderSide(data)
            persistent static_bottomBorderSide;
            if nargin
               static_bottomBorderSide = data;
            end
            out = static_bottomBorderSide;
        end
        %{
        function out = maxNeighbors(data)
            persistent static_maxNeighbors;
            if nargin
               static_maxNeighbors = data;
            end
            out = static_maxNeighbors;
        end
        %}
    end

    methods (Access = public)
        function preallocateArrays(obj, neighborsNum)
            %Boid.maxNeighbors(neighborsNum);
            obj.neighbors = zeros(2, neighborsNum);
            obj.close_neighbors = zeros(2, neighborsNum);
            obj.neighborsAlignment = zeros(2, neighborsNum);
        end
        
        % Method that updates the coordinate of the boid according to its new velocity.
        % The new velocity is adjusted by its previous velocity, and the 5 rules, 
        % which are: cohesion, separation, alignment, edge avoidance, and predator avoidance.
        function update(obj, boids, predators, predatorsCount)
            obj.findNeighbors(boids);
            [v1x, v1y] = obj.avoid_edge();
            v1x = v1x * Boid.borderAvoidanceMultiplier; v1y = v1y * Boid.borderAvoidanceMultiplier;
            [v2x, v2y] = obj.cohesion();
            %v2x = v2x * Boid.cohensionMultiplier; v2y = v2y * Boid.cohensionMultiplier;
            [v3x, v3y] = obj.separation();
            %v3x = v3x * Boid.separationMultiplier; v3y = v3y * Boid.separationMultiplier;
            [v4x, v4y] = obj.alignment();
            %v4x = v4x * Boid.alignmentMultiplier; v4y = v4y * Boid.alignmentMultiplier;
            [v5x, v5y] = obj.avoid_predators(predators, predatorsCount);
            %v5x = v5x * Boid.predatorAvoidanceMultiplier; v5y = v5y * Boid.predatorAvoidanceMultiplier;
          
            % New velocity is previous velocity plus change of velocity due to the rules
            obj.velocity = [obj.velocity(1) + (v1x + v2x + v3x + v4x + v5x) * 0.5, ...
                            obj.velocity(2) + (v1y + v2y + v3y + v4y + v5y) * 0.5];
            obj.limit_speed();
            obj.preventMovingOutsideBorders();
            obj.position = [obj.position(1) + obj.velocity(1), obj.position(2) + obj.velocity(2)];

            % Reset the neighbors arrays
            obj.idx_neighbor = 0;
            obj.idx_neighborAlignment = 0;
            obj.idx_closeNeighbor = 0;
        end
        
        % Method that finds other nearby Boids.
        function findNeighbors(obj, boids)
            for i = 1 : numel(boids)
                if boids(i) ~= obj
                    distance = sqrt((obj.position(1) - boids(i).position(1))^2 + (obj.position(2) - boids(i).position(2))^2);
                    % Add boid to neighbors, if boids distance is less or equal to set cohension.
                    % Cohesion rule applies for them.
                    if distance < Boid.cohensionMultiplier
                        obj.idx_neighbor = obj.idx_neighbor + 1;
                        obj.neighbors(1, obj.idx_neighbor) = boids(i).position(1);
                        obj.neighbors(2, obj.idx_neighbor) = boids(i).position(2);
                    end
                    % Add boid to alignment neighbors, if boids distance is less or equal to set alignment.
                    % Separation rule applies for them.
                    if distance < Boid.alignmentMultiplier
                        obj.idx_neighborAlignment = obj.idx_neighborAlignment + 1;
                        obj.neighborsAlignment(1, obj.idx_neighborAlignment) = boids(i).velocity(1);
                        obj.neighborsAlignment(2, obj.idx_neighborAlignment) = boids(i).velocity(2);
                    end
                    % Add boid to close neighbors, if boids distance is less or equal to set separation.
                    % Separation rule applies for them.
                    if distance < Boid.separationMultiplier
                        obj.idx_closeNeighbor = obj.idx_closeNeighbor + 1;
                        obj.close_neighbors(1, obj.idx_closeNeighbor) = boids(i).position(1);
                        obj.close_neighbors(2, obj.idx_closeNeighbor) = boids(i).position(2);
                    end
                end
            end
        end
        
        % Rule 1. Method that calculates the velocity change due to cohesion factor. 
        % The method calculates the average position of its neighbors, and returns a vector,
        % that is from the boids current position to the average position divided by a coefficient.
        function [x, y] = cohesion(obj)
            avg_position = [0, 0];
            if obj.idx_neighbor == 0
                x = 0;
                y = 0;
            else
                for i = 1 : obj.idx_neighbor
                    avg_position = avg_position + [obj.neighbors(1, i) obj.neighbors(2, i)];
                end
                avg_position = avg_position / obj.idx_neighbor;
                x = (avg_position(1) - obj.position(1)) * 0.05;
                y = (avg_position(2) - obj.position(2)) * 0.05;
            end
        end
        
        % Rule 2. Method that calculates the velocity change due to alignment factor. 
        % The method calculates the average velocity, divides it by a coefficient, and returns the result.
        function [x, y] = alignment(obj)
            if obj.idx_neighborAlignment == 0
                x = 0;
                y = 0;
            else
                avg_vector = [0, 0];
                for i = 1 : obj.idx_neighborAlignment
                    avg_vector = [avg_vector(1) + obj.neighborsAlignment(1, i), ...
                                  avg_vector(2) + obj.neighborsAlignment(2, i)];
                end
                x = avg_vector(1) / obj.idx_neighborAlignment * 0.25;
                y = avg_vector(2) / obj.idx_neighborAlignment * 0.25;
            end
        end
        
        % Rule 3. Method that calculates the velcotiy change due to separation factor.
        % First, calculate the coordinate difference between the boid and its close neighbors, 
        % add them all, and average them. Them. change the sign of the vector so that it faces 
        % away from the other boids, and divide it by a coefficient.
        function [x, y] = separation(obj)
            goal_pos = [0, 0];
            if obj.idx_closeNeighbor == 0
                x = 0;
                y = 0;
            else
                for i = 1 : obj.idx_closeNeighbor
                    goal_pos = [goal_pos(1) - (obj.close_neighbors(1,i) - obj.position(1)), ...
                                goal_pos(2) - (obj.close_neighbors(2,i) - obj.position(2))];
                end
                x = goal_pos(1) / obj.idx_closeNeighbor * 0.5;
                y = goal_pos(2) / obj.idx_closeNeighbor * 0.5;
            end
        end
        
        % Rule 4. Method that makes the boid turn if it is close to the border. 
        % The method checks, if the boid is approaching the edge and if it is, then the method 
        % returns a vector which faces the opposite direction of the boids direction.
        function [x, y] = avoid_edge(obj)
            x = 0;
            y = 0;
            
            if obj.position(1) < Boid.leftBorderSide
                % left border
                x = Boid.borderStep - (obj.position(1) / Boid.leftBorderSide)*Boid.borderStep;
            elseif obj.position(1) > Boid.rightBorderSide
                % right border
                x = -((obj.position(1) - Boid.rightBorderSide) / Boid.leftBorderSide) * Boid.borderStep;
            end

            if obj.position(2) < Boid.bottomBorderSide
                % bottom border
                y = Boid.borderStep  - (obj.position(2) / Boid.bottomBorderSide)*Boid.borderStep;
            elseif obj.position(2) > Boid.topBorderSide
                % top border
                y = -((obj.position(2) - Boid.topBorderSide) / Boid.bottomBorderSide) * Boid.borderStep;
            end
        end
        
        % Rule 5. Method that makes boid turn away from the predator.
        % If a predator is within a certain distance from the boid, then it returns a vector 
        % that faces to the opposite direction of the predator, divided b a coefficient.
        function [x, y] = avoid_predators(obj, predators, predators_count)
            x = 0;
            y = 0;
            if predators == 0
                return;
            else
                if predators_count ~= 0
                    close_predators_num = 0;
                    x_temp = obj.position(1);
                    y_temp = obj.position(2);
                    parfor i = 1 : predators_count
                        % Calculate distance between boid and predator.
                        dx = x_temp - predators(i).position(1);
                        dy = y_temp - predators(i).position(2);
                        distance = sqrt(dx^2 + dy^2);
                        % If distance is closer than set predator avoidance, then calculate vector that faces directly opposite from predator.
                        if distance < Boid.predatorAvoidanceMultiplier
                            close_predators_num = close_predators_num + 1;
                            x = x + (dx * Boid.predatorAvoidanceMultiplier / distance);
                            y = y + (dy * Boid.predatorAvoidanceMultiplier / distance);
                        end
                    end
                    % Add all vectors created from each nearby predators, and divide it by the number of predators. 
                    % Then, return the resulting vector.
                    if close_predators_num ~= 0
                        x = x / close_predators_num * 0.066;
                        y = y / close_predators_num * 0.066;
                    end
                end
            end
        end
        
        
        % Method that checks the speed of the boid, and reduces the speed, if it exceeds the limit speed.
        function limit_speed(obj)
            curr_speed = sqrt(obj.velocity(1)^2 + obj.velocity(2)^2);
            if curr_speed > Boid.maxSpeed
                obj.velocity = obj.velocity * (Boid.maxSpeed / curr_speed);
            end
        end

        function preventMovingOutsideBorders(obj)
            if obj.position(1) >= Boid.width
                obj.velocity(1) = -1;
            end
            if obj.position(1) <= 0
                obj.velocity(1) = 1;
            end
            if obj.position(2) >= Boid.height
                obj.velocity(2) = -1;
            end
            if obj.position(2) <= 0
                obj.velocity(2) = 1;
            end
        end
    end
end
