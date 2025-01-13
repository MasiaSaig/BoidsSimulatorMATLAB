classdef Predator < handle
     properties
        position = [0, 0];
        velocity = [0, 0];
     end

     properties (Access = private)
        neighbors = [];
        close_neighbors = [];
        idx_neighbor = 0;
        idx_closeNeighbor = 0; 
     end
       
    methods (Static)
        function out = maxSpeedPredator(newSpeed)
            persistent static_maxSpeed;
            if nargin
               static_maxSpeed = newSpeed;
            end
            out = static_maxSpeed;
        end
        function out = visionDistance(data)
            persistent static_visionDistance;
            if nargin
               static_visionDistance = data;
            end
            out = static_visionDistance;
        end
        function out = width(newWidth)
         persistent static_width;
         if nargin
            Predator.leftBorderSide(newWidth*0.1)
            Predator.rightBorderSide(newWidth*0.9);
            static_width = newWidth;
         end
         out = static_width;
      end

      function out = height(newHeight)
         persistent static_height;
         if nargin
            Predator.topBorderSide(newHeight*0.9);
            Predator.bottomBorderSide(newHeight*0.1);
            static_height = newHeight;
         end
         out = static_height;
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
    end

    methods (Access = public)
        function preallocateArrays(obj, neighborsNum)
            obj.neighbors = zeros(2, neighborsNum);
            obj.close_neighbors = zeros(2, neighborsNum);
        end

        function update(obj, boids, boidsNum, predators, predatorNum)
            % First, find boids that are close to the pred.
            obj.findNeighbors(boids, boidsNum, predators, predatorNum);
            [v1x, v1y] = obj.avoid_edge();
            [v2x, v2y] = obj.chase();
            [v3x, v3y] = obj.separation();
            disp(obj.idx_neighbor);
            if obj.idx_neighbor > 0
                obj.velocity(1) = 2*(v1x + v2x + v3x);
                obj.velocity(2) = 2*(v1y + v2y + v3y);
            else
                obj.velocity(1) = obj.velocity(1) + (v1x + v2x + v3x);
                obj.velocity(2) = obj.velocity(2) + (v1y + v2y + v3y);
            end
            obj.limit_speed();
            obj.position = [obj.position(1) + obj.velocity(1), obj.position(2) + obj.velocity(2)];
            
            % Reset the neighbors arrays
            obj.idx_neighbor = 0;
            obj.idx_closeNeighbor = 0;
        end
    end

    methods (Access = private)        
        function findNeighbors(obj, boids, boidsNum, predators, predatorsNum)
            for i = 1 : boidsNum
                if boids(i) ~= obj
                    distance = sqrt((obj.position(1) - boids(i).position(1))^2 + (obj.position(2) - boids(i).position(2))^2);
                    if distance < Predator.visionDistance
                        obj.idx_neighbor = obj.idx_neighbor + 1;
                        obj.neighbors(1, obj.idx_neighbor) = boids(i).position(1);
                        obj.neighbors(2, obj.idx_neighbor) = boids(i).position(2);
                    end
                end
            end
            for i=1:predatorsNum
                distance = sqrt((obj.position(1) - predators(i).position(1))^2 + (obj.position(2) - predators(i).position(2))^2);
                if distance <= 20
                        obj.idx_closeNeighbor = obj.idx_closeNeighbor + 1;
                        obj.close_neighbors(1, obj.idx_closeNeighbor) = predators(i).position(1);
                        obj.close_neighbors(2, obj.idx_closeNeighbor) = predators(i).position(2);
                end
            end
        end
        
        function [x, y] = avoid_edge(obj)
            x = 0;
            y = 0;
            if obj.position(1) < Predator.leftBorderSide
                % left border
                x = Predator.borderStep - (obj.position(1) / Predator.leftBorderSide)*Predator.borderStep;
            elseif obj.position(1) > Predator.rightBorderSide
                % right border
                x = -((obj.position(1) - Predator.rightBorderSide) / Predator.leftBorderSide) * Predator.borderStep;
            end

            if obj.position(2) < Predator.bottomBorderSide
                % bottom border
                y = Predator.borderStep  - (obj.position(2) / Predator.bottomBorderSide)*Predator.borderStep;
            elseif obj.position(2) > Predator.topBorderSide
                % top border
                y = -((obj.position(2) - Predator.topBorderSide) / Predator.bottomBorderSide) * Predator.borderStep;
            end
        end

        % Method that makes preds chase boids when they are near them.
        function [x, y] = chase(obj)
            % Try to move to the average position of the neighbors, 
            % which makes the pred move towards the flock of boids.
            avg_position = [0, 0];
            if obj.idx_neighbor == 0
                x = 0;
                y = 0;
            else
                for i = 1 : obj.idx_neighbor
                    avg_position = avg_position + [obj.neighbors(1, i) obj.neighbors(2, i)];
                end
                avg_position = avg_position / obj.idx_neighbor;
                x = (avg_position(1) - obj.position(1)) * 0.5;
                y = (avg_position(2) - obj.position(2)) * 0.5;
            end
        end

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
                x = goal_pos(1) / obj.idx_closeNeighbor * 0.25;
                y = goal_pos(2) / obj.idx_closeNeighbor * 0.25;
            end
        end

        function limit_speed(obj)
            curr_speed = sqrt(obj.velocity(1)^2 + obj.velocity(2)^2);
            if curr_speed > Predator.maxSpeedPredator
                obj.velocity = obj.velocity * (Predator.maxSpeedPredator / curr_speed);
            end
        end
    end
end
