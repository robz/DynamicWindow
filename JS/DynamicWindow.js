(function () {
    var CLOSE_ENOUGH_TO_GOAL = .1,
        ANGULAR_ACCEL = Math.PI/2, ANGULAR_SAMPLES = 10, ANGULAR_MIN = -Math.PI/2, ANGULAR_MAX = Math.PI/2,
        LINEAR_ACCEL = .4, LINEAR_SAMPLES = 10, LINEAR_MIN = 0, LINEAR_MAX = .8,
        DT = .1, 
        CLEARANCE_MIN = .6, CLEARANCE_MAX = 1.4,
        LINEAR_WEIGHT = .1, CLEARANCE_WEIGHT = 10, GOAL_DIR_WEIGHT = .1,
        SLOW_DOWN_RADIUS = 2,
        
        euclidDist = function (x1, y1, x2, y2) {
            return Math.sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
        },
        
        boundAngle0to2Pi = function (angle) {
            return angle - Math.floor(angle/(2*Math.PI))*2*Math.PI;
        },
        
        angleDif = function (ang1, ang2) {
            var res = boundAngle0to2Pi(ang1) - boundAngle0to2Pi(ang2);
            
            if (res > Math.PI) {
                res -= Math.PI*2;
            } else if (res < -Math.PI) {
                res += Math.PI*2;
            }
            
            return res;
        },
        
        calcClearance = function (cloud, x, y) {
            if (cloud.length === 0) {
                return CLEARANCE_MAX;
            }
            
            var min_clearance = euclidDist(
                cloud[0].x, 
                cloud[0].y, 
                x, 
                y
                );
            
            for (var i = 1; i < cloud.length; i++) {
                var clearance = euclidDist(
                    cloud[i].x, 
                    cloud[i].y, 
                    x, 
                    y
                    );
                
                if (clearance < min_clearance) {
                    min_clearance = clearance;
                }
            }
            
            if (min_clearance > CLEARANCE_MAX) {
                min_clearance = CLEARANCE_MAX;
            }
            
            return min_clearance - CLEARANCE_MIN;
        },

        calcKinematic = function (cur_x, cur_y, cur_dir, linear, angular) {
            var dir = cur_dir + DT*angular;
            
            if (Math.abs(angular) < 1e-2) {
                x = cur_x + DT*linear*Math.cos(cur_dir);
                y = cur_y + DT*linear*Math.sin(cur_dir);
            } else if (angular < 0) {
                var len = DT*linear;
                var beta = cur_dir - dir; 
                var R = len/beta;

                x = cur_x + R*Math.cos(cur_dir - Math.PI/2) + R*Math.cos(cur_dir + Math.PI/2 - beta);
                y = cur_y + R*Math.sin(cur_dir - Math.PI/2) + R*Math.sin(cur_dir + Math.PI/2 - beta);
            } else if (angular > 0) {
                var len = DT*linear;
                var beta = dir - cur_dir; 
                var R = len/beta;

                x = cur_x + R*Math.cos(cur_dir + Math.PI/2) + R*Math.cos(cur_dir - Math.PI/2 + beta);
                y = cur_y + R*Math.sin(cur_dir + Math.PI/2) + R*Math.sin(cur_dir - Math.PI/2 + beta);
            } 
            
            return [x, y, dir];
        },
        
        // returns true on success
        DynamicWindow = function (spec) {
            if (typeof spec.action_out === "undefined") {
                return false;
            }
            
            cur_x = spec.cur_x; 
            cur_y = spec.cur_y;
            cur_dir = spec.cur_dir;
            cur_linear = spec.cur_linear;
            cur_angular = spec.cur_angular;
            cloud = spec.cloud;
            goal_x = spec.goal_x; 
            goal_y = spec.goal_y;
            
            var distToGoal = euclidDist(goal_x, goal_y, cur_x, cur_y);
            if (distToGoal < CLOSE_ENOUGH_TO_GOAL) {
                spec.action_out.linear = 0;
                spec.action_out.angular = 0;
                return true;
            }
            
            var options = [];
            
            var ang_inc = ANGULAR_ACCEL*2/(ANGULAR_SAMPLES - 1),
                lin_inc = LINEAR_ACCEL*2/(LINEAR_SAMPLES - 1);
            /*    
            console.log(cur_angular + ang_inc*~~(-(ANGULAR_SAMPLES - 1)/2), 
                        cur_angular + ang_inc*~~((ANGULAR_SAMPLES - 1)/2));
            console.log(cur_linear + lin_inc*~~(-(LINEAR_SAMPLES - 1)/2), 
                        cur_linear + lin_inc*~~((LINEAR_SAMPLES - 1)/2));
            */
            
            for (var ang_index = ~~(-(ANGULAR_SAMPLES - 1)/2); 
                 ang_index <= ~~((ANGULAR_SAMPLES - 1)/2);
                 ang_index += 1) 
            {
                var angular = cur_angular + ang_index*ang_inc;
                
                if (angular < ANGULAR_MIN || angular > ANGULAR_MAX) {
                    continue;
                }
                
                var clearance = CLEARANCE_MAX;
                
                for (var lin_index = ~~(-(LINEAR_SAMPLES - 1)/2); 
                     lin_index <= ~~((LINEAR_SAMPLES - 1)/2);
                     lin_index += 1) 
                {
                    var linear = cur_linear + lin_index*lin_inc;
                    
                    if (linear < LINEAR_MIN || linear > LINEAR_MAX) {
                        continue;
                    } 
                    
                    // calcular new point
                    var tmp = calcKinematic(cur_x, cur_y, cur_dir, linear, angular)
                    x = tmp[0]
                    y = tmp[1]
                    dir = tmp[2]
                           
                    // calculate clearance
                    var local_clearance = calcClearance(cloud, x, y);
                    
                    clearance += 1;
                    
                    // ignore if the point is too close to obstacles
                    if (local_clearance < 0) {
                        break;
                    }
                }
                
                for (var lin_index = ~~(-(LINEAR_SAMPLES - 1)/2); 
                     lin_index <= ~~((LINEAR_SAMPLES - 1)/2);
                     lin_index += 1) 
                {
                    // ignore if the point is too close to obstacles
                    if (clearance < 0) {
                        break;
                    }
                    
                    var linear = cur_linear + lin_index*lin_inc;
                    
                    if (linear < LINEAR_MIN || linear > LINEAR_MAX) {
                        continue;
                    } 
                    
                    // calcular new point
                    var tmp = calcKinematic(cur_x, cur_y, cur_dir, linear, angular)
                    x = tmp[0]
                    y = tmp[1]
                    dir = tmp[2]
                    
                    Plotter.drawArrow(x, y, dir);
                    
                    // calculate normalized clearance weight
                    var clearanceNorm = clearance/CLEARANCE_MAX;
                    
                    // calculate normalized goal direction weight
                    var goalDir = Math.atan2(goal_y - cur_y, goal_x - cur_x);
                    var goalDirDif = Math.PI - Math.abs(angleDif(goalDir, dir));
                    var goalDirDifNorm = goalDirDif/Math.PI;
                    
                    // calculate normalized linear velocity weight
                    var linearNorm = linear/LINEAR_MAX;
                    
                    // put it all together
                    var weight = clearanceNorm*CLEARANCE_WEIGHT +
                                 goalDirDifNorm*GOAL_DIR_WEIGHT + 
                                 linearNorm*LINEAR_WEIGHT;
                    
                    options.push({
                        weight: weight, 
                        linear: linear, 
                        angular: angular
                    });
                }
            }
            
            if (options.length === 0) {
                spec.action_out.linear = 0;
                spec.action_out.angular = 0;
                return false;
            }
            
            if (typeof nonexistantFlag === "undefined") {
                console.log(options);
                nonexistantFlag = true;
            }
            
            var best_weight = options[0].weight;
            var best_index = 0;
            
            for (var i = 1; i < options.length; i++) {
                if (options[i].weight > best_weight) {
                    best_weight = options[i].weight;
                    best_index = i;
                }
            }
            
            // slow down when approaching the goal
            if (distToGoal < SLOW_DOWN_RADIUS) {
                options[best_index].linear *= (distToGoal/SLOW_DOWN_RADIUS);
            }
            
            spec.action_out.linear = options[best_index].linear;
            spec.action_out.angular = options[best_index].angular;
            return true;
        };
    
    document.getElementById("linearVel").value = "" + LINEAR_WEIGHT;
    document.getElementById("clearance").value = "" + CLEARANCE_WEIGHT;
    document.getElementById("goalDir").value = "" + GOAL_DIR_WEIGHT;
    
    var canvas = document.getElementById("canvas");
    
    Plotter.init(canvas, .5, .5, 50, 50);
    Plotter.clear();
    Plotter.drawAxises(1, 1);
    
    var cur_x = 0,
        cur_y = 0,
        cur_dir = 0,
        goal_x = 0, 
        goal_y = 0, 
        cur_lin = 0,
        cur_ang = 0;
        
    var cloud = [];
    for (var i = 0; i < 80; i++) {
        cloud.push({x: Math.random()*26-13, y: Math.random()*14-7});
    }
        
    var step = function() {
        var spec = {
            action_out: {linear: 0, angular: 0},
            cur_x: cur_x,
            cur_y: cur_y,
            cur_dir: cur_dir,
            cur_linear: cur_lin,
            cur_angular: cur_ang,
            cloud: cloud,
            goal_x: goal_x,
            goal_y: goal_y,
        }
        
        Plotter.clear();
        Plotter.plotPoint(cur_x, cur_y, "lightBlue", CLEARANCE_MAX);
        Plotter.plotPoint(cur_x, cur_y, "lightGreen", CLEARANCE_MIN);
        Plotter.drawAxises(1, 1);
        
        for (var i = 0; i < spec.cloud.length; i++) {
            Plotter.plotPoint(spec.cloud[i].x, spec.cloud[i].y)
        }
        
        Plotter.plotPoint(goal_x, goal_y, "blue", .1);

        DynamicWindow(spec);  
        
        var linear = spec.action_out.linear,
            angular = spec.action_out.angular;        

        var tmp = calcKinematic(cur_x, cur_y, cur_dir, linear, angular);
        
        cur_x = tmp[0];
        cur_y = tmp[1];
        cur_dir = tmp[2];  
        cur_lin = linear;
        cur_ang = angular;          

        Plotter.drawRedArrow(cur_x, cur_y, cur_dir);
    };
    
    canvas.onmousedown = function (event) {
        event.preventDefault();
        
        LINEAR_WEIGHT = document.getElementById("linearVel").value + 0;
        CLEARANCE_WEIGHT = document.getElementById("clearance").value + 0;
        GOAL_DIR_WEIGHT = document.getElementById("goalDir").value + 0;
        
        var coords = Plotter.getPlotCoords(event.offsetX, event.offsetY);
        goal_x = coords[0];
        goal_y = coords[1];
    };
    
    setInterval(step, 50);
})();
