(function () {
    var CLOSE_ENOUGH_TO_GOAL = .2,
        ANGULAR_ACCEL = 1.0, ANGULAR_INC = .1, ANGULAR_MIN = -.4, ANGULAR_MAX = .4,
        LINEAR_ACCEL = 1.0, LINEAR_INC = .1, LINEAR_MIN = 0, LINEAR_MAX = .75,
        DT = .1, 
        CLEARANCE_MIN = .6, CLEARANCE_MAX = 2.25,
        LINEAR_WEIGHT = .25, CLEARANCE_WEIGHT = .2, GOAL_DIR_WEIGHT = .4,
        SLOW_DOWN_RADIUS = 1.0, SIZE_RADIUS = .6, BUFFER_SPACE = .1,
        NUM_OBSTACLES = 0,
        
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
            
            for (var angular = cur_angular - ANGULAR_ACCEL; 
                    angular <= cur_angular + ANGULAR_ACCEL;
                    angular += ANGULAR_INC) 
            {
                if (angular < ANGULAR_MIN || angular > ANGULAR_MAX) {
                    continue;
                }
                
                for (var linear = cur_linear - LINEAR_ACCEL; 
                        linear <= cur_linear + LINEAR_ACCEL;
                        linear += LINEAR_INC) 
                {
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
                    var res = clib.calcIntersection(cur_x, cur_y, cur_dir, linear, angular, cloud);
                    
                    if (res.point === false) { 
                        var clearance = CLEARANCE_MAX,
                            color = "gray";
                    
                        if (clearance < CLEARANCE_MIN) {
                            console.log("gotcha");
                            continue;
                        }
                    } else if (res.delta > CLEARANCE_MAX) {
                        var clearance = CLEARANCE_MAX,
                            color = "gray";
                    
                        if (clearance < CLEARANCE_MIN) {
                            console.log("gotcha");
                            continue;
                        }
                        
                        Plotter.plotPoint(res.point.x, res.point.y, color, .1);
                    } else {
                        var clearance = res.delta,
                            color = "red";
                    
                        if (clearance < CLEARANCE_MIN) {
                            console.log("gotcha");
                            continue;
                        }
                        
                        Plotter.plotPoint(res.point.x, res.point.y, color, .1);
                    }
                    
                    Plotter.plotPoint(res.point.x, res.point.y, color, .1);
                        
                    if (res.traj.type === "circle") {
                        Plotter.plotCircle(res.traj.x, res.traj.y, color, res.traj.r);
                    } else if (res.traj.type === "vector") {
                        Plotter.plotVector(res.traj.x, res.traj.y, res.traj.dir, color);
                    }
                    
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
                        angular: angular,
                        clearanceNorm: clearanceNorm,
                        goalDirDifNorm: goalDirDifNorm,
                        linearNorm: linearNorm
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
            
            /*
            console.log(
                options[best_index].clearanceNorm,
                options[best_index].goalDirDifNorm,
                options[best_index].linearNorm
                );
            */
            
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
    for (var i = 0; i < NUM_OBSTACLES; i++) {
        cloud.push({x: Math.random()*26-13, 
                    y: Math.random()*14-7,
                    r: SIZE_RADIUS + BUFFER_SPACE,
                    dir: Math.random()*2*Math.PI});
    }
    /*
    OBSTACLE_VELOCITY = .01;
    
    var obstacleStep = function () {
        for (var i = 0; i < NUM_OBSTACLES; i++) {
            cloud[i].x += OBSTACLE_VELOCITY*Math.cos(cloud[i].dir);
            cloud[i].y += OBSTACLE_VELOCITY*Math.sin(cloud[i].dir);
        }
    };
    
    setInterval(obstacleStep, 100);
    */
        
    var step = function () {
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
        Plotter.plotPoint(cur_x, cur_y, "lightGreen", SIZE_RADIUS);
        Plotter.plotCircle(cur_x, cur_y, "darkGreen", SIZE_RADIUS);
        Plotter.drawAxises(1, 1);
        
        for (var i = 0; i < spec.cloud.length; i++) {
            Plotter.plotPoint(spec.cloud[i].x, spec.cloud[i].y, "black", .05);
            Plotter.plotCircle(spec.cloud[i].x, spec.cloud[i].y, "gray", spec.cloud[i].r);
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
    
    selectNewGoal = function () {
        LINEAR_WEIGHT = document.getElementById("linearVel").value + 0;
        CLEARANCE_WEIGHT = document.getElementById("clearance").value + 0;
        GOAL_DIR_WEIGHT = document.getElementById("goalDir").value + 0;
        
        
        var coords = Plotter.getPlotCoords(
            Math.random()*canvas.width,
            Math.random()*canvas.height
            );
            
        goal_x = coords[0];
        goal_y = coords[1];
        
        setTimeout(selectNewGoal, 10000);
    }
    
    setInterval(step, 50);
    selectNewGoal();
})();
