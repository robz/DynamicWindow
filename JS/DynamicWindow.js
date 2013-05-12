(function () {
    var CLOSE_ENOUGH_TO_GOAL = .5,
        ANGULAR_ACCEL = 1.0, ANGULAR_INC = .1, ANGULAR_MIN = -.4, ANGULAR_MAX = .4,
        LINEAR_ACCEL = 1.0, LINEAR_INC = .1, LINEAR_MIN = 0, LINEAR_MAX = .75,
        DT = .1, 
        CLEARANCE_MIN = .2, CLEARANCE_MAX = 4,
        LINEAR_WEIGHT = .25, CLEARANCE_WEIGHT = .2, GOAL_DIR_WEIGHT = .4,
        SLOW_DOWN_RADIUS = 1.0, SIZE_RADIUS = .6, BUFFER_SPACE = .1,
        NUM_OBSTACLES = 200,
        
        // returns true on success
        DynamicWindow = function (
            action_out,
            cur_x,
            cur_y,
            cur_dir,
            cur_linear,
            cur_angular,
            mycloud,
            goal_x, 
            goal_y
            ) 
        {
            var distToGoal = glib.euclid(goal_x, goal_y, cur_x, cur_y);
            
            if (distToGoal < CLOSE_ENOUGH_TO_GOAL) {
                action_out.linear = 0;
                action_out.angular = 0;
                return;
            }
            
            var best_weight = -1,
                best_linear = null,
                best_angular = null,
                best_found = false;
            
            for (var angular = ANGULAR_MIN; 
                     angular <= ANGULAR_MAX; 
                     angular += ANGULAR_INC) 
            {
                if (angular < cur_angular - ANGULAR_ACCEL || 
                    angular > cur_angular + ANGULAR_ACCEL) {
                    continue;
                }
                
                for (var linear = LINEAR_MIN; 
                         linear <= LINEAR_MAX; 
                         linear += LINEAR_INC)
                {
                    if (linear < cur_linear - LINEAR_ACCEL || 
                        linear > cur_linear + LINEAR_ACCEL) {
                        continue;
                    } 
                    
                    //
                    // get clearance and normalize
                    //
                    var res = clib.calcIntersection(
                        cur_x, 
                        cur_y, 
                        cur_dir, 
                        linear, 
                        angular, 
                        mycloud
                        );
                        
                    var clearance = CLEARANCE_MAX,
                        color = "gray";
                        
                    if (res.point && res.delta < CLEARANCE_MAX) {
                        clearance = res.delta;
                        color = "red";
                    
                        if (clearance < CLEARANCE_MIN) {
                            continue;
                        }
                    }
                    
                    var clearanceNorm = clearance/CLEARANCE_MAX;
                    
                    //
                    // get normalized goal direction weight
                    //
                    var newDir = cur_dir + angular*DT,
                        goalDir = Math.atan2(goal_y - cur_y, goal_x - cur_x),
                        goalDirDif = Math.PI - Math.abs(glib.angleDif(goalDir, newDir)),
                        goalDirDifNorm = goalDirDif/Math.PI;
                    
                    //
                    // get normalized linear velocity weight
                    //
                    var linearNorm = linear/LINEAR_MAX;
                    
                    //
                    // get the final weight, compare it with what we've seen so far
                    //
                    var weight = clearanceNorm*CLEARANCE_WEIGHT +
                                 goalDirDifNorm*GOAL_DIR_WEIGHT + 
                                 linearNorm*LINEAR_WEIGHT;
                    
                    if (weight > best_weight) {
                        best_weight = weight;
                        best_linear = linear;
                        best_angular = angular;
                        best_found = true;
                    }
                    
                    //
                    // plot trajectory & intersection point, if there was one
                    //
                    if (res.point) {
                        Plotter.plotPoint(res.point.x, res.point.y, .1, color);
                    }
                    
                    if (res.traj.type === "circle") {
                        Plotter.plotCircle(res.traj.x, res.traj.y, res.traj.r, color);
                    } else if (res.traj.type === "vector") {
                        Plotter.plotVector(res.traj.x, res.traj.y, res.traj.dir, color);
                    }
                }
            }
            
            if (!best_found || (Math.abs(best_linear) <= 1e-6 && Math.abs(best_angular) <= 1e-6)) {
                action_out.linear = 0;
                action_out.angular = 0;
                
                return;
            }
            
            if (distToGoal < SLOW_DOWN_RADIUS) {
                best_linear *= (distToGoal/SLOW_DOWN_RADIUS);
            }
            
            action_out.linear = best_linear;
            action_out.angular = best_angular;
        };
    
    document.getElementById("linearVel").value = "" + LINEAR_WEIGHT;
    document.getElementById("clearance").value = "" + CLEARANCE_WEIGHT;
    document.getElementById("goalDir").value = "" + GOAL_DIR_WEIGHT;
    
    var canvas = document.getElementById("canvas"),
        cur_x = 0,
        cur_y = 0,
        cur_dir = 0,
        goal_x = 0, 
        goal_y = 0, 
        cur_lin = 0,
        cur_ang = 0,
        cloud = [];
    
    for (var i = 0; i < NUM_OBSTACLES; i++) {
        cloud.push({
            x: Math.random()*26-13, 
            y: Math.random()*14-7,
            r: SIZE_RADIUS + BUFFER_SPACE,
            dir: Math.random()*2*Math.PI
            });
    }
    
    var plotEverything = function (cloud) {
        Plotter.clear();
        
        Plotter.plotPoint(cur_x, cur_y, CLEARANCE_MAX, "lightBlue");
        Plotter.plotPoint(cur_x, cur_y, SIZE_RADIUS, "lightGreen");
        Plotter.plotCircle(cur_x, cur_y, SIZE_RADIUS, "darkGreen");
        Plotter.drawAxises(1, 1);
        
        for (var i = 0; i < cloud.length; i++) {
            Plotter.plotPoint(cloud[i].x, cloud[i].y, 0.05, "black");
            Plotter.plotCircle(cloud[i].x, cloud[i].y, cloud[i].r, "gray");
        }
        
        Plotter.plotPoint(goal_x, goal_y, 0.1, "blue");
    };
    
    Plotter.init(canvas, .5, .5, 50, 50);
    //plotEverything();
        
    var step = function () {
        plotEverything(cloud);
        
        var action_out = {linear: 0, angular: 0};
        
        var mycloud = [];
        for (var i = 0; i < cloud.length; i++) {
            if (glib.euclid(cloud[i].x, cloud[i].y, cur_x, cur_y) < CLEARANCE_MAX) {
                mycloud.push(cloud[i]);
            }
        }

        DynamicWindow(
            action_out,
            cur_x,
            cur_y,
            cur_dir,
            cur_lin,
            cur_ang,
            mycloud,
            goal_x,
            goal_y
            );  
        
        cur_lin = action_out.linear;
        cur_ang = action_out.angular;

        var kine = glib.calcTrajectoryStepFromTime(
            cur_x, 
            cur_y, 
            cur_dir, 
            cur_lin, 
            cur_ang,
            DT
            );
        
        cur_x = kine[0];
        cur_y = kine[1];
        cur_dir = kine[2];
        
        Plotter.plotArrow(cur_x, cur_y, cur_dir, "red");
        
        setTimeout(step, 20);
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
    
    setTimeout(step, 20);
})();
