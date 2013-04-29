(function () {
    var CLOSE_ENOUGH_TO_GOAL = .1,
        ANGULAR_ACCEL = 1, ANGULAR_SAMPLES = 3, ANGULAR_MIN = -1, ANGULAR_MAX = 1,
        LINEAR_ACCEL = 1, LINEAR_SAMPLES = 3, LINEAR_MIN = 0, LINEAR_MAX = 1,
        DT = 1, 
        CLEARANCE_MIN = .3, CLEARANCE_MAX = 2,
        LINEAR_WEIGHT = .1, CLEARANCE_WEIGHT = 0, GOAL_DIR_WEIGHT = 1,
        
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
            
            return min_clearance;
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
            
            if (euclidDist(goal_x, goal_y, cur_x, cur_y) < CLOSE_ENOUGH_TO_GOAL) {
                spec.action_out.linear = 0;
                spec.action_out.angular = 0;
                return true;
            }
            
            var options = [];
            
            for (var ang_add = cur_angular - ANGULAR_ACCEL; 
                 ang_add <= cur_angular + ANGULAR_ACCEL;
                 ang_add += ANGULAR_ACCEL*2/(ANGULAR_SAMPLES - 1)) 
            {
                var angular = cur_angular + ang_add;
                
                if (angular < ANGULAR_MIN || angular > ANGULAR_MAX) {
                    continue;
                }
                
                for (var lin_add = cur_linear - LINEAR_ACCEL; 
                     lin_add <= cur_linear + LINEAR_ACCEL;
                     lin_add += LINEAR_ACCEL*2/(LINEAR_SAMPLES - 1)) 
                {
                    var linear = cur_linear + lin_add;
                
                    if (linear < LINEAR_MIN || linear > LINEAR_MAX) {
                        continue;
                    } 
                    
                    // calcular new point
                    var dir = cur_dir + DT*angular,
                        x = cur_x + DT*linear*Math.cos(dir),
                        y = cur_y + DT*linear*Math.sin(dir);
                        
                    // console.log("debug", x, y, dir);
                
                    Plotter.drawArrow(x, y, dir);
                           
                    // calculate clearance
                    var clearance = calcClearance(cloud, x, y);
                    
                    // ignore if the point is too close to obstacles
                    if (clearance < CLEARANCE_MIN) {
                        continue;
                    }
                    
                    // calculate normalized clearance weight
                    var clearanceNorm = (clearance - CLEARANCE_MIN)/CLEARANCE_MAX;
                    
                    // calculate normalized goal direction weight
                    var goalDir = Math.atan2(goal_y - cur_y, goal_x - cur_x);
                    var goalDirDif = Math.abs(angleDif(goalDir, dir));
                    var goalDirDifNorm = (Math.PI - goalDirDif)/Math.PI;
                    
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
            
            var best_weight = options[0].weight;
            var best_index = 0;
            
            for (var i = 1; i < options.length; i++) {
                if (options[i].weight > best_weight) {
                    best_weight = options[i].weight;
                    best_index = i;
                }
            }
            
            spec.action_out.linear = options[best_index].linear;
            spec.action_out.angular = options[best_index].angular;
            return true;
        },
        
        testCaseArr = [
            {expected: {linear: LINEAR_ACCEL, angular: 0},
             action_out: {linear: 0, angular: 0},
             cur_x: 0, 
             cur_y: 0,
             cur_dir: Math.PI/2,
             cur_linear: 0,
             cur_angular: 0,
             cloud: [],
             goal_x: 0,
             goal_y: 2,
             LINEAR_WEIGHT: 1, 
             CLEARANCE_WEIGHT: 0, 
             GOAL_DIR_WEIGHT: .1
            }, 
            
            {expected: {linear: LINEAR_ACCEL, angular: ANGULAR_ACCEL},
             action_out: {linear: 0, angular: 0},
             cur_x: 0,
             cur_y: 0,
             cur_dir: Math.PI/2,
             cur_linear: 0,
             cur_angular: 0,
             cloud: [{x: 0, y: 1}],
             goal_x: 0,
             goal_y: 2,
             LINEAR_WEIGHT: 0, 
             CLEARANCE_WEIGHT: 1, 
             GOAL_DIR_WEIGHT: 0
            }, 
            
            {expected: {linear: LINEAR_ACCEL, angular: -Math.PI/4},
             action_out: {linear: 0, angular: 0},
             cur_x: 0,
             cur_y: 0,
             cur_dir: Math.PI/2,
             cur_linear: 0,
             cur_angular: 0,
             cloud: [],
             goal_x: 2,
             goal_y: 2,
             LINEAR_WEIGHT: .1, 
             CLEARANCE_WEIGHT: 0, 
             GOAL_DIR_WEIGHT: 1
            }
        ];
    /*
    for (var i = 0; i < testCaseArr.length; i++) {
        LINEAR_WEIGHT = testCaseArr[i].LINEAR_WEIGHT;
        CLEARANCE_WEIGHT = testCaseArr[i].CLEARANCE_WEIGHT;
        GOAL_DIR_WEIGHT = testCaseArr[i].GOAL_DIR_WEIGHT;
    
        var success = DynamicWindow(testCaseArr[i]),
            res = testCaseArr[i].action_out,
            expected = testCaseArr[i].expected;
        
        if (success) {
            if (res.angular === expected.angular &&
                res.linear === expected.linear) {
                console.log("test", i, "successful");
            } else {
                console.log("test", i, "failed!");
                console.log("  returned:", res.linear, ",", res.angular);
                console.log("  expected: ", expected.linear, ",", expected.angular);
            }
        } else {
            console.log("test", i, "returned failure!");
        }
    }
    */
    
    var canvas = document.getElementById("canvas");
    
    Plotter.init(canvas, .5, .5, 50, 50);
    Plotter.clear();
    Plotter.drawAxises(1, 1);
    
    var goal_x = null, 
        goal_y = null, 
        cloud_x = null,
        cloud_y = null,
        state = 0;

    canvas.onmousedown = function (event) {
        var x = event.offsetX, 
            y = event.offsetY;
        
        var coords = Plotter.getPlotCoords(x, y);
        x = coords[0];
        y = coords[1];
        
        if (state === 0) {
            goal_x = x;
            goal_y = y;
            state = 0;
            
            var spec = {
                action_out: {linear: 0, angular: 0},
                cur_x: 0,
                cur_y: 0,
                cur_dir: Math.PI/2,
                cur_linear: 0,
                cur_angular: 0,
                cloud: [],
                goal_x: goal_x,
                goal_y: goal_y,
            }
            
            Plotter.clear();
            Plotter.drawAxises(1, 1);
            DynamicWindow(spec);  
            Plotter.plotPoint(x, y);
            
            var linear = spec.action_out.linear,
                angular = spec.action_out.angular,
                dir = Math.PI/2 + DT*angular,
                x = DT*linear*Math.cos(dir),
                y = DT*linear*Math.sin(dir);
                
            Plotter.drawRedArrow(x, y, dir);
        }
    };
})();