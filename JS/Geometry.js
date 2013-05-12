var glib = (function () {
    var lib = {};
    
    lib.euclid = function (x1, y1, x2, y2) {
        return Math.sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
    };
        
    lib.boundAngle0to2Pi = function (angle) {
        return angle - Math.floor(angle/(2*Math.PI))*2*Math.PI;
    };
        
    lib.angleDif = function (ang1, ang2) {
        var res = lib.boundAngle0to2Pi(ang1) - lib.boundAngle0to2Pi(ang2);
        
        if (res > Math.PI) {
            res -= Math.PI*2;
        } else if (res < -Math.PI) {
            res += Math.PI*2;
        }
        
        return res;
    };
    
    var calcTrajectoryStep = function (
        cur_x, 
        cur_y, 
        cur_dir, 
        linear, 
        angular, 
        arclen, 
        new_dir
        )
    {
        var x, y;
    
        if (Math.abs(angular) < 1e-2) {
            x = cur_x + arclen*Math.cos(cur_dir);
            y = cur_y + arclen*Math.sin(cur_dir);
        } else if (angular < 0) {
            var beta = cur_dir - new_dir,
                R = arclen/beta;

            x = cur_x + R*Math.cos(cur_dir - Math.PI/2) 
                      + R*Math.cos(cur_dir + Math.PI/2 - beta);
            y = cur_y + R*Math.sin(cur_dir - Math.PI/2) 
                      + R*Math.sin(cur_dir + Math.PI/2 - beta);
        } else if (angular > 0) {
            var beta = new_dir - cur_dir,
                R = arclen/beta;

            x = cur_x + R*Math.cos(cur_dir + Math.PI/2) 
                      + R*Math.cos(cur_dir - Math.PI/2 + beta);
            y = cur_y + R*Math.sin(cur_dir + Math.PI/2) 
                      + R*Math.sin(cur_dir - Math.PI/2 + beta);
        } 
        
        return [x, y, new_dir];
    };
    
    lib.calcTrajectoryStepFromArc = function (
        cur_x, 
        cur_y, 
        cur_dir, 
        linear, 
        angular, 
        arclen
        )
    {
        if (Math.abs(linear) <= 1e-6) {
            throw "exception: linear velocity cannot be zero here!";   
        }
        
        var new_dir = cur_dir + (arclen/linear)*angular;
        
        return calcTrajectoryStep(cur_x, cur_y, cur_dir, linear, angular, arclen, new_dir);
    };

    lib.calcTrajectoryStepFromTime = function (
        cur_x, 
        cur_y, 
        cur_dir, 
        linear, 
        angular, 
        dt
        )
    {
        var arclen = dt*linear,
            new_dir = cur_dir + dt*angular;
        
        return calcTrajectoryStep(cur_x, cur_y, cur_dir, linear, angular, arclen, new_dir);
    };
    
    return lib;
})();
