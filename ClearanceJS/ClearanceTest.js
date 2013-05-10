var MAX_RADIUS = 50,
    MIN_RADIUS = 10,
    NUM_CIRCLES = 10,
    
    canvas = document.getElementById("canvas");

    
var gi = (function () {
    var lib = {},
    
        context = canvas.getContext("2d"),
        WIDTH = canvas.width,
        HEIGHT = canvas.height,
        LARGE_DISTANCE = Math.sqrt(WIDTH*WIDTH + HEIGHT*HEIGHT),
        POINT_SIZE = 2;
        
    lib.WIDTH = WIDTH;
    lib.HEIGHT = HEIGHT;
    
    lib.drawCircle = function (c, color) {
        context.strokeStyle = color || "blue";
        context.beginPath();
        context.arc(c.x, c.y, c.r, 0, Math.PI*2, false);
        context.stroke();
    };

    lib.drawPoint = function (p, color) {
        context.fillStyle = color || "red";
        context.beginPath();
        context.arc(p.x, p.y, POINT_SIZE, 0, Math.PI*2, false);
        context.fill();
    };

    lib.drawVector = function (v, color) {
        context.strokeStyle = color || "blue";
        context.beginPath();
        context.moveTo(v.x, v.y);
        context.lineTo(v.x + LARGE_DISTANCE*Math.cos(v.dir),
                       v.y + LARGE_DISTANCE*Math.sin(v.dir));
        context.stroke();
    };

    lib.draw = (function () {
        var lookup = {
            "circle": lib.drawCircle,
            "point": lib.drawPoint,
            "vector": lib.drawVector
        };
        
        return function (thing, color) {
            lookup[thing.type](thing, color);
        }
    })();
    
    lib.clear = function () {        
        context.fillStyle = "lightGray";
        context.fillRect(0, 0, gi.WIDTH, gi.HEIGHT);
    };
    
    return lib;

})();
//
// Make background circles
//

var staticCircles = [];

for (var i = 0; i < NUM_CIRCLES; i++) {
    var c = clib.makeCircle(
        Math.random()*gi.WIDTH, 
        Math.random()*gi.HEIGHT, 
        Math.random()*(MAX_RADIUS - MIN_RADIUS) + MIN_RADIUS
    );
    
    staticCircles.push(c);
}

//
// Set up trajectory calculations
//

var globalAngle = 0, 
    globalX = 0, 
    globalY = 0;

canvas.onmousemove = function (event) {
    globalX = event.offsetX,
    globalY = event.offsetY;
};

var developTrajectories = function () {
    var circles = staticCircles.concat([
            clib.makeCircle(globalX, globalY, 50)
            ]),
            
        x = gi.WIDTH/2, 
        y = gi.HEIGHT/2;
    
    gi.clear();
    
    for (var i = 0; i < circles.length; i++) {
        gi.draw(circles[i]);
    }

    for (var angular = -.1; angular <= .1; angular += .02) {
        for (var linear = 1; linear <= 5; linear += 1) {
            var res = clib.calcIntersection(
                x,
                y,
                globalAngle, 
                linear,
                angular,
                circles
                );
            
            gi.draw(res.traj, "black");
            
            if (res.point) {
                var kime = clib.calcKinematicFromArc(
                    x,
                    y, 
                    globalAngle, 
                    linear,
                    angular,
                    res.delta
                    );
                
                gi.draw(clib.makePoint(kime[0], kime[1]));
            }
        }
    }
}

setInterval(function() {
    globalAngle += .01;
    developTrajectories();
}, 10);
    







