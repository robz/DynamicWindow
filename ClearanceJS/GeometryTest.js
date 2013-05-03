var canvas = document.getElementById("canvas"),
    context = canvas.getContext("2d"),
    WIDTH = canvas.width,
    HEIGHT = canvas.height,
    MAX_RADIUS = 100,
    MIN_RADIUS = 10,
    NUM_CIRCLES = 100,
    POINT_SIZE = 2;

context.fillStyle = "lightGray";
context.fillRect(0, 0, WIDTH, HEIGHT);

var drawCircle = function (c) {
    context.strokeStyle = "blue";
    context.beginPath();
    context.arc(c.x, c.y, c.r, 0, Math.PI*2, false);
    context.stroke();
}

var drawPoint = function (p) {
    context.fillStyle = "red";
    context.beginPath();
    context.arc(p.x, p.y, POINT_SIZE, 0, Math.PI*2, false);
    context.fill();
}

var circles = []

for (var i = 0; i < NUM_CIRCLES; i++) {
    var c = makeCircle(
        Math.random()*WIDTH, 
        Math.random()*HEIGHT, 
        Math.random()*(MAX_RADIUS - MIN_RADIUS) + MIN_RADIUS
    ); 
    
    drawCircle(c);
    circles.push(c);
}

for (var i = 0; i < NUM_CIRCLES; i++) {
    for (var j = 0; j < NUM_CIRCLES; j++) {
        if (i === j) {
            continue;
        }
            
        var intersections = circleIntersections(circles[i], circles[j]);  
        
        for (var k = 0; k < intersections.length; k++) {
            drawPoint(intersections[k]);
        }  
    }
}









