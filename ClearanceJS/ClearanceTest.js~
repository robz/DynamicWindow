var canvas = document.getElementById("canvas"),
    context = canvas.getContext("2d"),
    WIDTH = canvas.width,
    HEIGHT = canvas.height,
    LARGE_DISTANCE = Math.sqrt(WIDTH*WIDTH + HEIGHT*HEIGHT),
    MAX_RADIUS = 50,
    MIN_RADIUS = 10,
    NUM_CIRCLES = 10,
    POINT_SIZE = 2;

context.fillStyle = "lightGray";
context.fillRect(0, 0, WIDTH, HEIGHT);

var drawCircle = function (c, color) {
    context.strokeStyle = color || "blue";
    context.beginPath();
    context.arc(c.x, c.y, c.r, 0, Math.PI*2, false);
    context.stroke();
}

var drawPoint = function (p, color) {
    context.fillStyle = color || "red";
    context.beginPath();
    context.arc(p.x, p.y, POINT_SIZE, 0, Math.PI*2, false);
    context.fill();
}

var drawVector = function (v, color) {
    context.strokeStyle = color || "blue";
    context.beginPath();
    context.moveTo(v.x, v.y);
    context.lineTo(v.x + LARGE_DISTANCE*Math.cos(v.dir),
                   v.y + LARGE_DISTANCE*Math.sin(v.dir));
    context.stroke();
};

var draw = (function () {
    var lookup = {
        "circle": drawCircle,
        "point": drawPoint,
        "vector": drawVector
    };
    
    return function (thing, color) {
        lookup[thing.type](thing, color);
    }
})();

var circles = [];

for (var i = 0; i < NUM_CIRCLES; i++) {
    var c = makeCircle(
        Math.random()*WIDTH, 
        Math.random()*HEIGHT, 
        Math.random()*(MAX_RADIUS - MIN_RADIUS) + MIN_RADIUS
    );
    
    draw(c);
    
    circles.push(c);
}

for (var w = -.1; w <= .1; w += .01) {
    var traj = calculateTrajetory(makePose(
        WIDTH/2, 
        HEIGHT/2, 
        Math.PI*3/2, 
        1, 
        w
    ));
    
    draw(traj, "black");
    
    
}






