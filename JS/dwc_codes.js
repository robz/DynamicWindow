
function sign(x) { return x > 0 ? 1 : x < 0 ? -1 : 0; }
function min(a, b) { return (a < b) ? a : b; }
function max(a, b) { return (a > b) ? a : b; }

//A movement arc is a v, w, center point, and radius. You make them like: 
/*
    MovementArc(double newV, double newW)
        {
            v = newV;
            w = newW;

            if(newW != 0.0)
            {
                curve.radius = fabs(newV / newW);
                curve.center = Point(0, newV / newW, 0.0); //w.r.t. robot
            }
            else //is actually a straight line
            {
                curve.radius = -1.0;
                curve.center = Point(1.0, 0.0, 0.0);
            }
        }
 */
 
var makePoint = function (x, y, x) {
    x = x || 0;
    y = y || 0;
    z = z || 0;

    var that = {x:x, y:y, z:z};
    
    that.sub = function (point) {
        return makePoint(that.x - point.x, that.y - point.y, that.z - point.z);
    };
    
    that.remake = function () {
        that.x = that.y = that.z = 0;
    };
    
    that.copy = function (point) {
        that.x = point.x;
        that.y = point.y;
        that.z = point.z;
    };
    
    return that;
}
 
var makeMovementArc = function (newV, newW) {
    var that = {};
    that.v = newV;
    that.w = newW;
    
    that.curve = {};
    if (newW != 0.0) {
        that.curve.radius = fabs(newV / newW);
        that.curve.center = makePoint(0, newV / newW, 0.0); //w.r.t. robot
    } else {
        that.curve.radius = -1.0;
        that.curve.center = makePoint(1.0, 0.0, 0.0);
    }
    
    return that;
}

var makeFootprint = function (center, dir, radius) {
    return [
        center,
        makePoint(center.x + radius*Math.cos(dir), center.y + radius*Math.sin(dir))
    ];
};

var distance2D = function (p1, p2) {
    return Math.sqrt((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y));
}

//returns boolean whether or not it hit something
//Points are just float x,y,z (where z is always 0)
var getClearLinearDistance = function (laserPoint, footprint, intersectionPoint_out, drivableDistance_out)
{
    var minY = 1000000.0;
    var maxY = 0.0;
    var intersectionFound = false;
    
    if(laserPoint.x < 0) //laser can't be on our path
    {
        intersectionPoint_out.remake();
        return false;
    }
    
    var radius = distance2D(footprint[0], footprint[1]); //we made circle footprints with footprint[0] being the center point, and footprint[1] being a point on the circle at the front of the robot
    
    if((radius * radius) < (laserPoint.y * laserPoint.y))
    {
        return false;
    }
    else
    {
        var xoff = Math.sqrt(radius * radius - laserPoint.y * laserPoint.y);
        var intersection1 = makePoint(laserPoint.x + xoff, 0, 0);
        var distance1 = distance2D(intersection1, makePoint());
        var intersection2 = makePoint(laserPoint.x - xoff, 0, 0);
        var distance2 = distance2D(intersection2, makePoint());
        
        if(distance1 < distance2)
        {
            intersectionPoint_out.copy(intersection1);
            drivableDistance_out.val = distance1;
        }
        else
        {
            intersectionPoint.copy(intersection2);
            drivableDistance_out.val = distance2;
        }
        return true;
    }
}

var innerAngle = function (firstPoint, vertex, secondPoint)
{
    var v1 = secondPoint.sub(vertex); //2e-6,0.4
    var v2 = firstPoint.sub(vertex); //-3e-6,0.0

    var length1 = distance2D(makePoint(0, 0, 0), v1) + 0.00001; //addition to fix float rounding to zero
    var length2 = distance2D(makePoint(0, 0, 0), v2) + 0.00001;

    return Math.acos(((v1.x * v2.x + v1.y * v2.y)) / (length1 * length2));
}

var angleInDirection = function (angleStart, vertex, angleEnd, direction)
{
    var smallAngle = innerAngle(angleStart, vertex, angleEnd);
    if((direction == CCW && !isLeftOfLine(vertex, angleStart, angleEnd)) || (direction == CW && isLeftOfLine(vertex, angleStart, angleEnd)))
    {
        //angle is inverted
        return 2 * Math.PI - smallAngle;
    }
    return smallAngle;
}

var getTurnableAngleCircular = function (laserPoint, turnCenter, robotRadius, direction)
{
    var turnRadius = distance2D(turnCenter, makePoint());
    
    var dx = laserPoint.x - turnCenter.x;
    var dy = laserPoint.y - turnCenter.y;
    var centerSeparation = Math.sqrt(dx * dx + dy * dy);
    
    if(centerSeparation > (turnRadius + robotRadius)) //circles are too far apart, no overlap
    {
        return 2 * Math.PI;
    }
    
    if(centerSeparation < fabs(turnRadius - robotRadius)) // one circle is inside the other, no overlap
    {
        return 2 * Math.PI;
    }
    
    var intersectionCenterDistance = ((turnRadius * turnRadius) - (robotRadius * robotRadius) + (centerSeparation * centerSeparation)) / (2.0 * centerSeparation);
    var intersectionCenter = makePoint(turnCenter.x + (dx * intersectionCenterDistance / centerSeparation), turnCenter.y + (dy * intersectionCenterDistance / centerSeparation), 0);
    
    var intersectionPointToCenterDistance = Math.sqrt((turnRadius * turnRadius) - (intersectionCenterDistance * intersectionCenterDistance));
    var xOff = -dy * (intersectionPointToCenterDistance / centerSeparation);
    var yOff = dx * (intersectionPointToCenterDistance / centerSeparation);
    
    var intersection1 = makePoint(intersectionCenter.x + xOff, intersectionCenter.y + yOff, 0);
    var intersection2 = makePoint(intersectionCenter.x - xOff, intersectionCenter.y - yOff, 0);
    
    var turnableAngle1 = angleInDirection(intersection1, turnCenter, laserPoint, direction);
    var turnableAngle2 = angleInDirection(intersection2, turnCenter, laserPoint, direction);
    
    return min(turnableAngle1, turnableAngle2);
}


var getArcableAngle = function (laserPoint, turnCenter, footprint, arcForward)
{
    var direction = sign(turnCenter.y) * (arcForward? 1 : -1);
    
    return getTurnableAngleCircular(laserPoint, turnCenter, distance2D(footprint[0], footprint[1]), direction);
}

var lineIntersection = function (p1, p2, p3, p4, intersection_out)
{
    // Store the values for fast access and easy
    // equations-to-code conversion
    var x1 = p1.x, x2 = p2.x, x3 = p3.x, x4 = p4.x;
    var y1 = p1.y, y2 = p2.y, y3 = p3.y, y4 = p4.y;

    var d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    // If d is zero, there is no intersection
    if(d == 0)
    {
        return false;
    }

    // Get the x and y
    var pre = (x1 * y2 - y1 * x2), post = (x3 * y4 - y3 * x4);
    var x = (pre * (x3 - x4) - (x1 - x2) * post) / d;
    var y = (pre * (y3 - y4) - (y1 - y2) * post) / d;

    // Check if the x and y coordinates are within both lines
    //random additions to fix retarded float rounding
    if(x+0.000001 < min(x1, x2) || x-0.000001 > max(x1, x2) || x+0.000001 < min(x3, x4) || x-0.000001 > max(x3, x4))
    {
        return false;
    }
    if(y+0.000001 < min(y1, y2) || y-0.000001 > max(y1, y2) || y+0.000001 < min(y3, y4) || y-0.000001 > max(y3, y4))
    {
        return false;
    }

    // Return the point of intersection
    intersection_out.x = x;
    intersection_out.y = y;
    return true;
}

//MovementArc arc, Point obstruction, int targetDirection
// returns double
function availableDistance(arc, obstruction, targetDirection, footprint, slowDownDistance)
{
    if(arc.w == 0.0)
    {
        var obstaclePoint = makePoint();
        var distancep = {val:0};
        if(getClearLinearDistance(obstruction, footprint, obstaclePoint, distancep))
        {
            return distancep.val;
        }
        else
        {
            return slowDownDistance;
        }
    }
    else
    {
        var turnableAngle = getArcableAngle(obstruction, arc.curve.center, footprint);
        var distance = turnableAngle * arc.curve.radius;
        if(epsilon(turnableAngle, 2 * Math.PI) || distance > slowDownDistance)
        {
            return slowDownDistance;
        }
        else
        {
            return distance;
        }
    }
}