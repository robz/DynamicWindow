var Plotter = (function (spec) {
    var plotter = {},
    
        POINT_COLOR = spec.POINT_COLOR,
        POINT_RADIUS = spec.POINT_RADIUS,
        BG_COLOR = spec.BG_COLOR,
        AXIS_COLOR = spec.AXIS_COLOR,
        ARROW_COLOR = spec.ARROW_COLOR,
        ARROW_LENGTH = spec.ARROW_LENGTH,
        
        context = null,
        width = null,
        height = null,
        originx = null,
        originy = null,
        scalex = null,
        scaley = null;
    
    // expecting originx and originy to be between 0 and 1 
    // where (0,0) would be the bottom-left of the canvas, and 
    // (1,1) would be the top-right.
    plotter.init = function (canvas, _originx, _originy, _scalex, _scaley) {
        context = canvas.getContext("2d");
        width = canvas.width;
        height = canvas.height;
        originx = _originx;
        originy = _originy;
        scalex = _scalex;
        scaley = _scaley;
    };
    
    plotter.getPlotCoords = function (x, y) {
        return [
            (x - originx*width)/scalex,
            (height - y - originy*height)/scaley
        ];
    };
    
    // where incx and incy are where the tick marks appear on the axis
    plotter.drawAxises = function (ratiox, ratioy) {
        context.strokeStyle = AXIS_COLOR;
        context.lineWidth = 3;
        
        context.beginPath();
        context.moveTo(0, height - originy*height);
        context.lineTo(width, height - originy*height);
        context.stroke();
        
        context.beginPath();
        context.moveTo(originx*width, 0);
        context.lineTo(originx*width, height);
        context.stroke();
        
        for (var x = originx*width; x <= width; x += ratiox*scalex) {
            context.beginPath();
            context.moveTo(x, height - originy*height);
            context.lineTo(x, height - originy*height - 5);
            context.stroke();
        }
        
        for (var x = originx*width; x >= 0; x -= ratiox*scalex) {
            context.beginPath();
            context.moveTo(x, height - originy*height);
            context.lineTo(x, height - originy*height - 5);
            context.stroke();
        }
        
        for (var y = originy*height; y <= height; y += ratioy*scaley) {
            context.beginPath();
            context.moveTo(originx*width, height - y);
            context.lineTo(originx*width + 5, height - y);
            context.stroke();
        }
        
        for (var y = originy*height; y >= 0; y -= ratioy*scaley) {
            context.beginPath();
            context.moveTo(originx*width, height - y);
            context.lineTo(originx*width + 5, height - y);
            context.stroke();
        }
    };
    
    plotter.plotPoint = function (x, y, color, radius) {
        if (typeof color === "undefined") {
            color = POINT_COLOR;
        }
        
        if (typeof radius === "undefined") {
            radius = POINT_RADIUS;
        } else {
            radius *= scalex;
        }
    
        context.save();
        context.fillStyle = color;
        context.beginPath();
        context.arc(
            x*scalex + originx*width, 
            height - (y*scaley + originy*height), 
            radius, 0, Math.PI*2, false);
        context.fill();
        context.restore();
    };
    
    plotter.plotMousePoint = function (x, y) {
        context.save();
        context.fillStyle = POINT_COLOR;
        context.beginPath();
        context.arc(x, y, POINT_RADIUS, 0, Math.PI*2, false);
        context.fill();
        context.restore();
    };
    
    moveTo = function (x, y) {
        x = x*scalex + originx*width;
        y = height - (y*scaley + originy*height);
        
        context.moveTo(x, y);
    }
    
    lineTo = function (x, y) {
        x = x*scalex + originx*width;
        y = height - (y*scaley + originy*height);
        
        context.lineTo(x, y);
    }
    
    arc = function (x, y, radius) {
        x = x*scalex + originx*width;
        y = height - (y*scaley + originy*height);
        
        context.arc(x, y, radius, 0, Math.PI*2, false);
    }
    
    plotter.drawArrow = function (x, y, dir) {
        context.save();
        
        context.strokeStyle = ARROW_COLOR;
        context.lineWidth = 2;
        context.beginPath();
        
        moveTo(x, y);
        lineTo(x + ARROW_LENGTH*Math.cos(dir), 
                       y + ARROW_LENGTH*Math.sin(dir));
        lineTo(x + ARROW_LENGTH*Math.cos(dir) + ARROW_LENGTH/2*Math.cos(dir + 4*Math.PI/5), 
                       y + ARROW_LENGTH*Math.sin(dir) + ARROW_LENGTH/2*Math.sin(dir + 4*Math.PI/5));
        lineTo(x + ARROW_LENGTH*Math.cos(dir), 
                       y + ARROW_LENGTH*Math.sin(dir));
        lineTo(x + ARROW_LENGTH*Math.cos(dir) + ARROW_LENGTH/2*Math.cos(dir - 4*Math.PI/5), 
                       y + ARROW_LENGTH*Math.sin(dir) + ARROW_LENGTH/2*Math.sin(dir - 4*Math.PI/5));
        context.stroke();
        
        context.restore();
    };
    
    plotter.drawRedArrow = function (x, y, dir) {
        var tmp = ARROW_COLOR;
        ARROW_COLOR = "red";
        plotter.drawArrow(x, y, dir);
        ARROW_COLOR = tmp;
    };
    
    plotter.clear = function () {
        context.save();
        context.fillStyle = BG_COLOR;
        context.fillRect(0, 0, width, height);
        context.restore();
    };
    
    return plotter;
})({
    POINT_COLOR: "black",
    POINT_RADIUS: 5,
    BG_COLOR: "lightGray",
    AXIS_COLOR: "gray",
    ARROW_COLOR: "darkGray",
    ARROW_LENGTH: .3,
});