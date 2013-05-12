var Plotter = (function () {
    var plotter = {},
    
        //
        // constants
        //
        VECTOR_COLOR = "black",
        BG_COLOR = "lightGray",
        AXIS_COLOR = "gray",
        ARROW_COLOR = "darkGray",
        ARROW_LENGTH = 0.3,
        
        //
        // variables to be set during initialization
        //
        context = null,
        width = null,
        height = null,
        originx = null,
        originy = null,
        scalex = null,
        scaley = null,
    
        //
        // functions to make converting plot to canvas coords easier
        //
        plotToCanvasX = function (x) {
            return x*scalex + originx*width;
        },
        
        plotToCanvasY = function (y) {
            return height - (y*scaley + originy*height);
        },
        
        moveTo = function (x, y) {
            x = plotToCanvasX(x);
            y = plotToCanvasY(y);
            
            context.moveTo(x, y);
        },
        
        lineTo = function (x, y) {
            x = plotToCanvasX(x);
            y = plotToCanvasY(y);
            
            context.lineTo(x, y);
        },
    
        arc = function (x, y, radius) {
            x = plotToCanvasX(x);
            y = plotToCanvasY(y);
            
            context.arc(x, y, radius, 0, Math.PI*2, false);
        };
    
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
    
    // where ratioxy = 1 will show a tick on the axis every 1 unit;
    //               = 2 will shows every 2 units, etc
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
    
    plotter.plotPoint = function (x, y, radius, color) {
        radius *= scalex;
    
        context.save();
        
        context.fillStyle = color;
        
        context.beginPath();
        arc(x, y, radius);
        context.fill();
        
        context.restore();
    };
    
    plotter.plotCircle = function (x, y, radius, color) {
        radius *= scalex;
    
        context.save();
        
        context.strokeStyle = color;
        context.lineWidth = 1;
        
        context.beginPath();
        arc(x, y, radius);
        context.stroke();
        
        context.restore();
    };
    
    plotter.plotVector = function (x, y, dir, color) {
        if (typeof color === "undefined") {
            color = VECTOR_COLOR;
        }
        
        context.save();
        
        context.strokeStyle = color;
        context.lineWidth = 1;
        
        context.beginPath();
        moveTo(x, y);
        lineTo(x + 1e6*Math.cos(dir), y + 1e6*Math.sin(dir));
        context.stroke();
        
        context.restore();
    };
    
    plotter.plotArrow = function (x, y, dir, color) {
        if (typeof color === "undefined") {
            color = ARROW_COLOR;
        }
        
        var endx = x + ARROW_LENGTH*Math.cos(dir),
            endy = y + ARROW_LENGTH*Math.sin(dir);
    
        context.save();
        
        context.strokeStyle = color;
        context.lineWidth = 2;
        
        context.beginPath();
        moveTo(x, y);
        lineTo(endx, endy);
        lineTo(endx + ARROW_LENGTH/2*Math.cos(dir + 4*Math.PI/5), 
               endy + ARROW_LENGTH/2*Math.sin(dir + 4*Math.PI/5));
        lineTo(endx, endy);
        lineTo(endx + ARROW_LENGTH/2*Math.cos(dir - 4*Math.PI/5), 
               endy + ARROW_LENGTH/2*Math.sin(dir - 4*Math.PI/5));
        context.stroke();
        
        context.restore();
    };
    
    plotter.clear = function () {
        context.save();
        
        context.fillStyle = BG_COLOR;
        context.fillRect(0, 0, width, height);
        
        context.restore();
    };
    
    return plotter;
})();
