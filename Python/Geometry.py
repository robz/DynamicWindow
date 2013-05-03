import math

class Circle:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def euclid(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def intersections(c1, c2):
    dist = euclid(c1.x, c1.y, c2.x, c2.y)
    angle = math.atan2(c2.y - c1.y, c2.x - c1.x)
    
    small = min(c1.r, c2.r)
    big = max(c1.r, c2.r)
    
    if dist == small + big or dist == big - small:
        return [Point(c1.x + c1.r*math.cos(angle), 
                      c1.y + c1.r*math.sin(angle))]
    elif dist < small + big and dist > big - small:
        # law of cosines
        angle_offset = math.acos((c1.r**2 + dist**2 - c2.r**2)/(2*c1.r*dist))
        return [Point(c1.x + c1.r*math.cos(angle + angle_offset), 
                      c1.y + c1.r*math.sin(angle + angle_offset)), 
                Point(c1.x + c1.r*math.cos(angle - angle_offset), 
                      c1.y + c1.r*math.sin(angle - angle_offset))]	
    
    return []