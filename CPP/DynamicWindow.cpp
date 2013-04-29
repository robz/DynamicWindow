#include <vector>

struct Action {
    double linear, angular;
} typedef Action;

struct Point {
    double x, y;
} typedef Point;

class ActionOption {
    public:
        double weight, linear, angular;
        ActionOption();
        ActionOption(double, double, double);
};

ActionOption::ActionOption() {
    weight = 0;
    linear = 0;
    angular = 0;
 }
 
ActionOption::ActionOption(
    double _weight, 
    double _linear, 
    double _angular) {
    weight = _weight;
    linear = _linear;
    angular = _angular;
 }

double CLOSE_ENOUGH_TO_GOAL = .1,
       ANGULAR_ACCEL = 2, ANGULAR_SAMPLES = 100, ANGULAR_MIN = -1, ANGULAR_MAX = 1,
       LINEAR_ACCEL = .5, LINEAR_SAMPLES = 5, LINEAR_MIN = 0, LINEAR_MAX = 1,
       DT = 1, 
       CLEARANCE_MIN = .3, CLEARANCE_MAX = 2,
       LINEAR_WEIGHT = .1, CLEARANCE_WEIGHT = 0, GOAL_DIR_WEIGHT = 1;

double euclidDist(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

double boundAngle0to2Pi(double angle) {
    return angle - floor(angle/(2*M_PI))*2*M_PI;
}

double angleDif(double ang1, double ang2) {
    double res = boundAngle0to2Pi(ang1) - boundAngle0to2Pi(ang2);
    
    if (res > M_PI) {
        res -= M_PI*2;
    } else if (res < -M_PI) {
        res += M_PI*2;
    }
    
    return res;
}

double calcClearance(Point* cloud, int cloudLen, double x, double y) {
    if (cloudLen == 0) {
        return CLEARANCE_MAX;
    }
    
    double min_clearance = euclidDist(
        cloud[0].x, 
        cloud[0].y, 
        x, 
        y
        );
    
    for (int i = 1; i < cloudLen; i++) {
        double clearance = euclidDist(
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
}

// returns true on success
bool DynamicWindow(
    Action* action_out,
    double cur_x, 
    double cur_y,
    double cur_dir,
    double cur_linear,
    double cur_angular,
    Point* cloud,
    int cloudLen,
    double goal_x, 
    double goal_y
    ) {
    
    if (!action_out) {
        return false;
    }
    
    if (euclidDist(goal_x, goal_y, cur_x, cur_y) < CLOSE_ENOUGH_TO_GOAL) {
        (*action_out).linear = 0;
        (*action_out).angular = 0;
        return true;
    }
    
    vector<ActionOption> options(0);
    
    for (double ang_add = cur_angular - ANGULAR_ACCEL; 
         ang_add <= cur_angular + ANGULAR_ACCEL;
         ang_add += ANGULAR_ACCEL*2/(ANGULAR_SAMPLES - 1)) 
    {
        double angular = cur_angular + ang_add;
        
        if (angular < ANGULAR_MIN || angular > ANGULAR_MAX) {
            continue;
        }
        
        for (double lin_add = cur_linear - LINEAR_ACCEL; 
             lin_add <= cur_linear + LINEAR_ACCEL;
             lin_add += LINEAR_ACCEL*2/(LINEAR_SAMPLES - 1)) 
        {
            double linear = cur_linear + lin_add;
        
            if (linear < LINEAR_MIN || linear > LINEAR_MAX) {
                continue;
            } 
            
            // calcular new point
            double dir = cur_dir + DT*angular,
                   x = cur_x + DT*linear*cos(cur_dir + dir),
                   y = cur_y + DT*linear*sin(cur_dir + dir);
                   
            // calculate clearance
            double clearance = calcClearance(cloud, cloudLen, x, y);
            
            // ignore if the point is too close to obstacles
            if (clearance < CLEARANCE_MIN) {
                continue;
            }
            
            // calculate normalized clearance weight
            double clearanceNorm = (clearance - CLEARANCE_MIN)/CLEARANCE_MAX;
            
            // calculate normalized goal direction weight
            double goalDir = atan2(goal_y - cur_y, goal_x - cur_x);
            double goalDirDif = abs(angleDif(goalDir, dir));
            double goalDirDifNorm = (M_PI - goalDirDif)/M_PI;
            // cout << dir << "," << goalDir << "," << goalDirDif << "," << goalDirDifNorm << endl;
            
            // calculate normalized linear velocity weight
            double linearNorm = linear/LINEAR_MAX;
            
            // put it all together
            double weight = clearanceNorm*CLEARANCE_WEIGHT +
                            goalDirDifNorm*GOAL_DIR_WEIGHT + 
                            linearNorm*LINEAR_WEIGHT;
            
            // cout << "weight: " << weight << "," << linear << "," << angular << endl;
            options.push_back(ActionOption(weight, linear, angular));
        }
    }
    
    if (options.size() == 0) {
        (*action_out).linear = 0;
        (*action_out).angular = 0;
        return false;
    }
    
    double best_weight = options.at(0).weight;
    int best_index = 0;
    
    for (unsigned int i = 1; i < options.size(); i++) {
        if (options.at(i).weight > best_weight) {
            best_weight = options.at(i).weight;
            best_index = i;
        }
    }
    
    (*action_out).linear = options.at(best_index).linear;
    (*action_out).angular = options.at(best_index).angular;
    return true;
}


struct TestCase {
    Action res;
    double cur_x; 
    double cur_y;
    double cur_dir;
    double cur_linear;
    double cur_angular;
    Point* cloud;
    int cloudLen;
    double goal_x; 
    double goal_y;
    double LINEAR_WEIGHT, CLEARANCE_WEIGHT, GOAL_DIR_WEIGHT;
} typedef TestCase;

Point cloud0[] = {{0, 0}};

Point cloud1[] = {{0, 1}};

Point cloud2[] = {
    {1, 0}, 
    {sqrt(2.0)/2.0, sqrt(2.0)/2.0}, 
    {0, 1}, 
    {-sqrt(2.0)/2.0, sqrt(2.0)/2.0}, 
    {-1, 0}
    };

TestCase testCaseArr[] = {
    {{LINEAR_ACCEL, 0},   // expected action (linear, angular)
     0,        // cur_x
     0,        // cur_y
     M_PI/2.0, // cur_dir
     0,        // cur_linear
     0,        // cur_angular
     cloud0,   // cloud
     0,//sizeof(cloud1)/sizeof(Point), // cloudLen 
     0,        // goal_x
     2,        // goal_y
     1, 0, .1
    }, 
     
    {{LINEAR_ACCEL, ANGULAR_ACCEL},   // expected action (linear, angular)
     0,        // cur_x
     0,        // cur_y
     M_PI/2.0, // cur_dir
     0,        // cur_linear
     0,        // cur_angular
     cloud1,   // cloud
     1,//sizeof(cloud1)/sizeof(Point), // cloudLen 
     0,        // goal_x
     2,         // goal_y
     0, 1, 0 
    },
    
    {{LINEAR_ACCEL, ANGULAR_ACCEL},   // expected action (linear, angular)
     0,        // cur_x
     0,        // cur_y
     M_PI/2.0, // cur_dir
     0,        // cur_linear
     0,        // cur_angular
     cloud0,   // cloud
     0,//sizeof(cloud1)/sizeof(Point), // cloudLen 
     2,        // goal_x
     2,         // goal_y
     .1, 0, 1 
    }
};
    
void testcases() {
    for (unsigned int i = 0; i < sizeof(testCaseArr)/sizeof(TestCase); i++) {
        Action res = {0, 0};
    
        LINEAR_WEIGHT = testCaseArr[i].LINEAR_WEIGHT;
        CLEARANCE_WEIGHT = testCaseArr[i].CLEARANCE_WEIGHT;
        GOAL_DIR_WEIGHT = testCaseArr[i].GOAL_DIR_WEIGHT;
    
        bool success = DynamicWindow(
            &res,
            testCaseArr[i].cur_x,
            testCaseArr[i].cur_y,
            testCaseArr[i].cur_dir,
            testCaseArr[i].cur_linear,
            testCaseArr[i].cur_angular,
            testCaseArr[i].cloud,
            testCaseArr[i].cloudLen,
            testCaseArr[i].goal_x,
            testCaseArr[i].goal_y
            );
        
        if (success) {
            if (res.angular == testCaseArr[i].res.angular &&
                res.linear == testCaseArr[i].res.linear) {
                cout << "test " << i << " successful" << endl;
            } else {
                cout << "test " << i << " failed!" << endl;
                cout << "  returned: " << res.linear << "," << res.angular << endl;
                cout << "  expected: " << testCaseArr[i].res.linear << "," << testCaseArr[i].res.angular << endl;
            }
        } else {
            cout << "test " << i << " returned failure" << endl;
        }
    }
}


int main(void) {
    cout << "hi!" << endl;
    testcases();
    
    //cout << angleDif(.1, -.1) << endl;
    
    return 0;
}