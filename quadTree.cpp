#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace std;

//basics of a quad tree
/* Each node represents a neighborhood/area
 * 4 children: NE, NW, SE, SW or is a leaf node
 * these are smaller parts of the og neighborhood
*/

struct Point {
    int x, y;
};

struct Boundary {
    int x, y;
    int xMid, yMid;
    double width = xMid*2;
    double height = yMid*2;

    // checks if the point is inside the area/tree/map
    bool contains(Point p){
        return (p.x >= x - xMid && p.x < x + xMid && p.y >= y - yMid && p.y < y + yMid);
    }

    // helping with the function for closest
    double distanceTo(const Point& p) {
        double dx = max(max(static_cast<double>(x - xMid - p.x), 0.0), static_cast<double>(p.x - (x + xMid)) );
        double dy = max(max(static_cast<double>(y - yMid - p.y), 0.0), static_cast<double>(p.y - (y + yMid)) );

        return hypot(dx, dy);
    }

};

class QuadTree {
private:
    //Boundary of the node
    static const int capacity = 4;

    //og area
    Boundary boundary;
    vector<Point> points;
    bool divided = false;

    // children & subsections
    QuadTree* NE = nullptr;
    QuadTree* NW = nullptr;
    QuadTree* SE = nullptr;
    QuadTree* SW = nullptr;


public:
    QuadTree(Boundary boundary) : boundary(boundary) {}

    // create new children/neighborhoods in tree/area/map
    void subdivide(){
        int x = boundary.x;
        int y = boundary.y;
        int width = boundary.xMid / 2;
        int height = boundary.yMid / 2;

        NW = new QuadTree({x - width, y - height, width, height});
        NE = new QuadTree({x + width, y - height, width, height});
        SW = new QuadTree({x - width, y + height, width, height});
        SE = new QuadTree({x + width, y + height, width, height});

        divided = true;
    }

    // returns true if inserted false if else
    bool insert(Point p){
        if (!boundary.contains(p)){
            return false;
        }

        if(points.size() < capacity){
            points.push_back(p);
            return true;
        }

        if (!divided){
            subdivide();
        }

        return(NE->insert(p) || NW->insert(p) || SE->insert(p) || SW->insert(p));
    }

    // finds closest response for QT
    Point closest(Point target, double& bestDist, Point bestPoint){
        // go through each point in QT
        for (const auto& p : points){
            // find the closest distance between two points using pythagorean theorem
            double dist = hypot(p.x - target.x, p.y - target.y);
            if (dist < bestDist){       // update bestDistance if there is a smaller distance
                bestDist = dist;
                bestPoint = p;          // update best point as closest match
            }

            // if there are children nodes, check each direction/child
            if (divided){

                //create a vector that holds the direction & its closest distance
                vector<pair<QuadTree*, double>> directions = {
                        {NW, NW->boundary.distanceTo(target)},
                        {NE, NE->boundary.distanceTo(target)},
                        {SW, SW->boundary.distanceTo(target)},
                        {SE, SE->boundary.distanceTo(target)},

                };

                // sort each direction by distance
                sort(directions.begin(), directions.end(), [](pair<QuadTree*, double>& a, pair<QuadTree*, double>& b){return a.second < b.second;});

                // iterate through each direction, if smaller go find closest
                for (auto i=0; i < directions.size(); i++){
                    QuadTree* child = directions[i].first;
                    double distance = directions[i].second;

                    if (distance < bestDist){
                        bestPoint = child->closest(target, bestDist, bestPoint);
                    }
                }

            }

            return bestPoint;
        }
    }

    void print(string prefix = ""){
        for (const auto& p: points){
            cout << prefix << "Point(" << p.x << ", " << p.y << ")\n";
        }
        if (divided){
            NW->print(prefix + " NW");
            NE->print(prefix + " NE");
            SW->print(prefix + " SW");
            SE->print(prefix + " SE");
        }
    }

    //destructor - delete child nodes
    ~QuadTree(){
        delete NE;
        delete NW;
        delete SE;
        delete SW;
    }
};

Point toQuadPoint(double latitude, double longitude){
    return {static_cast<int>(latitude*10000), static_cast<int>(longitude*10000)};
}
