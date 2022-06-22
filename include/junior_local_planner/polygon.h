#ifndef POLYGON_H_
#define POLYGON_H_

#include "junior_local_planner/struct_defs.h"
#include "junior_local_planner/matrix.h"

#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#define MIN(x, y) (x <= y ? x : y)
#define MAX(x, y) (x >= y ? x : y)

namespace bg = boost::geometry; 
typedef bg::model::d2::point_xy<double> point_xy;

class Polygon
{
public:
    std::vector<Waypoint> points;

    inline bool PointInsidePolygon(const Polygon& polygon,const Waypoint& p)
    {
        int counter = 0;
        int i;
        double xinters;
        Waypoint p1, p2;
        int N = polygon.points.size();
        if(N <= 0) return false;

        p1 = polygon.points.at(0);
        for(i = 1; i <= N; i++)
        {
            p2 = polygon.points.at(i % N);

            if (p.y > MIN(p1.y, p2.y))
            {
                if (p.y <= MAX(p1.y, p2.y))
                {
                    if (p.x <= MAX(p1.x, p2.x))
                    {
                        if (p1.y != p2.y)
                        {
                            xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
                            if (p1.x == p2.x || p.x <= xinters)
                                counter++;
                        }
                    }
                }
            }
            p1 = p2;
        }

        if (counter % 2 == 0)
            return false;
        else
            return true;
    }

    inline bool BoxInsidePolygon(const Polygon& polygon,const BoxObstacle& b)
    {
        const clock_t begin_time = clock();

        std::vector<point_xy> points;
        for(int i = 0; i < polygon.points.size(); i++)
        {
            points.emplace_back(point_xy(polygon.points[i].x, polygon.points[i].y));
        }
        bg::model::polygon<point_xy> poly1;
        bg::assign_points(poly1, points);
        
        Mat3 rotationMat(b.heading-M_PI_2);
        Mat3 translationMat(b.x, b.y);

        Waypoint bottom_left;
        bottom_left.x = -b.width/2.0;
        bottom_left.y = -b.length/2.0;

        Waypoint bottom_right; 
        bottom_right.x = b.width/2.0;
        bottom_right.y = -b.length/2.0;

        Waypoint top_right;
        top_right.x = b.width/2.0;
        top_right.y = b.length/2.0;

        Waypoint top_left;
        top_left.x = -b.width/2.0;
        top_left.y = b.length/2.0;

        bottom_left = rotationMat*bottom_left;
        bottom_left = translationMat*bottom_left;

        bottom_right = rotationMat*bottom_right;
        bottom_right = translationMat*bottom_right;

        top_right = rotationMat*top_right;
        top_right = translationMat*top_right;

        top_left = rotationMat*top_left;
        top_left = translationMat*top_left;

        points.clear();
        points.emplace_back(point_xy(bottom_left.x, bottom_left.y));
        points.emplace_back(point_xy(bottom_right.x, bottom_right.y));
        points.emplace_back(point_xy(top_right.x, top_right.y));
        points.emplace_back(point_xy(top_left.x, top_left.y));
        points.emplace_back(point_xy(bottom_left.x, bottom_left.y));

        bg::model::polygon<point_xy> poly2;
        bg::assign_points(poly2, points);
        
        // std::cout << "Time elapsed: " << float( clock () - begin_time )/CLOCKS_PER_SEC*pow(10,3) << "ms" << std::endl;

        return bg::intersects(poly1, poly2);
    }
};

#endif