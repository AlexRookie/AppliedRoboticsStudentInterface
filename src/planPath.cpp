#pragma once
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <atomic>
#include <unistd.h>

#include <sstream>
#include <experimental/filesystem>
#include "clipper.hpp"
#include "dubins.h"

namespace student{

    int robot_radius = 60;

    std::vector<Polygon> obstacleOffsetting(const std::vector<Polygon> ob){

        std::vector<Polygon> offsettedObstacles;

        for(int i = 0; i < ob.size(); i++){
            ClipperLib::Path srcPoly;  
            ClipperLib::Paths newPoly; 
            ClipperLib::ClipperOffset co;

            Polygon temp;

            const double INT_ROUND = 1000;
            // Push all points of obstacle polygon to clipper lib method
            for(size_t a = 0; a < ob[i].size(); ++a){
                int x = ob[i][a].x * INT_ROUND;
                int y = ob[i][a].y * INT_ROUND;

                srcPoly << ClipperLib::IntPoint(x, y);
            }

            // If not a closed polygon
            if(ob[i].size() == 3)
            {
                co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedLine);
            }
            else // If it is a closed polygon
            {
                co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
            }
            co.Execute(newPoly, robot_radius);
            for(const ClipperLib::Path &path: newPoly){
                for(const ClipperLib::IntPoint &pt: path){
                    double x = pt.X / INT_ROUND;
                    double y = pt.Y / INT_ROUND;
                    temp.emplace_back(x, y);
                }
            }

            offsettedObstacles.emplace_back(temp);
        }

        return offsettedObstacles;
    }

    std::vector<Polygon> mergePolygon(const std::vector<Polygon> &obstacle_list, const Polygon &borders){
        std::vector<Polygon> new_list;
        std::vector<Polygon> clipped_list;
        ClipperLib::Paths solution;
        ClipperLib::Clipper c;
        ClipperLib::Paths clip(1);

        const double INT_ROUND = 1000.0;
        for(int i= 0; i < obstacle_list.size(); i++){
            ClipperLib::Paths subj(1);
            for (const auto &pt: obstacle_list[i]) {
                subj[0] << ClipperLib::IntPoint(pt.x*INT_ROUND, pt.y*INT_ROUND);
            }

            c.AddPaths(subj, ClipperLib::ptSubject, true);
        }

        c.Execute(ClipperLib::ctUnion, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

        for(int i= 0; i < solution.size(); i++){
            Polygon p;
            for (const auto &pt: solution[i]) {
                p.push_back(Point((double) pt.X/INT_ROUND, (double)pt.Y/INT_ROUND));
            }

            new_list.emplace_back(p);
        }

        ClipperLib::Clipper inter;
        ClipperLib::Paths sol2;

        for(int i= 0; i < new_list.size(); i++){
            ClipperLib::Paths subj(1);
            for (const auto &pt: new_list[i]) {
                subj[0] << ClipperLib::IntPoint(pt.x*INT_ROUND, pt.y*INT_ROUND);
            }

            inter.AddPaths(subj, ClipperLib::ptSubject, true);
        }

        for(auto &pt: borders){
            clip[0] << ClipperLib::IntPoint(pt.x*INT_ROUND, pt.y*INT_ROUND);
        }

        inter.AddPaths(clip, ClipperLib::ptClip, true);
        inter.Execute(ClipperLib::ctIntersection, sol2, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

        for(int i= 0; i < sol2.size(); i++){
            Polygon p;
            for (const auto &pt: sol2[i]) {
                p.push_back(Point((double) pt.X/INT_ROUND, (double)pt.Y/INT_ROUND));
            }

            clipped_list.emplace_back(p);
        }


        return clipped_list;
    }

    
    bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path){
        std::vector<Polygon> obsctacle_offset = obstacleOffsetting(obstacle_list);
        std::vector<Polygon> merged_list = mergePolygon(obsctacle_offset, borders);

    }
}
