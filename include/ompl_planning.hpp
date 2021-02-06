#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "ompl/util/Console.h"
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/PlannerDataGraph.h>


#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>

#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <memory>

#include <fstream>

#include "utils.hpp"

#include<iostream>
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> boost_point;
typedef bg::model::polygon <boost_point> boost_polygon;
typedef boost::geometry::model::linestring<boost_point> boost_linestring;

/// @brief Choice of Optimal planner
enum optimalPlanner
{
    ///  PRM STAR implementation
    PLANNER_PRMSTAR,
    /// RRT Star implementation
    PLANNER_RRTSTAR,
};

/// @brief An enum of the supported optimization objectives
enum planningObjective
{
    /// Path clearance objective
    OBJECTIVE_PATHCLEARANCE,
    /// Path length objective 
    OBJECTIVE_PATHLENGTH,
    /// Path length with threshold based objective
    OBJECTIVE_THRESHOLDPATHLENGTH,
    /// Weighted combination obejctive of all the above three or two
    OBJECTIVE_WEIGHTEDCOMBO
};

/**
*  @brief get path length objective using the space information configured 
*  @param si Space information   
*  @return  Optimization objective pointer 
*/

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}

/**
*  @brief get path length objective with threshold using the space information configured 
*  @param si Space information   
*  @return  Optimization objective pointer 
*/


ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}


class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
    }

    // Our requirement is to maximize path clearance from obstacles,
    // but we want to represent the objective as a path cost
    // minimization. Therefore, we set each state's cost to be the
    // reciprocal of its clearance, so that as state clearance
    // increases, the state cost decreases.
    ob::Cost stateCost(const ob::State* s) const override
    {
        return ob::Cost(1 / (si_->getStateValidityChecker()->clearance(s) +
            std::numeric_limits<double>::min()));
    }
};

/**
*  @brief get Obstace clearance objective using the space information configured 
*  @param si Space information   
*  @return  Optimization objective pointer 
*/

ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ClearanceObjective>(si);
}

/**
*  @brief get Path length objective with go to heuristic cost using the space information configured 
*  @param si Space information   
*  @return  Optimization objective pointer 
*/

ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
    return obj;
}

/**
*  @brief get balanced objective between Path length optimization and Object clearance 
*  @param si Space information   
*  @return  Optimization objective pointer 
*/


ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si)
{
    auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    auto clearObj(std::make_shared<ClearanceObjective>(si));

    return 10.0*lengthObj + clearObj;
}

/**
*  @brief get balanced objective between Path length optimization, Object clearance and Multi Optimization 
*  @param si Space information   
*  @return  Optimization objective pointer 
*/
ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si)
{
    auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    auto clearObj(std::make_shared<ClearanceObjective>(si));
    auto opt(std::make_shared<ob::MultiOptimizationObjective>(si));
    opt->addObjective(lengthObj, 10.0);
    opt->addObjective(clearObj, 1.0);

    return ob::OptimizationObjectivePtr(opt);
}

/**
*  @brief Allocate planner 
*  @details only two types of planner are considered RRT* and PRM_RRT* 
*  @param si Space information
*  @param plannerType  Type of planner to be used    
*  @return Planner Pointer with corresponding planner 
*/
ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, optimalPlanner plannerType)
{
    switch (plannerType)
    {
        case PLANNER_PRMSTAR:
        {
            return std::make_shared<og::PRMstar>(si);
            break;
        }
        case PLANNER_RRTSTAR:
        {
            return std::make_shared<og::RRTstar>(si);
            break;
        }
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
        }
    }
}


/**
*  @brief Allocate objective 
*  @details Four objectives of planning 
*  @param si Space information which includes PATHCLEARANCE, PATHLENGTH, THRESHOLDPATHLENGTH, WEIGHTEDCOMBO
*  @param objectiveType  Type of Objective to be used    
*  @return Objective Pointer with corresponding planner 
*/
ob::OptimizationObjectivePtr allocateObjective(const ob::SpaceInformationPtr& si, planningObjective objectiveType)
{
    switch (objectiveType)
    {
        case OBJECTIVE_PATHCLEARANCE:
            return getClearanceObjective(si);
            break;
        case OBJECTIVE_PATHLENGTH:
            return getPathLengthObjective(si);
            break;
        case OBJECTIVE_THRESHOLDPATHLENGTH:
            return getThresholdPathLengthObj(si);
            break;
        case OBJECTIVE_WEIGHTEDCOMBO:
            return getBalancedObjective1(si);
            break;
        default:
            OMPL_ERROR("Optimization-objective enum is not implemented in allocation function.");
            return ob::OptimizationObjectivePtr();
            break;
    }
}

/**
*  @brief Convert Polygon To Boost polygon 
*  @details Interface polygon struct is converted to boost polygon for easy operation
*  @param poly Polygon to be converted
*  @return Boost polygon object 
*/
boost_polygon convertPolygonToBoostPolygon(const Polygon &poly) {
    boost_polygon boost_poly;

    for(auto &iter : poly){
        bg::append(boost_poly.outer(), boost_point(iter.x, iter.y));
    }
    
    bg::append(boost_poly.outer(), boost_point(poly[0].x, poly[0].y));

    return boost_poly;
}

/**
*  @brief Class to check if new random state is valid or not    
*/
class ValidityChecker : public ob::StateValidityChecker
{
public:

    std::vector<Polygon> obstacles;
    /**
    *  @brief Constructor 
    *  @param obstacle_list Obstacle list that is obtained from map 
    */
    ValidityChecker(const ob::SpaceInformationPtr& si,std::vector<Polygon> obstacle_list ) :
        ob::StateValidityChecker(si) {obstacles = obstacle_list;}

    /**
    *  @brief to check if state is valid of not 
    *  @details Check if new obatined state is inside the obstacle polygon, convert new state to boost point and   
    *  also polygon to boost polygon and use boost::within() method to check the validity of state    
    *  @param state new state that is generated randomly by RRT Star algorithm
    *  @return true/false based on the validity of state 
    */
    bool isValid(const ob::State* state) const override
    {
        //return this->clearance(state) > 0.0;
        const ob::RealVectorStateSpace::StateType* state2D = state->as<ob::RealVectorStateSpace::StateType>();
        double x = state2D->values[0];
        double y = state2D->values[1];
        boost_point centerPoint(x, y);
        for (Polygon polygon : obstacles) {
                boost_polygon Poly = convertPolygonToBoostPolygon(polygon);
                if (boost::geometry::within(centerPoint, Poly))
                    return false;
        }
        return true;
    }


};

