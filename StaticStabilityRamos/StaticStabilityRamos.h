#pragma once
/** @file StaticStabilityRamos.h
 *  @brief Implementation of Static Stability Constraint.
 *  
 * Class containing all methods necessary to check w.r.t
 * the Static Stability Constraint proposed by Ramos (2015).
 *
 *  @author Corinna Krebs
 *  @bug No known bugs.
 */

#include "../../Validator/cpp/Validator/Instance.h"
#include "../../Validator/cpp/Validator/Item.h"
#include <boost/geometry.hpp>

typedef boost::geometry::model::d2::point_xy<double> bPoint;
typedef boost::geometry::model::multi_point<bPoint> multi_point;
typedef boost::geometry::model::polygon<bPoint> polygon;

class StaticStabilityRamos {
	
public:
	/**
	 * Calculates the Gravitational Force of an Item and adds it to the method parameters.
	 * @param item the item causing the Gravitational Force
	 * @param x the weighted x-coordinate
	 * @param y the weighted y-coordinate
	 * @param sum the sum of forces
	 */
	static void addItemGravitationalForce(const validator::Item& item, long& x, long& y, long& sum);

	/**
	 * Calculates the Resultant Force caused by the item and its above items.
	 * @param item the item for which the Resultant Force should be calculated
	 * @param instance the instance data to get additional information
	 * @return Boost-Point containing the x- and y-coordinate of the point of the resultant force
	 */
	static bPoint getResultantForcePoint(const validator::Item& item, validator::Instance& instance);

	
	/**
	 * Checks the Static Stability for one item.
	 * @param item the item for which the constraint is checked.
	 * @param instance the instance data to get additional information
	 * @return true if the position of the item is stable.
	 */
	static bool checkItem(const validator::Item& item, validator::Instance& instance);

	
	/**
	 * Checks the Static Stability for all items of the instance.
	 * The sets itemsAbove and itemsBelow of each item must be provided (e.g. via getRelevantItems beforehand).
	 * @param instance the instance data to get additional information
	 * @param msg parameter to determine whether a error message should be printed or not.
	 * @return true if the position of all items are stable.
	 */
	static bool checkStaticStabilityRamos(validator::Instance& instance, const bool& msg);
};