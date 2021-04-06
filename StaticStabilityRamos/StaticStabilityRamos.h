#pragma once

#include "../../Validator/cpp/Validator/Instance.h"
#include "../../Validator/cpp/Validator/Item.h"
#include <boost/geometry.hpp>

typedef boost::geometry::model::d2::point_xy<double> bPoint;
typedef boost::geometry::model::polygon<bPoint> polygon_type;

class StaticStabilityRamos {
	
public:
	/**
	 * Calculates the Gravitational Force of an Item and adds it to the method parameters.
	 */
	static void addItemGravitationalForce(const validator::Item& item, long& x, long& y, long& sum);

	/**
	 * Calculates the Gravitational Force of an Item and adds it to the method parameters.
	 */
	static bPoint getResultantForcePoint(const validator::Item& item, validator::Instance& instance);
	/**
	 * Checks the Static Stability for one item.
	 */
	static bool checkItem(const validator::Item& item, validator::Instance& instance);
	/**
	 * Checks the Static Stability for all items of the instance.
	 * The sets itemsAbove and itemsBelow of each item must be provided (e.g. via getRelevantItems beforehand).
	 */
	static bool checkStaticStabilityRamos(validator::Instance& instance, const bool& msg);
};