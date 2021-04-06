#include "StaticStabilityRamos.h"

#include <iostream>

#include "../../Validator/cpp/Validator/Instance.h"
#include "../../Validator/cpp/Validator/Tour.h"


void StaticStabilityRamos::addItemGravitationalForce(const validator::Item& item, long& x, long& y, long& sum) {
	const float G = 9.81f;
	const long force = (long) item.mass * G;
	const unsigned int length = item.rotated ? item.w : item.h;
	const unsigned int width = item.rotated ? item.l : item.w;

	x += (long) (item.min.x + length * 0.5) * force;
	y += (long) (item.min.y + width * 0.5) * force;
	sum += force;
}

bPoint StaticStabilityRamos::getResultantForcePoint(const validator::Item& item, validator::Instance& instance) {

	long x = 0, y = 0, resultantForce = 0;

	addItemGravitationalForce(item, x, y, resultantForce);
	for (const auto& id : item.itemsAbove)	{
		addItemGravitationalForce(instance.items.at(id), x, y, resultantForce);
	}
	x /= resultantForce;
	y /= resultantForce;
	return { (double) x, (double) y };
}

bool StaticStabilityRamos::checkStaticStabilityRamos(validator::Instance& instance, const bool& msg) {
	// Item Sets (itemsAbove, itemsBelow) must be filled via getRelevantItems beforehand
	for (const auto& item : instance.items) {
		if (!checkItem(item, instance)) {
			if (msg) std::cerr << "No stable position of Item " << item.id << std::endl;
			return false;
		}
	}
	return true;
}

bool StaticStabilityRamos::checkItem(const validator::Item& item, validator::Instance& instance) {
	
	bPoint p = getResultantForcePoint(item, instance);
	
	std::vector<bPoint> vertices;
	vertices.reserve(item.itemsBelow.size() * 4);
	bool stable = false;
	for (const auto& id : item.itemsBelow)
	{
		validator::Item& item_k = instance.items.at(id);
		if ((item_k.min.x <= p.get<0>() <= item_k.max.x) && (item_k.min.y <= p.get<1>() <= item_k.max.y)) {
			stable = true;
		}

		// Determine Intersection Points
		vertices.emplace_back(bPoint(std::max(item.min.x, item_k.min.x), std::max(item.min.y, item_k.min.y)));
		vertices.emplace_back(bPoint(std::min(item.max.x, item_k.max.x), std::max(item.min.y, item_k.min.y)));
		vertices.emplace_back(bPoint(std::max(item.min.x, item_k.min.x), std::min(item.max.y, item_k.max.y)));
		vertices.emplace_back(bPoint(std::min(item.max.x, item_k.max.x), std::min(item.max.y, item_k.max.y)));
	}

	if (!stable) {
		// Get Convex Hull
		boost::geometry::model::polygon<bPoint> hull;
		boost::geometry::model::polygon<bPoint> polygon;
		boost::geometry::convex_hull(polygon, hull);

		// Point in Polygon Test
		stable = boost::geometry::within(p, hull);
	}

	if (stable)	{
		for (const auto& id : item.itemsBelow) {
			if (!checkItem(instance.items.at(id), instance)) {
				return false;
			}
		}
	}

	return true;
}
