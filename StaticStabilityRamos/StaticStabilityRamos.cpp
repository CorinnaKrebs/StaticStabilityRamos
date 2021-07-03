#include "StaticStabilityRamos.h"

#include <iostream>

#include "../../Validator/cpp/Validator/Instance.h"
#include "../../Validator/cpp/Validator/Tour.h"


void StaticStabilityRamos::addItemGravitationalForce(const validator::Item& item, long double& x, long double& y, long double& sum) {
	const float G = 1.0f;
	const double force = item.mass * G;
	const double length = item.rotated ? item.w : item.l;
	const double width = item.rotated ? item.l : item.w;

	x += (item.min.x + length * 0.5) * force;
	y += (item.min.y + width * 0.5) * force;
	sum += force;
}

bPoint StaticStabilityRamos::getResultantForcePoint(validator::Item& item, validator::Instance& instance) {

	long double x = 0, y = 0, resultantForce = 0;

	addItemGravitationalForce(item, x, y, resultantForce);
	for (const auto& id : item.itemsAbove)  {
		auto& item_above = instance.items.at(id);
		if (!item_above.isAbove(item, true)) continue;
		addItemGravitationalForce(item_above, x, y, resultantForce);
	}
	x /= resultantForce;
	y /= resultantForce;
	return { (double) x, (double) y };
}

bool StaticStabilityRamos::checkStaticStabilityRamos(validator::Instance& instance, const bool& msg) {
	// Item Sets (itemsAbove, itemsBelow) must be filled via getRelevantItems beforehand
	for (auto& item : instance.items) {
		if (item.customer_id == 0) continue;
		if (!checkItem(item, instance)) {
			if (msg) std::cerr << "No stable position of Item " << item.id << std::endl;
			return false;
		}
	}
	return true;
}

bool StaticStabilityRamos::checkItem(validator::Item& item, validator::Instance& instance) {
	if (item.min.z == 0) return true;
	bPoint p = getResultantForcePoint(item, instance);
	
	multi_point vertices;
	vertices.reserve(item.itemsBelow.size() * 4);
	bool stable = false;
	for (const auto& id : item.itemsBelow) {
		validator::Item& item_k = instance.items.at(id);
		if (!item_k.isBelow(item, true)) continue;
		if ((item_k.min.x <= p.get<0>() <= item_k.max.x) && (item_k.min.y <= p.get<1>() <= item_k.max.y)) {
			stable = true;
		}

		// Determine Intersection Points
		vertices.push_back(bPoint(std::max(item.min.x, item_k.min.x), std::max(item.min.y, item_k.min.y)));
		vertices.push_back(bPoint(std::min(item.max.x, item_k.max.x), std::max(item.min.y, item_k.min.y)));
		vertices.push_back(bPoint(std::max(item.min.x, item_k.min.x), std::min(item.max.y, item_k.max.y)));
		vertices.push_back(bPoint(std::min(item.max.x, item_k.max.x), std::min(item.max.y, item_k.max.y)));
	}

	if (!stable) {
		// Get Convex Hull
		polygon hull;
		boost::geometry::convex_hull(vertices, hull);

		// Point in Polygon Test
		stable = boost::geometry::within(p, hull);
	}

	if (stable)	{
		for (const auto& id : item.itemsBelow) {
			if (!checkItem(instance.items.at(id), instance)) {
				stable = false;
				return false;
			}
		}
	}

	return stable;
}
