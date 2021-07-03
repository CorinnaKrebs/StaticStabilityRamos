#include "pch.h"
#include "CppUnitTest.h"
#include "../../Validator/cpp/Validator/ConstraintsLoading.h"
#include "../../StaticStabilityRamos/StaticStabilityRamos/StaticStabilityRamos.h"
#include <boost/geometry.hpp>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace validator;

namespace StaticStability
{
	TEST_CLASS(ResultantForce)	{
		const int dim = 4;
	public:


		TEST_METHOD(Single) {
			// Items are stacked
			ItemType type(1, dim, dim, dim, 10, false, 0);

			Item item1(0, 1, 0, type);

			// Instance Creation
			Instance instance("", Vehicle(0, 0, 0, 0, 0, 0, 0, 0, 0), {}, {}, 0, 0);
			instance.items.emplace_back(item1);

			// Tour Creation
			Tour tour(1, {0}, {1});

			// Positions
			instance.items.at(item1.id).setPosition(validator::Point(0, 0, 0));
			bPoint p = StaticStabilityRamos::getResultantForcePoint(instance.items.at(item1.id), instance);

			// The Point of the Resultant Force is not influenced
			Assert::AreEqual(boost::geometry::get<0>(p), dim * 0.5);
			Assert::AreEqual(boost::geometry::get<1>(p), dim * 0.5);

		}
		
		TEST_METHOD(Stacked) {
			// Items are stacked
			ItemType type(1, dim, dim, dim, 10, false, 0);

			Item item1(0, 1, 0, type);
			Item item2(1, 1, 0, type);

			// Instance Creation
			Instance instance("", Vehicle(0, 0, 0, 0, 0, 0, 0, 0, 0), {}, {}, 0, 0);
			instance.items.emplace_back(item1);
			instance.items.emplace_back(item2);

			// Tour Creation
			std::vector<unsigned int> customer_ids{ 1 };
			std::vector<unsigned int> item_ids{ 0, 1 };
			Tour tour(1, customer_ids, item_ids);

			// Positions
			instance.items.at(item1.id).setPosition(validator::Point(0, 0, 0));
			instance.items.at(item2.id).setPosition(validator::Point(0, 0, dim));

			const unsigned int endPos = tour.item_ids.size();
			validator::ConstraintsLoading::getRelevantItems(instance.items.at(item1.id), tour, endPos, instance);
			validator::ConstraintsLoading::getRelevantItems(instance.items.at(item2.id), tour, endPos, instance);

			bPoint p = StaticStabilityRamos::getResultantForcePoint(instance.items.at(item1.id), instance);

			// The Point of the Resultant Force is not influenced
			Assert::AreEqual(boost::geometry::get<0>(p), dim * 0.5);
			Assert::AreEqual(boost::geometry::get<1>(p), dim * 0.5);

		}

		TEST_METHOD(Middle) {
			// Items are stacked
			ItemType type(1, dim, dim, dim, 10, false, 0);

			Item item1(0, 1, 0, type);
			Item item2(1, 1, 0, type);

			// Instance Creation
			Instance instance("", Vehicle(0, 0, 0, 0, 0, 0, 0, 0, 0), {}, {}, 0, 0);
			instance.items.emplace_back(item1);
			instance.items.emplace_back(item2);

			// Tour Creation
			std::vector<unsigned int> customer_ids{ 1 };
			std::vector<unsigned int> item_ids{ 0, 1 };
			Tour tour(1, customer_ids, item_ids);

			// Positions
			instance.items.at(item1.id).setPosition(validator::Point(0, 0, 0));
			instance.items.at(item2.id).setPosition(validator::Point(0, dim/2, dim));

			const unsigned int endPos = tour.item_ids.size();
			validator::ConstraintsLoading::getRelevantItems(instance.items.at(item1.id), tour, endPos, instance);
			validator::ConstraintsLoading::getRelevantItems(instance.items.at(item2.id), tour, endPos, instance);

			bPoint p = StaticStabilityRamos::getResultantForcePoint(instance.items.at(item1.id), instance);

			// The Point of the Resultant Force is in the center of both item's center of gravity
			Assert::AreEqual(dim * 0.5, boost::geometry::get<0>(p));
			Assert::AreEqual(dim * 0.75, boost::geometry::get<1>(p));

		}

		TEST_METHOD(Weighted) {
			int mass2 = 10;
			int mass1 = 2 * mass2;
			
			// Items are stacked
			ItemType type1(1, dim, dim, dim, mass1, false, 0);
			ItemType type2(1, dim, dim, dim, mass2, false, 0);

			Item item1(0, 1, 0, type1);
			Item item2(1, 1, 0, type2);

			// Instance Creation
			Instance instance("", Vehicle(0, 0, 0, 0, 0, 0, 0, 0, 0), {}, {}, 0, 0);
			instance.items.emplace_back(item1);
			instance.items.emplace_back(item2);

			// Tour Creation
			std::vector<unsigned int> customer_ids{ 1 };
			std::vector<unsigned int> item_ids{ 0, 1 };
			Tour tour(1, customer_ids, item_ids);

			// Positions
			instance.items.at(item1.id).setPosition(validator::Point(0, 0, 0));
			instance.items.at(item2.id).setPosition(validator::Point(0, dim / 2, dim));

			const unsigned int endPos = tour.item_ids.size();
			validator::ConstraintsLoading::getRelevantItems(instance.items.at(item1.id), tour, endPos, instance);
			validator::ConstraintsLoading::getRelevantItems(instance.items.at(item2.id), tour, endPos, instance);

			bPoint p = StaticStabilityRamos::getResultantForcePoint(instance.items.at(item1.id), instance);

			// Due to different masses, the point is drawn to the center
			Assert::AreEqual(dim * 0.5, boost::geometry::get<0>(p));
			Assert::AreEqual(dim * 0.5, boost::geometry::get<1>(p));

		}
	};
}
