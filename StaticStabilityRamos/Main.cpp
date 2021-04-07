/** @file Main.cpp
 *
 *  Exemplary Code to check the Static Stability Constraint proposed by Ramos (2015).
 *
 *  @author Corinna Krebs
 *  @bug No known bugs.
 */


#include <iostream>
#include "../../Validator/cpp/Validator/ConstraintsLoading.h"
#include "../../Validator/cpp/Validator/ConstraintsRouting.h"
#include "../../Validator/cpp/Validator/Instance.h"
#include "../../Validator/cpp/Validator/Read.h"
#include "StaticStabilityRamos.h"

int main() {
	validator::Instance	  instance			= validator::Read::readInstanceFile("../../Input/Instances/Krebs_Ehmke_Koch_2020/001_n020_m200_bt3.txt");
	validator::ConstraintSet constraintSet	= validator::Read::readConstraintFile("../../Input/Constraint_Sets/P15.txt");
	validator::Solution	  solution			= validator::Read::readSolutionFile("../../Input/PackPlan/001_n020_m200_bt3_P15_1.txt", instance);
	constraintSet.vStability = validator::VerticalStability::none;

	if (validator::ConstraintsRouting::checkRoutingConstraints(solution, constraintSet, instance)
		&& validator::ConstraintsLoading::checkLoadingConstraints(solution, constraintSet, instance)
		&& StaticStabilityRamos::checkStaticStabilityRamos(instance, true)) {
		std::cout << "All Constraints checked. Solution is feasible." << std::endl;
		return 1;
	}
	std::cerr << "Solution is not feasible. Please check error hints above." << std::endl;
	return 0;
}
