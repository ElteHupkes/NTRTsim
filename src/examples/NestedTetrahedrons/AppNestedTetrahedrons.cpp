/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file AppNestedTetrahedrons.cpp
 * @brief Contains the definition function main() for the Nested Tetrahedrons
 * application.
 * @author Brian Tietz
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

// This application
#include "NestedStructureTestModel.h"
#include "NestedStructureSineWaves.h"
// This library
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
// The C++ Standard Library
#include <iostream>
#include <cmath>
#include <cstdlib>
// Other libraries
#include <omp.h>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv)
{
    std::cout << "StressTest" << std::endl;

    const double radius = 450;
    const double stepSize = 1.0/500.0; //Seconds
    int nSteps = 3000; // Number of steps in each episode, 3k is 6 seconds (timestep_physics*nSteps)

    bool visualize = argc > 1 && (std::string(argv[1]) == "--visualize");

    // i = # of segments, j = # of bots
    for (unsigned int segments = 4; segments <= 12; segments += 4) {
    	for (unsigned int nBots = 15; nBots <= 75; nBots += 15) {
    	    // First create the world
    	    const tgWorld::Config config(981, 50); // gravity, cm/sec^2
    	    tgWorld world(config);

    	    // Second create the view
    	    tgSimView* view;
    	    if (visualize) {
    	    	view = new tgSimViewGraphics(world, stepSize);
    	    	std::cout << "Enabling visualization" << std::endl;
    	    } else {
    	    	view = new tgSimView(world, stepSize);
    	    }

    	    // Third create the simulation
    	    tgSimulation simulation(*view);

    	    // Fourth create the models with their controllers and add the models to the
    	    // simulation
    	    std::vector<NestedStructureSineWaves*> waves;
    	    waves.reserve(nBots);

    	    unsigned int i;
    	    for (i = 0; i < nBots; ++i) {
    	    	double angle = 2 * i * M_PI/nBots;
    	    	btVector3 move(radius * cos(angle), 0, radius * sin(angle));

    	    	NestedStructureTestModel* model = new NestedStructureTestModel(segments, move,
    	    			1.5 * M_PI - angle - (0.1 - 0.2 * rand() / (RAND_MAX + 1.0)));
    	    	NestedStructureSineWaves* wave = new NestedStructureSineWaves();
    	    	model->attach(wave);
    	    	simulation.addModel(model);

    	    	waves.push_back(wave);
    	    }

    	    std::cout << "Simulating " << nBots << " bots with " << segments << " segments..." << std::endl;
    	    double start = omp_get_wtime();
			simulation.run(nSteps);
			double end = omp_get_wtime();
			std::cout << "Bots: " << nBots << ", segments: " << segments << ", time: " << (end - start) << std::endl;

			// Models are deleted in simulation teardown, minds are not
    	    for (unsigned i = 0; i < nBots; ++i) {
    	    	delete waves[i];
    	    }

    	    delete view;
    	}
    }

    //Teardown is handled by delete, so that should be automatic
    return 0;
}
