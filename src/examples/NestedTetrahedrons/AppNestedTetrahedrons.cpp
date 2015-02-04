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

    const double radius = 250;

    // First create the world
    const tgWorld::Config config(981, 50); // gravity, cm/sec^2
    tgWorld world(config); 

    // Second create the view
    const double stepSize = 1.0/500.0; //Seconds
    tgSimViewGraphics view(world, stepSize);
    //tgSimView view(world, stepSize);

    // Third create the simulation
    tgSimulation simulation(view);

    double start = omp_get_wtime();
    std::cout << start << std::endl;

    // i = # of segments, j = # of bots
//    for (int i = 4; i <= 12; i += 4) {
//    	for (int j = 15; j <= 75; j += 15) {
//
//    	}
//    }
    // Fourth create the models with their controllers and add the models to the
    // simulation
    const int segments = 12;

    const int structures = 3;

    std::vector<NestedStructureTestModel*> models;
    std::vector<NestedStructureSineWaves*> waves;

    unsigned int i;
    for (i = 0; i < structures; ++i) {
    	double angle = 2 * i * M_PI/structures;
    	btVector3 move(radius * cos(angle), 0, radius * sin(angle));
    	NestedStructureTestModel* model = new NestedStructureTestModel(segments, move,
    			1.5 * M_PI - angle - (0.1 - 0.2 * rand() / (RAND_MAX + 1.0)));
    	NestedStructureSineWaves* wave = new NestedStructureSineWaves();
    	model->attach(wave);
    	simulation.addModel(model);

    	models.push_back(model);
    	waves.push_back(wave);
    }

    int nEpisodes = 1;
    int nSteps = 3000; // Number of steps in each episode, 60k is 100 seconds (timestep_physics*nSteps)
    for (int i=0; i<nEpisodes; i++) {
        simulation.run(nSteps);
        simulation.reset();
    }

//    NestedStructureTestModel* myModel = new NestedStructureTestModel(segments);
//    NestedStructureSineWaves* const pMuscleControl =
//      new NestedStructureSineWaves();
//    myModel->attach(pMuscleControl);
//    simulation.addModel(myModel);
	
	// Run until the user stops
//    simulation.run();

    for (i = 0; i < structures; ++i) {
    	delete models[i];
    	delete waves[i];
    }

    //Teardown is handled by delete, so that should be automatic
    return 0;
}
