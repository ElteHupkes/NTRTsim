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
 * @file tgSimulation.cpp
 * @brief Contains the definitions of members of class tgSimulation
 * @author Ryan Adams, Brian Mirletz
 * $Id$
 */

// This module
#include "tgSimulation.h"
// This application
#include "tgModel.h"
#include "tgSimView.h"
#include "tgSimViewGraphics.h"
#include "tgWorld.h"
// The Bullet Physics Library
#include "LinearMath/btQuickprof.h"

// The C++ Standard Library
#include <stdexcept>

tgSimulation::tgSimulation(tgSimView& view) :
  m_view(view)
{
        m_view.bindToSimulation(*this);

    m_view.setup();

    // Postcondition
    assert(invariant());
}

tgSimulation::~tgSimulation()
{
    teardown();
    m_view.releaseFromSimulation();
    for (int i = 0; i < m_models.size(); i++)
    {
        delete m_models[i];
    }
}

void tgSimulation::addModel(tgModel* pModel)
{
    // Precondition
    if (pModel == NULL)
    {
        throw std::invalid_argument("NULL pointer to tgModel");
    }
    else
    {

        pModel->setup(m_view.world());
        m_models.push_back(pModel);
    }

    // Postcondition
    assert(invariant());
    assert(!m_models.empty());
}

void tgSimulation::onVisit(const tgModelVisitor& r) const
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgSimulation::onVisit");
#endif //BT_NO_PROFILE	
        // Removed sending the visitor to the world since it wasn't used
        // Write a worldVisitor if its necessary
        for (int i = 0; i < m_models.size(); i++) {
            m_models[i]->onVisit(r);
        }
}

void tgSimulation::reset()
{

    teardown();

    m_view.setup();
    for (int i = 0; i != m_models.size(); i++)
    {
        
        m_models[i]->setup(m_view.world());
    }
}

/**
 * @note This is not inlined because it depends on the definition of tgSimView.
 */
tgWorld& tgSimulation::getWorld() const
{
    return m_view.world();
}

void tgSimulation::step(double dt) const
{
// Trying to profile here creates trouble for tgLinearString -  this is outside of the profile loop	
	
        if (dt <= 0)
    {
        throw std::invalid_argument("dt for step is not positive");
    }
    else
    {
        // Step the world.
        // This can be done before or after stepping the models.
        m_view.world().step(dt);

        // Step the models
        for (int i = 0; i < m_models.size(); i++)
        {
            m_models[i]->step(dt);
        }
    }
}
  
void tgSimulation::teardown() const
{
    const size_t n = m_models.size();
    for (int i = 0; i < n; i++)
    {
        tgModel * const pModel = m_models[i];
        assert(pModel != NULL);
        
        pModel->teardown();
    }
    // Reset the world after the models - models need world info for
    // their onTeardown() functions
    m_view.world().reset();
    // Postcondition
    assert(invariant());
}

void tgSimulation::run() const
{
    m_view.run();
}

void tgSimulation::run(int steps) const
{    
    m_view.run(steps);
}

bool tgSimulation::invariant() const
{
  return true;
}   
