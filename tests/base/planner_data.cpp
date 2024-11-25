/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ryan Luna */

#define BOOST_TEST_MODULE "PlannerData"
#include <boost/test/unit_test.hpp>
#include <boost/serialization/export.hpp>
#include <iostream>
#include <vector>

#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerDataStorage.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

using namespace ompl;

// define a convenience macro
#define BOOST_OMPL_EXPECT_NEAR(a, b, diff) BOOST_CHECK_SMALL((a) - (b), diff)


class PlannerDataTestVertex : public ompl::base::PlannerDataVertex
{
public:
    PlannerDataTestVertex (base::State* st, int tag = 0, int tag2 = 0) : ompl::base::PlannerDataVertex(st, tag), tag2_(tag2) {}
    PlannerDataTestVertex (const PlannerDataTestVertex &rhs) : ompl::base::PlannerDataVertex(rhs.state_, rhs.tag_), tag2_(rhs.tag2_) {}

    ompl::base::PlannerDataVertex* clone () const override
    {
        return static_cast<ompl::base::PlannerDataVertex*>(new PlannerDataTestVertex(*this));
    }

    int tag2_;

protected:
    PlannerDataTestVertex() = default;

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive & ar, const unsigned int)
    {
        ar & boost::serialization::base_object<ompl::base::PlannerDataVertex>(*this);
        ar & tag2_;
    }
};

// This allows us to serialize the derived class PlannerDataTestVertex
BOOST_CLASS_EXPORT(PlannerDataTestVertex);

// Comment out to see the test passing
#define TRIGGER_CONDA_FORGE_FAILURE

BOOST_AUTO_TEST_CASE(Serialization)
{
    auto space(std::make_shared<base::RealVectorStateSpace>(1));
    auto si(std::make_shared<base::SpaceInformation>(space));
    base::PlannerData data(si);
    std::vector<base::State*> states;

    // Creating 5 states
    for (unsigned int i = 0; i < 5; ++i)
    {
        states.push_back(space->allocState());
        states[i]->as<base::RealVectorStateSpace::StateType>()->values[0] = (double)i;

#if defined(TRIGGER_CONDA_FORGE_FAILURE)
        PlannerDataTestVertex vtx(states[i], i, i+1);
#else
        ompl::base::PlannerDataVertex vtx(states[i], i);
#endif
        BOOST_CHECK (data.addVertex(vtx) == i );
        BOOST_CHECK (data.getVertex(i).getTag() == (signed)i);
#if defined(TRIGGER_CONDA_FORGE_FAILURE)
        BOOST_CHECK (static_cast<PlannerDataTestVertex&>(data.getVertex(i)).tag2_ == (signed)i+1);
#endif
    }

    // Mark some start and goal states
    data.markStartState(states[0]);
    data.markStartState(states[states.size()/2]);
    data.markStartState(states[states.size()-1]);
    data.markGoalState(states[1]);
    data.markGoalState(states[states.size()-2]);

    // Add a whole bunch of random edges
    unsigned int num_edges_to_add = 0;
    ompl::RNG rng(0);

    for (unsigned int i = 0; i < num_edges_to_add; ++i)
    {
        unsigned int v2, v1 = rng.uniformInt(0, states.size()-1);
        do v2 = rng.uniformInt(0, states.size()-1); while (v2 == v1 || data.edgeExists(v1, v2));

        BOOST_CHECK( data.addEdge(v1, v2) );
    }

    BOOST_CHECK_EQUAL ( data.numVertices(), states.size() );
    BOOST_CHECK_EQUAL ( data.numEdges(), num_edges_to_add );

    base::PlannerData data2(si);
    base::PlannerDataStorage storage;
    storage.store(data, "testdata");
    storage.load("testdata", data2);

    // Verify that data == data2
    BOOST_CHECK_EQUAL ( data2.numVertices(), states.size() );
    BOOST_CHECK_EQUAL ( data2.numEdges(), num_edges_to_add );

    // Check our start/goal states
    BOOST_CHECK ( data2.numStartVertices() == 3 );
    BOOST_CHECK ( data2.numGoalVertices() == 2 );
    BOOST_CHECK ( data2.isStartVertex(0) );
    BOOST_CHECK ( data2.isStartVertex(states.size()/2) );
    BOOST_CHECK ( data2.isStartVertex(states.size()-1) );
    BOOST_CHECK ( data2.isGoalVertex(1) );
    BOOST_CHECK ( data2.isGoalVertex(states.size()-2) );

    for (size_t i = 0; i < states.size(); ++i)
    {
        BOOST_CHECK (space->equalStates(data2.getVertex(i).getState(), states[i]) );
        BOOST_CHECK (data.getVertex(i).getTag() == data2.getVertex(i).getTag() );
#if defined(TRIGGER_CONDA_FORGE_FAILURE)
        BOOST_CHECK (static_cast<PlannerDataTestVertex&>(data2.getVertex(i)).tag2_ == (signed)i+1);
#endif
    }

    for (size_t i = 0; i < states.size(); ++i)
    {
        std::vector<unsigned int> neighbors, neighbors2;
        data.getEdges(i, neighbors);
        data2.getEdges(i, neighbors2);

        std::sort (neighbors.begin(), neighbors.end());
        std::sort (neighbors2.begin(), neighbors2.end());
        BOOST_REQUIRE_EQUAL( neighbors.size(), neighbors2.size() );

        for (size_t j = 0; j < neighbors.size(); ++j)
            BOOST_CHECK_EQUAL( neighbors[j], neighbors2[j] );
    }

    for (auto & state : states)
        space->freeState(state);
}
