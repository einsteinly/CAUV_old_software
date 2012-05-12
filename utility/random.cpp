/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "random.h"

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>


namespace { boost::mt19937 static_gen; }
int cauv::random(int min, int max)
{
	boost::uniform_int<> dist(min, max);
	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > die(static_gen, dist);

	return die();
}
int cauv::random(int min, int max, unsigned int seed)
{
	boost::mt19937 gen;
	gen.seed(seed);
	boost::uniform_int<> dist(min, max);
	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > die(gen, dist);

	return die();
}
