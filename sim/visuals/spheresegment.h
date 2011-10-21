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

/*
* This source file is part of the osgOcean library
* 
* Copyright (C) 2009 Kim Bale
* Copyright (C) 2009 The University of Hull, UK
* 
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU Lesser General Public License as published by the Free Software
* Foundation; either version 3 of the License, or (at your option) any later
* version.

* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
* http://www.gnu.org/copyleft/lesser.txt.
*/

#pragma once
#include <osg/Geode>
#include <osg/Geometry>

namespace cauv {

    namespace sim {

        class SphereSegment : public osg::Geode
        {
        public:
            SphereSegment( void );

            SphereSegment( float radius,
                           unsigned int longitudeSteps,
                           unsigned int lattitudeSteps,
                           float longStart,
                           float longEnd,
                           float latStart,
                           float latEnd );

            SphereSegment(const SphereSegment& copy, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY);

        protected:
            ~SphereSegment(void);

        public:
            // 0 >= longStart/longEnd <= 180
            // 0 >= latStart/latEnd <= 360
            void compute( float radius,
                          unsigned int longitudeSteps,
                          unsigned int lattitudeSteps,
                          float longStart,
                          float longEnd,
                          float latStart,
                          float latEnd	);
        private:
            osg::Vec2 sphereMap( osg::Vec3& vertex, float radius);

            inline unsigned int idx(unsigned int r, unsigned int c, unsigned int row_len)
            {
                return c + r * row_len;
            }
        };

    } // namespace sim

} // namespace cauv
