/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#ifndef VERTEX_H_
#define VERTEX_H_

#include <Eigen/Core>


class Vertex
{
    public:
        /*
         * OK this is the structure
         *
         *--------------------
         * vec3 position
         * float confidence
         *
         * float color (encoded as a 24-bit integer)
         * float <unused>
         * float initTime
         * float timestamp
         *
         * vec3 normal
         * float radius
         *--------------------

         * Which is three vec4s
         */
        static const int SIZE = sizeof(Eigen::Vector4f) * 3;

    private:
        Vertex(){}

};


#endif /* VERTEX_H_ */
