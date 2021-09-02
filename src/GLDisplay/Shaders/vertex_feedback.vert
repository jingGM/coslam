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

#version 330 core

layout (location = 0) in vec2 texcoord;

out vec4 vPosition; // [current frame position, confidence]
out vec4 vColor; // [[r,g,b,0], 0, time]
out vec4 vNormRad; // [norm vector, rad=sqrt(2)*z*f/abs(nz)]
out float zVal; // current frame depth

uniform sampler2D gSampler;
uniform sampler2D cSampler;
uniform vec4 cam; //cx, cy, 1/fx, 1/fy
uniform float cols;
uniform float rows;
uniform int time;
uniform float maxDepth;

#include "surfels.glsl"
#include "color.glsl"
#include "geometry.glsl"

void main()
{
    //Should be guaranteed to be in bounds
    float x = texcoord.x * cols;
    float y = texcoord.y * rows;

    vPosition = vec4(getVertex(texcoord.xy, x, y, cam, gSampler), 1); 
    // global position of the vertex
    
    vColor = textureLod(cSampler, texcoord.xy, 0.0);
    
    vec3 vNormLocal = getNormal(vPosition.xyz, texcoord.xy, x, y, cam, gSampler);
    // normal is the normalized cross product of depth differences in x and y directions.
    vNormRad = vec4(vNormLocal, getRadius(vPosition.z, vNormLocal.z));
    // radius = min [2sqrt(2) z (fxfy)/(fx+fy)), 2sqrt(2) z/abs(nz) (fxfy)/(fx+fy)) ]
    
    if(vPosition.z <= 0 || vPosition.z > maxDepth)
    {
	    zVal = 0;
    }
    else
    {
        zVal = vPosition.z;
    }
    
    // this is related to 1/exp(d^2) 
    vPosition.w = confidence(x, y, 1.0f);
    
    vColor.x = encodeColor(vColor.xyz); // 4 bytes = int : [x, y, z, 0]
    
    vColor.y = 0;
    //Timestamp
    vColor.w = float(time);
}
