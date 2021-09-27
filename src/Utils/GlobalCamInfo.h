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

#ifndef GLOBALCAMERAINFO_H_
#define GLOBALCAMERAINFO_H_

#include <cassert>


class GlobalCamInfo
{
    public:
        static const GlobalCamInfo & getInstance(int width=0, int height=0, float fx=0, float fy=0, float cx=0, float cy=0, int number=0);

        const int & width() const
        {
            return imgWidth;
        }

        const int & height() const
        {
            return imgHeight;
        }

        const int & cols() const
        {
            return imgWidth;
        }

        const int & rows() const
        {
            return imgHeight;
        }

        const int & numPixels() const
        {
            return imgNumPixels;
        }

        const float & fx() const
        {
            return fx_;
        }

        const float & fy() const
        {
            return fy_;
        }

        const float & cx() const
        {
            return cx_;
        }

        const float & cy() const
        {
            return cy_;
        }

        const int & camNumber() const
        {
            return number_;
        }

    private:
    GlobalCamInfo(int width, int height, float fx, float fy, float cx, float cy, int number)
         : imgWidth(width),
           imgHeight(height),
           imgNumPixels(width * height),
           fx_(fx),
           fy_(fy),
           cx_(cx),
           cy_(cy),
           number_(number)
        {
            assert(width > 0 && height > 0 && number>0 && "You haven't initialised the GlobalCamInfo class!");
        }

        const int imgWidth, imgHeight, imgNumPixels, number_;
        const float fx_, fy_, cx_, cy_;
};

#endif /* GLOBALCAMERAINFO_H_ */
