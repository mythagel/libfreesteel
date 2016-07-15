////////////////////////////////////////////////////////////////////////////////
// FreeSteel -- Computer Aided Manufacture Algorithms
// Copyright (C) 2004  Julian Todd and Martin Dunschen.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
// See fslicense.txt and gpl.txt for further details
////////////////////////////////////////////////////////////////////////////////
#include "pits/NormRay_gen.h"
#include "cages/SurfX.h"
#include "cages/SurfXboxed.h"


//////////////////////////////////////////////////////////////////////
void SurfXboxed::SliceFibreBox(std::size_t iu, std::size_t iv, Ray_gen& rgen)
{
	bucketX& bu = buckets[iu][iv]; 

    for (auto& p : bu.ckpoints)
        rgen.BallSlice(*p);

    for (auto& edge : bu.ckedges)
        rgen.BallSlice(*(edge.edx->p0), *(edge.edx->p1));

    for (auto& tri : bu.cktriangs)
        rgen.BallSlice(*(tri.trx->b12->p0), *(tri.trx->b12->p1), *(tri.trx->ThirdPoint()));
}


//////////////////////////////////////////////////////////////////////
void SurfXboxed::SliceUFibre(Ray_gen& rgen)  
{
	ASSERT(rgen.pfib->ftype == 1); 

	// case of drop down to the underlying surfx -- could also do if outside the region.  
	// some detection of region limits and anything on the far side of them is needed.  
	if (buckets.empty()) 
	{
		psurfx->SliceFibre(rgen); 
		return; 
	}

	// make the urange strip we scan within 
	double r = rgen.radball + searchbox_epsilon; 
	I1 urg = I1(rgen.pfib->wp - r, rgen.pfib->wp + r); 
	if (urg.Intersect(gbxrg)) 
	{
        auto iurg = xpart.FindPartRG(urg);

		// could loop in a more optimal order 
        for (auto iu = iurg.first; iu <= iurg.second; iu++)
		{
			I1 vrg = rgen.pfib->wrg.Inflate(r); 
			if (vrg.Intersect(gbyrg)) 
			{
                std::pair<int, int> ivrg = yparts[iu].FindPartRG(vrg); 

				for (int iv = ivrg.first; iv <= ivrg.second; iv++) 
					SliceFibreBox(iu, iv, rgen); 
			}
		}
	}
}


//////////////////////////////////////////////////////////////////////
void SurfXboxed::SliceVFibre(Ray_gen& rgen) 
{
	ASSERT(rgen.pfib->ftype == 2); 

	// case of drop down to the underlying surfx -- could also do if outside the region.  
	// some detection of bGeoOut region limits and anything on the far side of them is needed.  
	if (buckets.empty()) 
	{
		psurfx->SliceFibre(rgen); 
		return; 
	}

	// make the vrange strip we scan within 
	double r = rgen.radball + searchbox_epsilon; 
	I1 urg = rgen.pfib->wrg.Inflate(r); 
	if (urg.Intersect(gbxrg)) 
	{
        auto iurg = xpart.FindPartRG(urg);

		// could loop in a more optimal order 
        for (auto iu = iurg.first; iu <= iurg.second; iu++)
		{
			I1 vrg = I1(rgen.pfib->wp - r, rgen.pfib->wp + r); 
			if (vrg.Intersect(gbyrg)) 
			{
                std::pair<int, int> ivrg = yparts[iu].FindPartRG(vrg); 

				for (int iv = ivrg.first; iv <= ivrg.second; iv++) 
					SliceFibreBox(iu, iv, rgen); 
			}
		}
	}
}
