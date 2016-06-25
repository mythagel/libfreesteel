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

#ifndef SLI_GEN_H
#define SLI_GEN_H
#include "bolts/I1.h"
#include "bolts/P3.h"
#include "bolts/S1.h"
#include "bolts/P2.h"
#include "bolts/smallfuncs.h"
#include <vector>

//////////////////////////////////////////////////////////////////////
class SLi_gen
{
public:
	P3 p0;
	P3 p1;
	P3 v01n;
	P3 perp1;
	P3 perp2;
	P2 axis;

	P3 p0p; 

	// position of intersections 
    std::vector<double> inter; 

	void SetSlicePos(const P3& lp0, const P3& lp1); 

	void SliceTriangle(const P3& a, const P3& b1, const P3& b2); 

	void Convert(std::vector<I1>& res, const I1& xrg, const I1& yrg, const I1& zrg); 
};

#endif
