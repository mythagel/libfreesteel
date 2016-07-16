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

#ifndef Ray_gen2__h
#define Ray_gen2__h
#include "bolts/S1.h"
#include "bolts/I1.h"
#include "bolts/P2.h"
#include "cages/PathXSeries.h"
#include <vector>

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
// ray holding for flat cases
class Ray_gen2
{
public:
    S1* pfib;
    std::vector<B1> scuts; // local for cutting and offsetting.

    // the endpoints of the ray are at (0,0,zrg.lo) and (0, 0, zrg.hi)

    // the disc size
    double raddisc;
    double raddiscsq;

    Ray_gen2(double lraddisc);

    // result value (a range), validity from return function.
    void HoldFibre(S1* lpfib);
    void ReleaseFibre();

    void DiscSliceCapN(const P2& a, const P2& b);
    void LineCut(const P2& a, const P2& b); // fills in the scuts

    P2 Transform(const P2& p);
};

void HackToolpath(Ray_gen2& rgen2, const PathXSeries& pathxs, std::size_t iseg, const P2& ptpath);
void HackAreaOffset(Ray_gen2& rgen2, const PathXSeries paths);

#endif

