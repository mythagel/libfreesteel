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

#ifndef AREA2_GEN
#define AREA2_GEN
#include "cages/S2weave.h"
#include "cages/PathXSeries.h"
#include "bolts/P2.h"
#include "cages/SurfX.h"
#include "cages/SurfXboxed.h"


//////////////////////////////////////////////////////////////////////
void HackAreaOffset(S2weave& wve, const PathXSeries& paths, double rad);
void HackToolpath(S2weave& wve, const PathXSeries& paths, std::size_t ixseg, const P2& ptpath, double rad);

//////////////////////////////////////////////////////////////////////
// this is a general model of a 2D area.  
class Area2_gen : public S2weave
{
public: 
    SurfXboxed* psxb;
    double z;
    double r;

    Area2_gen(const I1& urg, const I1& vrg, double res);
    Area2_gen(const I1& urg, const I1& vrg, double res, SurfXboxed* lpsxb, double lr);

    // pull the path up to tolerance
    void HackDowntoZ(float lz);
    void FindInterior(SurfX& sx);

    void MakeContours(PathXSeries& ftpaths);
};

#endif


