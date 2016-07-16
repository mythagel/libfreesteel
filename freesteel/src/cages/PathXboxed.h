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

#ifndef PathXboxed__h
#define PathXboxed__h
#include "PathXSeries.h"
#include <vector>
#include "bolts/I1.h"
#include "bolts/P2.h"
#include "bolts/Partition1.h"

//////////////////////////////////////////////////////////////////////
struct ckpline 
{
    std::size_t iseg;
    mutable std::ptrdiff_t idup;
    double vmid; // the vrange is (vmid - vrad, vmid + vrad)
    double vrad;

    ckpline(std::size_t liseg, std::ptrdiff_t lidup, double lvmid, double lvrad)
        : iseg(liseg), idup(lidup), vmid(lvmid), vrad(lvrad)
    {}
};

//////////////////////////////////////////////////////////////////////
struct pucketX
{
    std::vector<std::size_t> ckpoints;
    std::vector<ckpline> cklines;
};

//////////////////////////////////////////////////////////////////////
class PathXboxed
{
public: 
    PathXSeries* ppathx;

    I1 gburg;
    bool bGeoOutLeft;
    bool bGeoOutRight;

    Partition1 upart;

    // simple buckets running parallel to the partitions.
    std::vector<pucketX> puckets;

    // integer places where the duplicate counters are looked up.
    std::vector<int> idups;
    mutable int maxidup;

    PathXboxed(PathXSeries* lppathx, const I1& lgburg, double boxwidth);

    void PutSegment(std::size_t iseg, bool bFirst, bool bRemove);
    void Add(const P2& p1);
    void Break();
    void Pop_back();
};

#endif


