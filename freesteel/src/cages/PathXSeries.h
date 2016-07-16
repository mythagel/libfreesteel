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

#ifndef PathXSeries__h
#define PathXSeries__h
#include <vector>
#include "bolts/P2.h"
#include "bolts/P3.h"

//////////////////////////////////////////////////////////////////////
// a series of toolpaths
class PathXSeries		
{
public: 
    double z;

    std::vector<P2> pths;  // individual 2D points ~ the actual path
    std::vector<std::size_t> brks; // breaks - indices where pths is non-consecutive

    // 3D paths linking 2D paths at breaks
    // runs parallel to the brks array.
    std::vector< std::vector<P3> > linkpths;

    PathXSeries() {}
    PathXSeries(double lz)
        : z(lz)
    {}

    void Add(const P2& pt)
    {
        pths.push_back(pt);
    }

    void Append(const std::vector<P2>& lpths)
    {
        pths.insert(pths.end(), lpths.begin(), lpths.end());
        Break();
    }

    void Break()
    {
        brks.push_back(pths.size());
        linkpths.emplace_back();
    }

    void Pop_back()
    {
        if (brks.empty() || (pths.size() != brks.back()))
            pths.pop_back();
    }

    // accessor functions for post-processing
    std::size_t GetNbrks() const
    {
        return brks.size();
    }

    std::size_t GetBrkIndex(std::size_t i) const
    {
        return brks[i];
    }

    std::size_t GetNlnks(std::size_t j) const
    {
        return linkpths[j].size();
    }

    double GetLinkX(std::size_t j, std::size_t i) const
    {
        return linkpths[j][i].x;
    }

    double GetLinkY(std::size_t j, std::size_t i) const
    {
        return linkpths[j][i].y;
    }

    double GetLinkZ(std::size_t j, std::size_t i) const
    {
        return linkpths[j][i].z;
    }

    std::size_t GetNpts() const
    {
        return pths.size();
    }

    double GetX(std::size_t i) const
    {
        return pths[i].u;
    }
    double GetY(std::size_t i) const
    {
        return pths[i].v;
    }
}; 

#endif


