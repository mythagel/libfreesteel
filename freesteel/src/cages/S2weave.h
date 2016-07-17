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

#ifndef S2WEAVE_H
#define S2WEAVE_H
#include "bolts/P2.h"
#include "bolts/I1.h"
#include "bolts/S1.h"
#include "cages/PathXSeries.h"
#include "cages/PathXboxed.h"
#include <vector>

//////////////////////////////////////////////////////////////////////
// point location which iterates through the weave.  
struct S2weaveB1iter
{
    enum class Fibre
    {
        none,
        u,
        v
    } ftype;
    bool blower;

    double w; // in fibre
    double wp; // perpendicular to fibre.

    std::size_t ixwp; // index of fibre
    //int ixw;  // index of point in fibre

    P2 GetPoint();
}; 


//////////////////////////////////////////////////////////////////////
// this is a general model of a 2D area.  
class S2weave 
{
    // this will have subdividing capability later.
public: 
    I1 urg;
    I1 vrg;

    // the fibres
    // look forward to handling subdividing types.
    // maybe a bucket between each of the main framework.
    std::vector<S1> ufibs;
    std::vector<S1> vfibs;
    int firstcontournumber; // contour numbers less than this are counted as unvisited.
    int lastcontournumber;

    // this is where the path goes
    S2weave(const I1& urg, const I1& vrg, double res);

    // contouring type functions
    void Advance(S2weaveB1iter& al);
    int& ContourNumber(S2weaveB1iter& al);
    void TrackContour(std::vector<P2>& pth, S2weaveB1iter al);

    void SetAllCutCodes(int lcutcode);
    void Invert();
};



//////////////////////////////////////////////////////////////////////
void CircleIntersectNew(std::vector<I1>& res, const P2& cpt, double crad, const PathXSeries& bound, const PathXboxed& pathxb, double prad); 


#endif

