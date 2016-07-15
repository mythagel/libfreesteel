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

#ifndef SurfX__h
#define SurfX__h
#include <vector>
#include "bolts/P3.h"
#include "bolts/I1.h"
#include "pits/SLi_gen.h"

struct triangX;

/////////////////////////////////////////////////////////////////////
struct edgeX
{
    P3* p0;
    P3* p1;
    triangX* tpR; // face index
    triangX* tpL;

    // the dot product with the 01-perp vector
    // where we hit the triangle on right and left
    double rdR;
    double rdL;

    edgeX(P3* lp0, P3* lp1, triangX* ltpR, triangX* ltpL)
        : p0(lp0), p1(lp1), tpR(ltpR), tpL(ltpL), rdR(), rdL()
    {}
};

//////////////////////////////////////////////////////////////////////
struct triangX // : public Facet
{
    edgeX* ab1;
    edgeX* ab2;
    edgeX* b12;

    P3 tnorm;	// displacement of the triangle to hit the tool
    double tp;  // plane of the triangle

    triangX(const P3& ltnorm)
        : ab1(), ab2(), b12(), tnorm(ltnorm)
    {}

    void SetEdge(edgeX* pe, struct triangXr& r);

    inline P3* FirstPoint() { return b12->p0; }
    inline P3* SecondPoint() { return b12->p1; }
    P3* ThirdPoint(edgeX* pe);
    P3* ThirdPoint();
};

//////////////////////////////////////////////////////////////////////
// this type of surface is highly boxed and several can be used in strips 
// to cover the space.  
// we are making a piece of the offset surface.  
class SurfX		// extra special made surface for a tool
{
public:

    // vectors components
    std::vector<P3> vdX;
    std::vector<edgeX> edX;
    std::vector<triangX> trX;

    I1 gxrg, gyrg, gzrg;

    // new horizontal slicing code
    void SliceFibre(class Ray_gen& rgen);
    void SliceRay(class SLi_gen& sgen);
};

class SurfXBuilder
{
private:
    // we pull in all geometry which touches these ranges
    // triangles may extend far beyond these ranges, so be careful.
    // this is why the input into this class is done by triangles
    // so that ones which do not span this region can be ignored.
    I1 gxrg, gyrg, gzrg;
    enum class RangeState
    {
        none,
        adsorbing,
        hardset
    } rangestate;

    // these are the arrays we initially load the points and the triangles
    // as triples of points, from which we then work.
    // they are erased after the components have been built.
    std::vector<P3> lvd;
public:
    SurfXBuilder(const I1& lgxrg, const I1& lgyrg, const I1& lgzrg);
    SurfXBuilder(); // one that builds the ranges

    void PushTriangle(const P3& p0, const P3& p1, const P3& p2);

    SurfX Build();
};

#endif
