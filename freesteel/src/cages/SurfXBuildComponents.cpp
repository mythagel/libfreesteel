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
#include "cages/SurfX.h"
#include <algorithm>


//////////////////////////////////////////////////////////////////////
SurfXBuilder::SurfXBuilder(const I1& lgxrg, const I1& lgyrg, const I1& lgzrg)
 : gxrg(lgxrg), gyrg(lgyrg), gzrg(lgzrg), rangestate(RangeState::hardset)
{}


//////////////////////////////////////////////////////////////////////
SurfXBuilder::SurfXBuilder()
 : rangestate(RangeState::none)
{}

//////////////////////////////////////////////////////////////////////
void SurfXBuilder::PushTriangle(const P3& p0, const P3& p1, const P3& p2)
{
    // does the triangle touch the region?
    // this condition overestimates it.
    // we can have triangles that go diagonally but miss it.
    if (rangestate == RangeState::hardset)
    {
        if ((p0.x < gxrg.lo) && (p1.x < gxrg.lo) && (p2.x < gxrg.lo))
            return;
        if ((p0.x > gxrg.hi) && (p1.x > gxrg.hi) && (p2.x > gxrg.hi))
            return;
        if ((p0.y < gyrg.lo) && (p1.y < gyrg.lo) && (p2.y < gyrg.lo))
            return;
        if ((p0.y > gyrg.hi) && (p1.y > gyrg.hi) && (p2.y > gyrg.hi))
            return;
        if ((p0.z < gzrg.lo) && (p1.z < gzrg.lo) && (p2.z < gzrg.lo))
            return;
        if ((p0.z > gzrg.hi) && (p1.z > gzrg.hi) && (p2.z > gzrg.hi))
            return;
    }
    else
    {
        ASSERT(EqualOr(rangestate, RangeState::none, RangeState::adsorbing));
        bool bFirst = (rangestate == RangeState::none);
        rangestate = RangeState::adsorbing;
        gxrg.Absorb(p0.x, bFirst);
        gyrg.Absorb(p0.y, bFirst);
        gzrg.Absorb(p0.z, bFirst);
        gxrg.Absorb(p1.x, bFirst);
        gyrg.Absorb(p1.y, bFirst);
        gzrg.Absorb(p1.z, bFirst);
        gxrg.Absorb(p2.x, bFirst);
        gyrg.Absorb(p2.y, bFirst);
        gzrg.Absorb(p2.z, bFirst);
    }

    // push the triangle in.
    lvd.reserve(lvd.size() + 3);
    lvd.push_back(p0);
    lvd.push_back(p1);
    lvd.push_back(p2);
}


//////////////////////////////////////////////////////////////////////
struct p3X_order
{
    bool operator()(const P3* a, const P3* b)
        { return ((a->x < b->x) || ((a->x == b->x) && ((a->y < b->y) || ((a->y == b->y) && (a->z < b->z))))); }
};


//////////////////////////////////////////////////////////////////////
struct triangXr
{
	P3* a; 
	P3* b1; 
	P3* b2; 
	
	P3 tnorm;	// displacement of the triangle to hit the tool

	triangXr(P3& p0, P3& p1, P3& p2); 
}; 


/////////////////////////////////////////////////////////////////////
triangXr::triangXr(P3& p0, P3& p1, P3& p2)
 : a(&p0), b1(&p1), b2(&p2), tnorm()
{
    using std::swap;

    auto less = p3X_order();

    if (!less(a, b1))
		swap(a, b1);
    if (!less(b1, b2))
		swap(a, b2);

	P3 v1 = *b1 - *a;
    P3 v2 = *b2 - *a;

    P3 ncross = -P3::CrossProd(v1, v2);
    double fac = 1.0;
	if (ncross.z < 0.0)
	{
        swap(b1, b2);
		fac = -1.0;
	}
    double nclen = ncross.Len();
	if (nclen != 0.0)
        fac = fac / nclen;
    tnorm = ncross * fac;
}




/////////////////////////////////////////////////////////////////////
struct edgeXr
{
	P3* p0; 
	P3* p1; 
	int itR; // face index 
	int itL; 

	edgeXr(P3* lp0, P3* lp1, int it); 
};

//////////////////////////////////////////////////////////////////////
edgeXr::edgeXr(P3* lp0, P3* lp1, int it)  
{
	if (p3X_order()(lp0, lp1))
	{
		itR = it; 
		itL = -1; 
		p0 = lp0; 
		p1 = lp1; 
	}
	else 
	{
		itR = -1; 
		itL = it; 
		p0 = lp1; 
		p1 = lp0; 
	}
}


//////////////////////////////////////////////////////////////////////
struct edgeXr_order
{
	bool operator()(const edgeXr* a, const edgeXr* b)
        { return ((a->p0 < b->p0) || ((a->p0 == b->p0) && ((a->p1 < b->p1) || ((a->p1 == b->p1) && (a->itR < b->itR))))); }
};

//////////////////////////////////////////////////////////////////////

void SetEdge(triangX& trX, edgeX* pe, triangXr& r)
{
    if (((r.a == pe->p0) && (r.b1 == pe->p1)) || ((r.a == pe->p1) && (r.b1 == pe->p0)))
    {
        ASSERT(trX.ab1 == NULL);
        trX.ab1 = pe;
    }
    else if (((r.a == pe->p0) && (r.b2 == pe->p1)) || ((r.a == pe->p1) && (r.b2 == pe->p0)))
    {
        ASSERT(trX.ab2 == NULL);
        trX.ab2 = pe;
    }
    else if (((r.b1 == pe->p0) && (r.b2 == pe->p1)) || ((r.b1 == pe->p1) && (r.b2 == pe->p0)))
    {
        ASSERT(trX.b12 == NULL);
        trX.b12 = pe;
    }
    else
        ASSERT(0);
}

//////////////////////////////////////////////////////////////////////
// this is the tooldef specific part  
SurfX SurfXBuilder::Build() const
{
    SurfX sx;
    sx.gxrg = gxrg;
    sx.gyrg = gyrg;
    sx.gzrg = gzrg;
    auto& vdX = sx.vdX;
    auto& edX = sx.edX;
    auto& trX = sx.trX;

    auto np = lvd.size(); // 3 times the number of triangles.

    std::vector<std::size_t> ltd;
    {
        // first sort all the points by increasing x
        std::vector<const P3*> p3X;
        p3X.reserve(np);
        for (auto& p : lvd) p3X.push_back(&p);
        std::sort(p3X.begin(), p3X.end(), p3X_order());

        // make the indexes into this array with duplicates removed
        ltd.resize(np);
        vdX.reserve(np);    // optimistic
        for (auto& pi : p3X)
        {
            if (vdX.empty() || !(vdX.back() == *pi))
                vdX.push_back(*pi);
            ltd[pi - &(lvd[0])] = vdX.size() - 1;
        }
    }

    // build up oriented triangles with normals and remove degenerate ones
    auto nt = np / 3;
    std::vector<triangXr> ttx;
    ttx.reserve(nt);
    for (std::size_t i = 0; i < nt; ++i)
        ttx.emplace_back(vdX[ltd[i * 3]], vdX[ltd[i * 3 + 1]], vdX[ltd[i * 3 + 2]]);

    // now make the array of linked edges
    std::vector<edgeXr> edXr;
    edXr.reserve(ttx.size());
    for (std::size_t i = 0; i < ttx.size(); ++i)
    {
        auto& tri = ttx[i];
        edXr.emplace_back(tri.a, tri.b1, i);
        edXr.emplace_back(tri.b1, tri.b2, i);
        edXr.emplace_back(tri.b2, tri.a, i);
    }

    std::vector<edgeXr*> pedXr;
    pedXr.reserve(edXr.size());
    for (auto& edge : edXr) pedXr.push_back(&edge);
    std::sort(pedXr.begin(), pedXr.end(), edgeXr_order());

    // build the final array of triangles into which the edges will point
    trX.reserve(ttx.size());
    for (auto& tri : ttx) trX.emplace_back(tri.tnorm);

    // build the final array of edges with pointers into these triangles
    edX.reserve(pedXr.size());
    for (std::size_t i = 0; i < pedXr.size(); )
    {
        // two edges can fuse into one with triangles on both sides
        if ((i + 1 < pedXr.size()) && (pedXr[i]->p0 == pedXr[i + 1]->p0) && (pedXr[i]->p1 == pedXr[i + 1]->p1) && ((pedXr[i]->itL == -1) != (pedXr[i + 1]->itL == -1)))
        {
            if (pedXr[i]->itL == -1)
                edX.emplace_back(pedXr[i]->p0, pedXr[i]->p1, &(trX[pedXr[i]->itR]), &(trX[pedXr[i + 1]->itL]));
            else
                edX.emplace_back(pedXr[i]->p0, pedXr[i]->p1, &(trX[pedXr[i + 1]->itR]), &(trX[pedXr[i]->itL]));
            i += 2;
        }
        else // one triangle sided edge
        {
            edX.emplace_back(pedXr[i]->p0, pedXr[i]->p1, (pedXr[i]->itR != -1 ? &(trX[pedXr[i]->itR]) : NULL), (pedXr[i]->itL != -1 ? &(trX[pedXr[i]->itL]) : NULL));
            i++;
        }
    }

    // put the backpointers to the edges into the triangles
    for (auto& edge : edX)
    {
        if (edge.tpL != NULL)
            SetEdge(*edge.tpL, &edge, ttx[edge.tpL - &(trX[0])]);
        if (edge.tpR != NULL)
            SetEdge(*edge.tpR, &edge, ttx[edge.tpR - &(trX[0])]);
    }

    return sx;
}

void SurfXBuilder::Reset()
{
    // kill the old arrays
    lvd.clear();
}
