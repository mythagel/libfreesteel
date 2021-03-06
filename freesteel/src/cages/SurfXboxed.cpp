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
#include "cages/SurfXboxed.h"
#include <algorithm>
#include "pits/NormRay_gen.h"

//////////////////////////////////////////////////////////////////////
// we could do proper allocation of partitions based on density analysis.
SurfXboxed::SurfXboxed(SurfX* lpsurfx, double boxwidth)
 : psurfx(lpsurfx), gbxrg(psurfx->gxrg), gbyrg(psurfx->gyrg),
   bGeoOutLeft(false), bGeoOutUp(false), bGeoOutRight(false), bGeoOutDown(false),
   xpart(Partition1(gbxrg, boxwidth)), yparts(), buckets(),
   idups(), maxidup(0), searchbox_epsilon(1e-4)
{
    // first setup all the arrays of partitions and boxes
    yparts.reserve(xpart.NumParts());
    buckets.reserve(xpart.NumParts());
    for (std::size_t ip = 0; ip < xpart.NumParts(); ip++)
    {
        yparts.emplace_back(gbyrg, boxwidth);

        buckets.emplace_back();
        buckets.back().resize(yparts.back().NumParts());
    }

    // now poke the geometry into the boxes
    for (auto& p : psurfx->vdX)     AddPointBucket(&p);
    for (auto& edge : psurfx->edX)  AddEdgeBucket(&edge);
    for (auto& tri : psurfx->trX)   AddTriangBucket(&tri);
}

//////////////////////////////////////////////////////////////////////
// needs to deal with outlying edges 
void SurfXboxed::AddPointBucket(P3* pp) 
{
    // no problem with points in the far quartiles being marked as out by one,
    // since we must sample as out by both to get there, so would strike the condition anyway.
    if (pp->x < xpart.Getrg().lo)
        bGeoOutLeft = true;
    else if (pp->x > xpart.Getrg().hi)
        bGeoOutRight = true;
    else
    {
        ASSERT(xpart.Getrg().Contains(pp->x));
        int ix = xpart.FindPart(pp->x);

        if (pp->y < yparts[ix].Getrg().lo)
            bGeoOutDown = true;
        if (pp->y > yparts[ix].Getrg().hi)
            bGeoOutUp = true;
        else
        {
            ASSERT(yparts[ix].Getrg().Contains(pp->y));
            int iy = yparts[ix].FindPart(pp->y);
            buckets[ix][iy].ckpoints.push_back(pp);
        }
    }
}


//////////////////////////////////////////////////////////////////////
static P2 TcrossX(double lx, const P3* pp0, const P3* pp1)
{
    ASSERT(pp0->x <= pp1->x);
    P2 fp0(pp0->z, pp0->y);
    P2 fp1(pp1->z, pp1->y);
    if (lx <= pp0->x)
        return fp0;
    if (lx >= pp1->x)
        return fp1;

    double lamx = InvAlong(lx, pp0->x, pp1->x);
    return Along(lamx, fp0, fp1);
}

//////////////////////////////////////////////////////////////////////
static double TcrossY(double ly, const P2& rzl, const P2& rzr)
{
    return 0.0;
}


//////////////////////////////////////////////////////////////////////
// needs to deal with bGeoOut stuff 
void SurfXboxed::AddEdgeBucket(edgeX* ped) 
{
    // order the two endpoints in increasing x
    bool bxinc = (ped->p0->x <= ped->p1->x);
    P3* pp0 = (bxinc ? ped->p0 : ped->p1);
    P3* pp1 = (!bxinc ? ped->p0 : ped->p1);
    ASSERT(pp0 != pp1);
    ASSERT(pp0->x <= pp1->x);
    I1 xrg(pp0->x, pp1->x);

    // find the ustrips we will cross
    if (xrg.lo < gbxrg.lo)
    {
        bGeoOutLeft = true;
        xrg.lo = gbxrg.lo;
    }
    if (xrg.hi > gbxrg.hi)
    {
        bGeoOutRight = true;
        xrg.hi = gbxrg.hi;
    }
    if (xrg.lo > xrg.hi)
        return;

    // marks when we have to add duplicate counters.
    int ipfck = -1;


    // loop through the strips
    auto ixrg = xpart.FindPartRG(xrg);
    P2 rzr = TcrossX(xpart.GetPart(ixrg.first).lo, pp0, pp1);
    for (auto ix = ixrg.first; ix <= ixrg.second; ix++)
    {
        P2 rzl = rzr;
        rzr = TcrossX(xpart.GetPart(ix).hi, pp0, pp1);

        // now find the range in y we must scan through.
        I1 yrg = I1::SCombine(rzl.v, rzr.v);

        // find any crossing out of vstrips
        if (yrg.lo < gbyrg.lo)
        {
            bGeoOutDown = true;
            yrg.lo = gbyrg.lo;
        }
        if (yrg.hi > gbyrg.hi)
        {
            bGeoOutUp = true;
            yrg.hi = gbyrg.hi;
        }
        if (yrg.lo > yrg.hi)
            continue;

        // find the vcells in this ustrip.
        std::pair<int, int> iyrg = yparts[ix].FindPartRG(yrg);
        double zhu = TcrossY(yparts[ix].GetPart(iyrg.first).lo, rzl, rzr);
        for (int iy = iyrg.first; iy <= iyrg.second; iy++)
        {
            double zhd = zhu;
            zhu = TcrossY(yparts[ix].GetPart(iyrg.first).hi, rzl, rzr);

            // first cell and we cross into more than one cell.
            if (ipfck == -1)
            {
                ASSERT((ix == ixrg.first) && (iy == iyrg.first));
                if ((ixrg.first != ixrg.second) || (iyrg.first != iyrg.second))
                {
                    ipfck = idups.size();
                    idups.push_back(0);
                }
            }

            // get the zh crossing value
            double zh = std::max(zhd, zhu);

            // put in this box
            buckets[ix][iy].ckedges.emplace_back(zh, ped, ipfck);
        }
    }
}


//////////////////////////////////////////////////////////////////////
static std::pair<P2, P2> TcrossX(double lx, const P3* pp0, const P3* pp1, const P3* pp2)
{
    ASSERT((pp0->x <= pp1->x) && (pp1->x <= pp2->x));
    P2 fp0(pp0->z, pp0->y);
    P2 fp1(pp1->z, pp1->y);
    P2 fp2(pp2->z, pp2->y);
    if (lx <= pp0->x)
        return {fp0, fp0};
    if (lx >= pp2->x)
        return {fp2, fp2};

    std::pair<P2, P2> res;
    double lam02 = InvAlong(lx, pp0->x, pp2->x);
    res.first = Along(lam02, fp0, fp2);

    if (lx <= pp1->x)
    {
        double lam01 = InvAlong(lx, pp0->x, pp1->x);
        res.second = Along(lam01, fp0, fp1);
    }
    else
    {
        double lam12 = InvAlong(lx, pp1->x, pp2->x);
        res.second = Along(lam12, fp1, fp2);
    }

    return res;
}

//////////////////////////////////////////////////////////////////////
static double TcrossY(double ly, const std::pair<P2, P2>& fp)
{
	if (fp.first.v <= fp.second.v)  
	{
		if (ly <= fp.first.v) 
			return fp.first.u; // the z height. 
		if (ly >= fp.second.v) 
			return fp.second.u; 
	}
	else 
	{
		if (ly <= fp.second.v) 
			return fp.second.u; // the z height. 
		if (ly >= fp.first.v) 
			return fp.first.u; 
	}

	double lam = InvAlong(ly, fp.first.v, fp.second.v); 
	return Along(lam, fp.first.u, fp.second.u); 
}



//////////////////////////////////////////////////////////////////////
void SurfXboxed::AddTriangBucket(const triangX* ptr)
{
    // order the triangle corners by increasing x
    bool bxinc = (ptr->FirstPoint()->x <= ptr->SecondPoint()->x);
    auto pp0 = (bxinc ? ptr->FirstPoint() : ptr->SecondPoint());
    auto pp2 = (!bxinc ? ptr->FirstPoint() : ptr->SecondPoint());
    auto pp1 = ptr->ThirdPoint();
    if (pp1->x < pp0->x)
        std::swap(pp0, pp1);
    else if (pp1->x > pp2->x)
        std::swap(pp0, pp1);
    I1 xrg(pp0->x, pp2->x);
    ASSERT((pp0->x <= pp1->x) && (pp1->x <= pp2->x));

    // find the ustrips we will cross
    if (xrg.lo < gbxrg.lo)
    {
        bGeoOutLeft = true;
        xrg.lo = gbxrg.lo;
    }
    if (xrg.hi > gbxrg.hi)
    {
        bGeoOutRight = true;
        xrg.hi = gbxrg.hi;
    }
    if (xrg.lo > xrg.hi)
        return;

    // marks when we have to add duplicate counters.
    int ipfck = -1;


    // loop through the strips
    auto ixrg = xpart.FindPartRG(xrg);
    std::pair<P2, P2> fpr = TcrossX(xpart.GetPart(ixrg.first).lo, pp0, pp1, pp2);
    I1 yrgr = I1::SCombine(fpr.first.v, fpr.second.v);
    for (auto ix = ixrg.first; ix <= ixrg.second; ix++)
    {
        // copy over the spare parts of
        std::pair<P2, P2> fpl = fpr;
        fpr = TcrossX(xpart.GetPart(ixrg.first).hi, pp0, pp1, pp2);
        I1 yrgl = yrgr;
        yrgr = I1::SCombine(fpr.first.v, fpr.second.v);

        // now find the range in y we must scan through.
        ASSERT(((fpl.first.v <= fpl.second.v) == (fpl.second.v <= fpl.second.v)) || ((fpl.first.v >= fpl.second.v) == (fpl.second.v >= fpl.second.v)));
        I1 yrg(std::min(yrgl.lo, yrgr.lo), std::max(yrgl.hi, yrgr.hi));
        bool brgc1 = xpart.GetPart(ix).Contains(pp1->x);
        if (brgc1)
            yrg.Absorb(pp1->y);

        // find any crossing out of vstrips
        if (yrg.lo < gbyrg.lo)
        {
            bGeoOutDown = true;
            yrg.lo = gbyrg.lo;
        }
        if (yrg.hi > gbyrg.hi)
        {
            bGeoOutUp = true;
            yrg.hi = gbyrg.hi;
        }
        if (yrg.lo > yrg.hi)
            continue;

        // find the vcells in this ustrip.
        std::pair<int, int> iyrg = yparts[ix].FindPartRG(yrg);
        double zhu = std::max(TcrossY(yparts[ix].GetPart(iyrg.first).lo, fpl), TcrossY(yparts[ix].GetPart(iyrg.first).lo, fpr));
        for (int iy = iyrg.first; iy <= iyrg.second; iy++)
        {
            double zhd = zhu;
            double zhu = std::max(TcrossY(yparts[ix].GetPart(iyrg.first).hi, fpl), TcrossY(yparts[ix].GetPart(iyrg.first).hi, fpr));

            // get the max point of this triangle in this cell.
            double zh = std::max(zhd, zhu);
            if ((pp1->z > zh) && xpart.GetPart(ix).Contains(pp1->x))
                zh = pp1->z;

            // fill in the duplicates index if we're to cross more than one cell.
            if (ipfck == -1)
            {
                ASSERT((ix == ixrg.first) && (iy == iyrg.first));
                if ((ixrg.first != ixrg.second) || (iyrg.first != iyrg.second))
                {
                    ipfck = idups.size();
                    idups.push_back(0);
                }
            }
            buckets[ix][iy].cktriangs.emplace_back(zh, ptr, ipfck);
        }
    }
}

//////////////////////////////////////////////////////////////////////
void SurfXboxed::SortBuckets()  
{
    for (std::size_t ix = 0; ix < xpart.NumParts(); ix++)
    {
        for (std::size_t iy = 0; iy < yparts[ix].NumParts(); iy++)
        {
            bucketX& bu = buckets[ix][iy];
            std::sort(bu.ckpoints.begin(), bu.ckpoints.end(), [](const P3* a, const P3* b) { return a->z < b->z; });
            std::sort(bu.ckedges.begin(), bu.ckedges.end(), [](const ckedgeX& a, const ckedgeX& b) { return a.zh < b.zh; });
            std::sort(bu.cktriangs.begin(), bu.cktriangs.end(), [](const cktriX& a, const cktriX& b) { return a.zh < b.zh; });
        }
    }
}


//////////////////////////////////////////////////////////////////////
void SurfXboxed::SliceFibreBox(std::size_t iu, std::size_t iv, Ray_gen& rgen)
{
    bucketX& bu = buckets[iu][iv];

    for (auto& p : bu.ckpoints)
        rgen.BallSlice(*p);

    for (auto& edge : bu.ckedges)
        rgen.BallSlice(*(edge.edx->p0), *(edge.edx->p1));

    for (auto& tri : bu.cktriangs)
        rgen.BallSlice(*tri.trx->FirstPoint(), *tri.trx->SecondPoint(), *tri.trx->ThirdPoint());
}


//////////////////////////////////////////////////////////////////////
void SurfXboxed::SliceUFibre(Ray_gen& rgen)
{
    ASSERT(rgen.pfib->ftype == S1::Fibre::u);

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
    ASSERT(rgen.pfib->ftype == S1::Fibre::v);

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
