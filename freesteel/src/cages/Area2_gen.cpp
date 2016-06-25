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
#include "cages/Area2_gen.h"
#include "pits/NormRay_gen.h"
#include "cages/Ray_gen2.h"

//////////////////////////////////////////////////////////////////////
void Area2_gen::FindInterior(SurfX& sx)
{
    SLi_gen sgen;
    std::vector<I1> res;

    for (auto& ufib : ufibs)
    {
        sgen.SetSlicePos(P3(ufib.wp, vrg.lo, z), P3(ufib.wp, vrg.hi, z));
        sx.SliceRay(sgen);
        sgen.Convert(res, urg, vrg, sx.gzrg);

        while (!res.empty())
        {
            ufib.Merge(res.back());
            res.pop_back();
        }
    }

    for (auto& vfib : vfibs)
    {
        sgen.SetSlicePos(P3(urg.lo, vfib.wp, z), P3(urg.hi, vfib.wp, z));
        sx.SliceRay(sgen);
        sgen.Convert(res, urg, urg, sx.gzrg);

        while (!res.empty())
        {
            vfib.Merge(res.back());
            res.pop_back();
        }
    }
}



//////////////////////////////////////////////////////////////////////
void Area2_gen::SetSurfaceTop(SurfXboxed* lpsxb, double lr)
{
    r = lr;
    psxb = lpsxb;
    z = psxb->psurfx->gzrg.hi;
}


//////////////////////////////////////////////////////////////////////
void Area2_gen::HackDowntoZ(float lz)
{
    ASSERT(lz <= z);
    z = lz;

    Ray_gen uryg(r, vrg);
    for (auto& ufib : ufibs)
    {
        uryg.HoldFibre(&ufib, z);
        psxb->SliceUFibre(uryg);
    }

    Ray_gen vryg(r, urg);
    for (auto& vfib : vfibs)
    {
        vryg.HoldFibre(&vfib, z);
        psxb->SliceVFibre(vryg);
    }
}


//////////////////////////////////////////////////////////////////////
void Area2_gen::MakeContours(PathXSeries& ftpaths)
{
    firstcontournumber = lastcontournumber + 1;

    S2weaveB1iter alscan;
    alscan.ftype = 1;
    for (alscan.ixwp = 0; alscan.ixwp < ufibs.size(); alscan.ixwp++)
    {
        alscan.wp = ufibs[alscan.ixwp].wp;
        for (std::size_t i = 0; i < ufibs[alscan.ixwp].ep.size(); i++)
        {
            alscan.w = ufibs[alscan.ixwp].ep[i].w;
            alscan.blower = ufibs[alscan.ixwp].ep[i].blower;

            if (ContourNumber(alscan) < firstcontournumber)
            {
                std::vector<P2> contour;
                TrackContour(contour, alscan);
                ftpaths.Append(contour);
                ftpaths.z = z;
            }
        }
    }
}


//////////////////////////////////////////////////////////////////////
void HackToolpath(S2weave& wve, const PathXSeries& pathxs, std::size_t iseg, const P2& ptpath, double rad)
{
    Ray_gen2 ryg2(rad);

    for (auto& ufib : wve.ufibs)
    {
        ryg2.HoldFibre(&ufib);
        HackToolpath(ryg2, pathxs, iseg, ptpath);
        ryg2.ReleaseFibre();
    }

    for (auto& vfib : wve.vfibs)
    {
        ryg2.HoldFibre(&vfib);
        HackToolpath(ryg2, pathxs, iseg, ptpath);
        ryg2.ReleaseFibre();
    }
}


//////////////////////////////////////////////////////////////////////
void HackAreaOffset(S2weave& wve, const PathXSeries& paths, double rad)
{
    Ray_gen2 ryg2(rad);

    for (auto& ufib : wve.ufibs)
    {
        ryg2.HoldFibre(&ufib);
        HackAreaOffset(ryg2, paths);
        ryg2.ReleaseFibre();
    }

    for (auto& vfib : wve.vfibs)
    {
        ryg2.HoldFibre(&vfib);
        HackAreaOffset(ryg2, paths);
        ryg2.ReleaseFibre();
    }
}
