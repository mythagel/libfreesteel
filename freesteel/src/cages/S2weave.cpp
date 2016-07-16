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
#include "S2weave.h"
#include "bolts/I1.h"
#include "bolts/maybe.h"

//////////////////////////////////////////////////////////////////////
S2weave::S2weave(const I1& lurg, const I1& lvrg, double res)
 : urg(lurg), vrg(lvrg), ufibs(), vfibs(),
   firstcontournumber(0), lastcontournumber(firstcontournumber - 1)
{
    std::size_t nufib = urg.Leng() / res + 2;
    std::size_t nvfib = vrg.Leng() / res + 2;

    // generate the fibres
    ufibs.reserve(nufib);
    for (std::size_t i = 0; i <= nufib; i++)
        ufibs.emplace_back(urg.Along((double)i / nufib), vrg, 1);
    vfibs.reserve(nvfib);
    for (std::size_t j = 0; j <= nvfib; j++)
        vfibs.emplace_back(vrg.Along((double)j / nvfib), urg, 2);
}


//////////////////////////////////////////////////////////////////////
P2 S2weaveB1iter::GetPoint()  
{
	return (ftype == 1 ? P2(wp, w) : P2(w, wp)); 
}


//////////////////////////////////////////////////////////////////////
//optional int
static maybe<std::size_t> FindInwards(const std::vector<S1>& wfibs, double lw, bool blower, double lwp, double lwpend, bool bedge)
{
    if (wfibs.empty()) return {};

	// just do this as an inefficient loop for now.  
	if (blower)
	{
        for (std::size_t i = 0; i < wfibs.size(); i++)
		{
			if (wfibs[i].wp > lwpend) 
				break; 
			if (bedge ? (wfibs[i].wp >= lwp) : (wfibs[i].wp > lwp)) 
				if (wfibs[i].Contains(lw)) 
                    return {i};
		}
	}
	else
	{
        for (std::size_t i = wfibs.size() - 1; i >= 0; i--)
		{
			if (wfibs[i].wp < lwpend) 
				break; 
			if (bedge ? (wfibs[i].wp <= lwp) : (wfibs[i].wp < lwp)) 
				if (wfibs[i].Contains(lw)) 
                    return {i};
		}
	}

    return {};
}


//////////////////////////////////////////////////////////////////////
// this is the real code 
void S2weave::Advance(S2weaveB1iter& al)
{
	bool bedge = true; 
	double wend; 
	while (true)
	{
		
		I1 frg = (al.ftype == 1 ? ufibs : vfibs)[al.ixwp].ContainsRG(al.w); 
		wend = (al.blower ? frg.hi : frg.lo); 
        auto lixwp = FindInwards((al.ftype == 1 ? vfibs : ufibs), al.wp, al.blower, al.w, wend, bedge);

		// hit an end.  
        if (!lixwp)
			break; 

		// we always turn perpendicular
		al.w = al.wp; 
		al.ftype = (al.ftype == 1 ? 2 : 1); 
        al.ixwp = *lixwp;
		al.wp = (al.ftype == 2 ? vfibs[al.ixwp].wp : ufibs[al.ixwp].wp); 
		if (al.ftype == 1)
			al.blower = !al.blower; 

		bedge = false; 
	}

	// we've hit an endpoint, go to it and reverse 
	al.w = wend; 
	al.blower = !al.blower; 
}


//////////////////////////////////////////////////////////////////////
void S2weave::TrackContour(std::vector<P2>& pth, S2weaveB1iter al)  
{
	lastcontournumber++; 
	ASSERT(pth.empty()); 
	while (ContourNumber(al) < firstcontournumber) 
	{
		ContourNumber(al) = lastcontournumber; 
		pth.push_back(al.GetPoint()); 
		Advance(al); 
	}
	pth.push_back(al.GetPoint()); 
	ASSERT(ContourNumber(al) == lastcontournumber); 
}


//////////////////////////////////////////////////////////////////////
int& S2weave::ContourNumber(S2weaveB1iter& al)  
{
	S1& wfib = (al.ftype == 1 ? ufibs : vfibs)[al.ixwp]; 
    for (std::size_t i = (al.blower ? 0 : 1); i < wfib.ep.size(); i += 2)
        if (wfib.ep[i].w == al.w)
            return wfib.ep[i].contournumber;
	static int balls = 1; 
	ASSERT(0); 
	return balls; 
}

//////////////////////////////////////////////////////////////////////
void S2weave::SetAllCutCodes(int lcutcode)  
{
    for (auto& ufib : ufibs)
        ufib.SetAllCutCodes(lcutcode);
    for (auto& vfib : vfibs)
        vfib.SetAllCutCodes(lcutcode);
}

void S2weave::Invert()
{
    for (auto& ufib : ufibs)
        ufib.Invert();
    for (auto& vfib : vfibs)
        vfib.Invert();
}
