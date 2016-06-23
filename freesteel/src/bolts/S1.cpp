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
#include "S1.h"


//////////////////////////////////////////////////////////////////////
bool S1::Check() const
{
    if (ep.size() % 2 != 0)
		return false;
    for (std::size_t i = 1; i < ep.size(); i++)
        ASSERT(ep[i - 1].w <= ep[i].w);
    for (std::size_t i = 1; i < ep.size(); i += 2)
        ASSERT(ep[i - 1].blower && !ep[i].blower);
	return true; 
}


//////////////////////////////////////////////////////////////////////
void S1::Merge(const I1& rg)  
{
	Merge(rg.lo, false, rg.hi, false);   
}

//////////////////////////////////////////////////////////////////////
void S1::Minus(const I1& rg)  
{
	Minus(rg.lo, false, rg.hi, false);   
}

//////////////////////////////////////////////////////////////////////
std::pair<std::ptrdiff_t, std::ptrdiff_t> S1::Loclohi(const I1& rg) const
{
    std::pair<std::ptrdiff_t, std::ptrdiff_t> res;
    for (res.first = 0; static_cast<std::size_t>(res.first) < ep.size(); ++res.first)
        if (ep[res.first].w >= rg.lo)
			break; 

    if (static_cast<std::size_t>(res.first) < ep.size())
	{
        for (res.second = ep.size() - 1; res.second >= res.first; --res.second)
            if (ep[res.second].w <= rg.hi)
				break; 
	}
	else 
		res.second = res.first - 1; 
		 
	#ifdef MDEBUG
	ASSERT(res.first <= res.second + 1); 

    if (static_cast<std::size_t>(res.first) < ep.size())
        ASSERT(rg.lo <= ep[res.first].w);
	if (res.first - 1 >= 0) 
        ASSERT(rg.lo > ep[res.first - 1].w);

    if ((res.second >= 0) && (static_cast<std::size_t>(res.second) < ep.size()))
        ASSERT(rg.hi >= ep[res.second].w);
    if (static_cast<std::size_t>(res.second + 1) < ep.size())
        ASSERT(rg.hi < ep[res.second + 1].w);
	#endif

	return res; 
}


//////////////////////////////////////////////////////////////////////
// we could have many speeding up cases  
void S1::Merge(double rglo, bool binterncellboundlo, double rghi, bool binterncellboundhi)  
{
    auto ilr = Loclohi(I1(rglo, rghi));
    auto il = ilr.first;
    auto ir = ilr.second;

	// off the end
    if (static_cast<std::size_t>(il) == ep.size())
	{
        ep.push_back(B1(rglo, true, binterncellboundlo));
        ep.push_back(B1(rghi, false, binterncellboundhi));
		ASSERT(Check()); 
		return; 
	}

	// no boundaries between the range.  
	if (il > ir) 
	{
        if (ep[il].blower)
		{
            ep.insert(ep.begin() + il, 2, B1(rghi, false, binterncellboundhi));
            ep[il] = B1(rglo, true, binterncellboundlo);
			ASSERT(Check()); 
		}
		return; 
	}

	// we have il to ir inclusive within the range 
    if (!ep[ir].blower)
	{
        ep[ir] = B1(rghi, false, binterncellboundhi);
		ir--; 
	}
    if (ep[il].blower)
	{
        ep[il] = B1(rglo, true, binterncellboundlo);
		il++; 
	}

	if (il <= ir) 
        ep.erase(ep.begin() + il, ep.begin() + ir + 1);
	ASSERT(Check()); 
}

//////////////////////////////////////////////////////////////////////
void S1::Minus(double rglo, bool binterncellboundlo, double rghi, bool binterncellboundhi)  
{
    auto ilr = Loclohi(I1(rglo, rghi));
    auto il = ilr.first;
    auto ir = ilr.second;

    if (static_cast<std::size_t>(il) == ep.size())
		return; 

	if (ir < il) 
	{
        if (ep[il].blower)
			return;

        ep.insert(ep.begin() + il, B1(rghi, true, binterncellboundhi));
        ep.insert(ep.begin() + il, B1(rglo, false, binterncellboundlo));
		ASSERT(Check()); 
		return; 
	}

	// we have il to ir inclusive within the range 
    if (!ep[il].blower)
	{
        ep[il] = B1(rglo, false, binterncellboundlo);
		++il;
	}
    if (ep[ir].blower)
	{
        ep[ir] = B1(rghi, true, binterncellboundlo);
		--ir;
	}

	if (il <= ir) 
        ep.erase(ep.begin() + il, ep.begin() + ir + 1);
	ASSERT(Check()); 
}


//////////////////////////////////////////////////////////////////////
bool S1::Contains(double lw) const 
{
    for (std::size_t i = 1; i < ep.size(); i += 2)
        if ((ep[i - 1].w <= lw) && (ep[i].w >= lw))
			return true; 
	return false; 
}

//////////////////////////////////////////////////////////////////////
I1 S1::ContainsRG(double lw) const 
{
    for (std::size_t i = 1; i < ep.size(); i += 2)
        if ((ep[i - 1].w <= lw) && (ep[i].w >= lw))
            return I1(ep[i - 1].w, ep[i].w);

	ASSERT(0); 
	return I1unit; 
}

//////////////////////////////////////////////////////////////////////
void S1::Invert() 
{
    if (ep.empty())
	{
        ep.push_back(B1(wrg.lo, true));
        ep.push_back(B1(wrg.hi, false));
		ASSERT(Check()); 
		return; 
	}

	// invert the flags 
    for (std::size_t i = 0; i < ep.size(); i++)
        ep[i].blower = !ep[i].blower;

	// the front condition  
    if (ep.front().w == wrg.lo)
	{
        ASSERT(!ep.front().blower);
        ep.erase(ep.begin());
	}
	else
        ep.insert(ep.begin(), B1(wrg.lo, true));

	// the back condition 
    if (ep.back().w == wrg.hi)
	{
        ASSERT(ep.back().blower);
        ep.pop_back();
	}
	else
        ep.push_back(B1(wrg.hi, false));

	ASSERT(Check()); 
}

//////////////////////////////////////////////////////////////////////
void S1::SetAllCutCodes(int lcutcode)
{
    for(auto& e : ep)
        e.cutcode = lcutcode;
}
