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

#ifndef S1_H
#define S1_H
#include <vector>
#include <utility>
#include "I1.h"

//////////////////////////////////////////////////////////////////////
// an endpoint in the fibre.  
struct B1
{
    double w;

    bool blower;
    bool binterncellbound;
    int contournumber;

    int cutcode;

    B1(double lw, bool lblower, bool lbinterncellbound = false)
        : w(lw), blower(lblower), binterncellbound(lbinterncellbound), contournumber(-1), cutcode()
    {}

    bool operator<(const B1& b) const { return w < b.w; }
}; 

//////////////////////////////////////////////////////////////////////
// this is a fibre 
struct S1
{
    std::vector<B1> ep;
    double wp; // the perpendicular position.
    I1 wrg;

    enum class Fibre
    {
        none,
        u,
        v,

        circ
    } ftype; // 1 for ufibre, 2 for vfibre

    void Merge(const I1& rg);
    void Merge(double rglo, bool binterncellboundlo, double rghi, bool binterncellboundhi);
    void Minus(const I1& rg);
    void Minus(double rglo, bool binterncellboundlo, double rghi, bool binterncellboundhi);

    std::pair<std::ptrdiff_t, std::ptrdiff_t> Loclohi(const I1& rg) const;

    void Invert();

    bool Check() const;

    bool Contains(double lw) const;
    I1 ContainsRG(double lw) const;

    void SetNew(double lwp, const I1& lwrg, Fibre lftype)
    {
        wp = lwp;
        wrg = lwrg;
        ftype = lftype;
        ep.clear();
    }

    S1()
        : ep(), wp(), wrg(I1unit), ftype()
    {}
    S1(double lwp, I1& lwrg, Fibre lftype)
        : ep(), wp(lwp), wrg(lwrg), ftype(lftype)
    {}

    void SetAllCutCodes(int lcutcode);
}; 


#endif 

