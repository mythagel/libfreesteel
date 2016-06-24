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
#ifndef VO__H
#define VO__H
#include <vector>

template<class V>
struct vo
{
    V* v;
    vo(bool bInit = true)
     : v(bInit ? V::New() : NULL)
    {}
    ~vo() { if (v != NULL)  v->Delete(); }
    V* operator->() { return v; }
    V* operator&() { return v; }
    void Renew()
    {
        if (v != NULL) v->Delete();
        v = V::New();
    }
}; 

template<class V>
struct vectorvo 
{
    std::vector<V*> vv;

    V* operator[](std::size_t i) { return vv[i]; }

    void resize(int n)
    {
        ASSERT(vv.empty());
        vv.resize(n);
        for (auto& v : vv) vv = V::New();
    }
    ~vectorvo()
    {
        for (auto& v : vv) v->Delete();
        vv.clear();
    }
}; 

#endif
