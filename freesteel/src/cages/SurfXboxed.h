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

#ifndef SurfXboxed__h
#define SurfXboxed__h
#include <vector>
#include "SurfX.h"
#include "bolts/P3.h"
#include "bolts/I1.h"
#include "bolts/Partition1.h"

////////////////////////////////////////////////////////////////////////////////
struct ckedgeX
{
    double zh; // highest z value in this bucket
    edgeX* edx;
    int idup; // -1 if no duplicates, otherwise points into the duplicates vector.

    ckedgeX(double lzh, edgeX* pedx, int lidup) :
        zh(lzh), edx(pedx), idup(lidup) {;}
}; 


////////////////////////////////////////////////////////////////////////////////
struct cktriX
{
    double zh; // highest z value in this bucket
    triangX* trx;
    int idup; // -1 if no duplicates, otherwise points into the duplicates vector.

    cktriX(double lzh, triangX* ptrx, int lidup) :
        zh(lzh), trx(ptrx), idup(lidup) {;}
}; 


////////////////////////////////////////////////////////////////////////////////
struct bucketX
{
    std::vector<P3*> ckpoints;
    std::vector<ckedgeX> ckedges;
    std::vector<cktriX> cktriangs;
};


//////////////////////////////////////////////////////////////////////
class SurfXboxed
{
public: 
    SurfX* psurfx; // this could be const.

    // dimensions of the buckets
    I1 gbxrg;
    I1 gbyrg;
    bool bGeoOutLeft;
    bool bGeoOutUp;
    bool bGeoOutRight;
    bool bGeoOutDown;


    Partition1 xpart;
    std::vector<Partition1> yparts;

    // raw buckets (corresponding to the partitions).
    std::vector< std::vector<bucketX> > buckets;

    // integer places where the duplicate counters are looked up.
    std::vector<int> idups;
    int maxidup;

    // used to over-extend the range of boxes we search within
    double searchbox_epsilon;

    // raw case where all we have is the surface itself.
    SurfXboxed(SurfX* lpsurfx, double boxwidth);

    void AddPointBucket(P3* pp);
    void AddEdgeBucket(edgeX* ped);
    void AddTriangBucket(triangX* ptr);
    void SortBuckets();

    // apply boxed triangles to a weave ray.
    void SliceFibreBox(std::size_t iu, std::size_t iv, class Ray_gen& rgen);
    void SliceUFibre(Ray_gen& rgen);
    void SliceVFibre(Ray_gen& rgen);
}; 


#endif



