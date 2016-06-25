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

#ifndef S2WEAVECELL_H
#define S2WEAVECELL_H
#include "bolts/S1.h"
#include "bolts/I1.h"
#include "bolts/P2.h"
#include <vector>
#include <utility>


//////////////////////////////////////////////////////////////////////
struct S2weaveCell
{
	class S2weave* ps2w; 

	// indexes for finding the cell.  
	// With subdivision the indexing will be more complex.  
    std::size_t iu;
    std::size_t iv;

	// cell boundary pointers (which get changed by splitting)
	const S1* pfulo; 
	const S1* pfuhi; 
	const S1* pfvlo; 
	const S1* pfvhi; 

	// further info about where the cell corners lie in the boundaries.  
	// cell bounds (for ease of viewing).  
	I1 clurg; 
	I1 clvrg; 

	// it goes bottom left, top left, top right, bottom right.  
	P2 GetCorner(int icn) const; 
	const S1* GetSide(int icn) const; // gets the following edge from the corner.  

    // the list of endpoints of fibres in this cell.
    // first int is the edge (left is first), second is the B1 entry in the array.
    std::vector< std::pair<std::size_t, B1*> > boundlist;
    bool bLDin;
    bool bLUin;
    bool bRUin;
    bool bRDin;

	// index into boundlist which marks connects the 
	// points between the boundary with lines, resolving ambiguities.  
	// not possible to control for order.  
    std::vector< std::pair<std::size_t, std::size_t> > bolistpairs;


    P2 GetBoundPoint(std::size_t ibl);
    bool GetBoundLower(std::size_t ibl);


	// changing and construction functions 
		void ConstructCellBounds(); 
        std::size_t CreateBoundList();
	void FindCellIndex(const P2& lptc); 
	void AdvanceCrossSide(int icn, const P2& cspt); 


    int GetBoundListPosition(std::size_t sic, const P2& ptb, bool bOnBoundOutside);
};



#endif

