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

#ifndef COREROUGHGENERATION_H
#define COREROUGHGENERATION_H
#include "bolts/I1.h"
#include "cages/PathXSeries.h"
#include "cages/Area2_gen.h"
#include "pits/S2weaveCellLinearCut.h"
#include <vector>


//////////////////////////////////////////////////////////////////////
struct MachineParams
{
    // linking parameters
    double leadoffdz;
    double leadofflen;
    double leadoffrad;
    double retractzheight;
    double leadoffsamplestep;

    // cutting parameters
    double toolcornerrad;
    double toolflatrad;
    double samplestep;
    double stepdown;
    double clearcuspheight;

    // weave parameters
    double triangleweaveres;
    double flatradweaveres;

    // steering parameters
    double dchangright;
    double dchangrightoncontour;
    double dchangleft;

    double dchangefreespace;
    double sidecutdisplch;

    // post processing
    int fcut;
    int fretract;
    double thintol;
};

//////////////////////////////////////////////////////////////////////
class CoreRoughGeneration 
{
public: 
	I1 machxrg; 
	I1 machyrg; 

	PathXSeries tsbound; 

	Area2_gen* pa2gg; 	

	S2weaveCellLinearCutTraverse wc; // the tracking management.  

	// the radius of the broken circle. 
	double trad; 

	int countfreespacesteps; // used to open out the free-space spiral 

	// the boxed type here 
	PathXboxed pathxb; 


	// markers of going out points which would be good to 
	// retract back down to and continue contour machining.  
    std::vector<BCellIndex> bcellixs; 
	bool bPrevPointDoubleRange; 

	CoreRoughGeneration(PathXSeries* px, const I1& lxrg, const I1& lyrg); 

	
	void FindGoStart(); 

	bool RestartAtBCI(BCellIndex& bci, const MachineParams& params, bool bConnectAtStart); 
	double ChangeBearing(const P2& pt, const P2& tvec, const MachineParams& params); 
	void AddPoint(const P2& ppt); 
	void GrabberAlg(const struct MachineParams& params); 
	int TrackLink(const std::vector<P2>& lnk2D, S2weaveCellLinearCutTraverse wclink, bool bFromEnd, const MachineParams& params);

  void setWeave(S2weave* weave) { wc.ps2w = weave; }
}; 

#endif
