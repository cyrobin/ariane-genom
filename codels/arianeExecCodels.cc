/**
 ** arianeExecCodels.cc
 **
 ** Codels called by execution task arianeExec
 **
 ** Author: cyril.robin@laas.fr
 ** Date: Fri Oct 04 2013
 **
 **/

#include <portLib.h>
#include <posterLib.h>

#include <ostream>
#include <sstream>
#include <cstdlib>
#include <limits>
#include <sys/time.h>
#include "boost/format.hpp"

#include "server/arianeHeader.h"
#include "gladys/weight_map.hpp"
#include "gdalwrap/gdal.hpp"
#include "ariane/ariane.hpp"

//#include <dtmStruct.h>

/*------------------------------------------------------------------------
 * Local structures and functions
 */
static inline
gladys::point_xy_t to_gladys_point(GEN_POINT_2D p) {//{{{
  gladys::point_xy_t res;
  res[0] = p.x ;
  res[1] = p.y ;

  return res;
}//}}}

static inline
GENPOS_CART_CONFIG from_gladys_point( gladys::point_xy_t p ) {//{{{
  GENPOS_CART_CONFIG res;
  res.x = p[0] ;
  res.y = p[1] ;
  res.theta = 0;

  return res;
}//}}}

void fill_gdal(gdalwrap::raster& gdal, const DTM_LABEL_POSTER* poster){//{{{
	int nbLines = poster->nbLines;
	int nbCols = poster->nbCols;

	for (size_t i = 0; i < nbLines; ++i) 
		for (size_t j = 0; j < nbCols; ++j)
		{
			switch (poster->state[i][j]) {
				case DTM_NO_LABEL:
					//gdal[i * nbCols + j] = 100.0;
					gdal[i + nbLines * j] = 100.0;
					break;
				case DTM_LABEL_TRAVERSABLE:
					//gdal[i * nbCols + j] = 3.14;
					gdal[i + nbLines * j] = 3.14;
					break;
				case DTM_LABEL_OBSTACLE:
					//gdal[i * nbCols + j] = std::numeric_limits<float>::infinity();
					gdal[i + nbLines * j] = std::numeric_limits<float>::infinity();
					break;
			}
		}
}//}}}

/*------------------------------------------------------------------------
 * Local variables
 */
// poster IDs that are initialized with PosterFind requests
static POSTER_ID robotPos_PosterID ;
//static POSTER_ID teammatePos_PosterID	;
static POSTER_ID dtm_PosterID ;

gladys::weight_map gwm ; // global weight map
ariane::ariane *ariadne  = NULL;
int dump_cnt = 0;


/*------------------------------------------------------------------------
 * Init
 *
 * Description: 
 *
 * Reports:      OK
 *              S_ariane_CANNOT_CONNECT
 *              S_ariane_FAILED_CREATION
 */

/* arianeInitMain  -  codel EXEC of Init
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
arianeInitMain(arianeInitParams *initParams, int *report)
{//{{{
  struct timeval tv0, tv1;

  /* Connect to Posters (POM) */
  // Robot pom Pos
  if (posterFind(initParams->pomPosterName, &robotPos_PosterID) == ERROR) {
    std::cerr << "#EEE# ariane : cannot find pom poster : " 
              << initParams->pomPosterName << std::endl;
    (*report) = S_ariane_CANNOT_CONNECT ;
    return ETHER;
  }

  // dtm label poster
  if (posterFind(initParams->dtmPosterName, &dtm_PosterID) == ERROR) {
    std::cerr << "#EEE# ariane : cannot find dtm poster : " 
              << initParams->dtmPosterName << std::endl;
    (*report) = S_ariane_CANNOT_CONNECT ;
    return ETHER;
  }

  std::cerr << "[ariane] Init -- posters found." << std::endl;

  // load the first global weight map
  gwm.load( initParams->f_region, initParams->f_robot );

  // Set internal parameters
  SDI_F->internalParams.max_step_length = initParams->max_step_length ;
  SDI_F->internalParams.curb_tolerance  = initParams->curb_tolerance ;

  SDI_F->dump = GEN_FALSE;

  strncpy(SDI_F->logDir,initParams->logDir,ariane_MAX_LENGTH) ;

  // Poster Init
  memset(&SDI_F->path, 0, sizeof(SDI_F->path));

  /* ariane init (load nav_graph) */
  gettimeofday(&tv0, NULL);
  ariadne = new ariane::ariane ( gwm,
         SDI_F->internalParams.max_step_length,
         SDI_F->internalParams.curb_tolerance);
  gettimeofday(&tv1, NULL);

  std::cerr << "[ariane] planner loaded ("
            << (tv1.tv_sec -  tv0.tv_sec) * 1000 +
            (tv1.tv_usec - tv0.tv_usec) / 1000 << " ms)." << std::endl;

  std::cerr << "[ariane] Init done." << std::endl;

  //end
  (*report) = OK ;
  return ETHER;
}///}}}

/*------------------------------------------------------------------------
 * FindPath
 *
 * Description: 
 *
 * Reports:      OK
 *              S_ariane_UNEXPECTED_DATA
 *              S_ariane_CANNOT_READ_POSTER
 *              S_ariane_CANNOT_UPDATE_POSTER
 */

/* arianeFindPathMain  -  codel EXEC of FindPath
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
arianeFindPathMain(GEN_POINT_2D *_goal, int *report)
{//{{{
  struct timeval tv0, tv1;

  std::cerr << "[ariane] Init -- read posters." << std::endl;

  /* Init */
  // Read pom posters
  POM_POS robotPos;
  if ( posterRead( robotPos_PosterID, 0, &robotPos, sizeof(POM_POS) ) == ERROR) {
    std::cerr << "#EEE# ariane : can not read pom poster." << std::endl;
    (*report) = S_ariane_CANNOT_READ_POSTER;
    return ETHER;
  }

  /* Transform POM_POS GENOM_POINT into gladys::point_xy_t */
  gladys::point_xy_t start, goal ;
  start[0] = robotPos.mainToOrigin.euler.x ;
  start[1] = robotPos.mainToOrigin.euler.y ;
  goal[0] = _goal->x ;
  goal[1] = _goal->y ;

  std::cerr << "[ariane] start (" << start[0] << "," << start[1] 
            << ") and goal (" << goal[0] << "," << goal[1] 
            << ")." << std::endl;

  if ( SDI_F->dump ) {//{{{
    // open dump file
    std::ostringstream oss, oss2;
    oss << SDI_F->logDir << "dump-ariane-";
    oss << boost::format("%04s") % dump_cnt << ".log";
    std::ofstream dump_file( oss.str() );

    // dump r_pos
    dump_file   << "start_and_goal "
                << start[0] << " " << start[1] << " "
                << goal[0]  << " " << goal[1]
                << std::endl;
    // dump internal params
    dump_file << "internal_params "
              << SDI_F->internalParams.max_step_length << " "
              << SDI_F->internalParams.curb_tolerance << " "
              << std::endl;

    // dump weight_map
    oss2 << SDI_F->logDir << "dump-ariane-" ;
    oss2 << boost::format("%04s") % dump_cnt << "-weight-map.tif";
    gwm.save( oss2.str() );
    dump_file << oss2.str() << std::endl;

    // close file
    dump_file.close();
    dump_cnt++;
  }//}}}

  /* Plan */
  gettimeofday(&tv0, NULL);
  gladys::path_t path = ariadne->plan( start, goal );
  gettimeofday(&tv1, NULL);

  // handle errors in ariane
  if ( path.empty() ) {
    std::cerr << "#EEE# ariane : no path found :'( " << std::endl;
    (*report) = S_ariane_NO_PATH;
    return ETHER;
  }

  // From here, there should be a valid plan
  std::cerr << "[ariane] Waypoints  computed (" 
            << (tv1.tv_sec -  tv0.tv_sec) * 1000 +
            (tv1.tv_usec - tv0.tv_usec) / 1000 << " ms)" << std::endl;

  /* and Post */
  gladys::path_t::const_iterator it;
  SDI_F->path.nbPts = 0;

  for ( it = path.begin();
        it != path.end() and SDI_F->path.nbPts < GENPOS_MAX_TRAJECTORY_SIZE ;
        ++it)
  {
    SDI_F->path.points[SDI_F->path.nbPts] = from_gladys_point( *it );
    SDI_F->path.nbPts++;
    std::cerr   << "[ariane] ----waypoint #" << SDI_F->path.nbPts++ 
                << " = (" << (*it)[0]<< "," << (*it)[1] 
                <<")"<<std::endl;
  }

  SDI_F->path.numRef++;

  return ETHER;
}//}}}

/*------------------------------------------------------------------------
 * UpdateMap
 *
 * Description: 
 *
 * Reports:      OK
 *              S_ariane_CANNOT_READ_POSTER
 *              S_ariane_CANNOT_UPDATE_NAV_GRAPH
 */

/* arianeUpdateMaphMain  -  codel EXEC of UpdateMap
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
arianeUpdateMapMain(int *report)
{//{{{
  struct timeval tv0, tv1;

  // read dtm poster ; link the dtm with the weight map
  const DTM_LABEL_POSTER* poster = (const DTM_LABEL_POSTER*)posterAddr(dtm_PosterID);
  posterTake(dtm_PosterID, POSTER_READ);

  gladys::weight_map wm;
  gdalwrap::raster& gdal = wm.setup_weight_band(poster->nbLines, poster->nbCols);

  std::cerr << "[ariane] (x0,y0,xS,yS) = ("
            << poster->xOrigin << ","
            << poster->yOrigin << ","
            << poster->xScale << ","
            << poster->yScale << ")"
            << std::endl;
  wm.get_map().set_transform(  poster->xOrigin,
                               poster->yOrigin,
                               poster->xScale,
                               poster->yScale );

  // Update the weight map with the dtm label poster
  gettimeofday(&tv0, NULL);
  fill_gdal(gdal, poster);
  gettimeofday(&tv1, NULL);

  posterGive(dtm_PosterID);

  std::cerr << "[ariane] dtm data loaded (" 
            << (tv1.tv_sec -  tv0.tv_sec) * 1000 +
            (tv1.tv_usec - tv0.tv_usec) / 1000 << " ms)." << std::endl;

  // merge the dtm weight map with the global weight_map
  gettimeofday(&tv0, NULL);
  gwm.merge( wm );
  gettimeofday(&tv1, NULL);
  std::cerr << "[ariane] weight map merged ("
            << (tv1.tv_sec -  tv0.tv_sec) * 1000 +
            (tv1.tv_usec - tv0.tv_usec) / 1000 << " ms)." << std::endl;

  // update the ariane instance
  gettimeofday(&tv0, NULL);
  delete ariadne ;
  ariadne = new ariane::ariane ( gwm,
         SDI_F->internalParams.max_step_length,
         SDI_F->internalParams.curb_tolerance);
  gettimeofday(&tv1, NULL);

  std::cerr << "[ariane] planner loaded ("
            << (tv1.tv_sec -  tv0.tv_sec) * 1000 +
            (tv1.tv_usec - tv0.tv_usec) / 1000 << " ms)." << std::endl;

  return ETHER;
}//}}}


