/**
 ** arianeCntrlTaskCodels.cc
 **
 ** Codels used by the control task arianeCntrlTask
 **
 ** Author: cyril.robin@laas.fr
 ** Date: Fri Oct 04 2013
 **
 **/

#include <portLib.h>

#include "server/arianeHeader.h"


/*------------------------------------------------------------------------
 * arianeSetParametersCtrl  -  control codel of CONTROL request SetParameters
 *
 * Description:    
 * Report: OK
 *
 * Returns:    OK or ERROR
 */

STATUS
arianeSetParametersCtrl(arianeInternalParams *internalParams, int *report)
{
  /* nothing to do here */
  //TODO check consistency
  return OK;
}

/*------------------------------------------------------------------------
 * arianeEnableDumpCtrl  -  control codel of CONTROL request EnableDump
 *
 * Description:    
 * Report: OK
 *
 * Returns:    OK or ERROR
 */

STATUS
arianeEnableDumpCtrl(int *report)
{
  SDI_F->dump = GEN_TRUE;
  return OK;
}

/*------------------------------------------------------------------------
 * arianeDisableDumpCtrl  -  control codel of CONTROL request DisableDump
 *
 * Description:    
 * Report: OK
 *
 * Returns:    OK or ERROR
 */

STATUS
arianeDisableDumpCtrl(int *report)
{
  SDI_F->dump = GEN_FALSE;
  return OK;
}


