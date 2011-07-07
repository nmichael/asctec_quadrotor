#include <mex.h>
#include "QuadrotorModel.h"

QuadrotorModel model;

extern "C" 
{
  void mexExit()
  {
    return;
  }

  void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
  {
    // Register the exit function
    mexAtExit(mexExit);

    // Verify the input is good, something must be given
    if (nrhs == 0) 
      {
        mexWarnMsgTxt("Need input argument");
        plhs[0] = mxCreateDoubleScalar(-1);
        return;
      }

    int buflen = 128;
    char buf[buflen];
    
    // Check to see that we can read the input string
    if (mxGetString(prhs[0], buf, buflen) != 0) 
      {
        mexWarnMsgTxt("Could not read string.");
        plhs[0] = mxCreateDoubleScalar(-1);
        return;
      }

    if (strcmp("update", buf) == 0)
      {               
#ifdef DEBUG
        mexPrintf("dt: %f\n", mxGetScalar(prhs[1]));
#endif
        model.Update(mxGetScalar(prhs[1]));
        return;
      }
    else if (strcmp("set_pd_cmd", buf) == 0)
      {
        int nrows = mxGetM(prhs[1]);
        int ncols = mxGetN(prhs[1]);

        bool ok = false;
        if (((nrows == 1) && (ncols == 9)) ||
            ((nrows == 9) && (ncols == 1)))
          ok = true;
        
        if (!ok)
          {
            mexWarnMsgTxt("Expected a 1 x 9 or 9 x 1 array");
            return;
          }

        double *p = mxGetPr(prhs[1]);
#ifdef DEBUG
        mexPrintf("pd cmd: ");
        for (int i = 0; i < 9; i++)
          mexPrintf("%f ", p[i]);
        mexPrintf("\n");
#endif
        model.SetPDCommand(p[0], p[1], p[2], p[3],
                           p[4], p[5], p[6], p[7], p[8]);
        model.SendCommand();
        return;
      }
    else if (strcmp("get_state", buf) == 0)
      {
        plhs[0] = mxCreateDoubleMatrix(1, 16, mxREAL);
        double *p = mxGetPr(plhs[0]);
        model.GetXYZ(p[0], p[1], p[2]);
        model.GetRPY(p[3], p[4], p[5]);
        model.GetWorldLinearVelocity(p[6], p[7], p[8]);
        model.GetBodyAngularVelocity(p[9], p[10], p[11]);
        model.GetRotorVelocity(p[12], p[13], p[14], p[15]);
#ifdef DEBUG
        mexPrintf("get_state: ");
        for (int i = 0; i < 16; i++)
          mexPrintf("%f ", p[i]);
        mexPrintf("\n");
#endif
        return;
      }
    else if (strcmp("set_state", buf) == 0)
      {
        int nrows = mxGetM(prhs[1]);
        int ncols = mxGetN(prhs[1]);
        
        bool ok = false;
        if (((nrows == 1) && (ncols == 16)) ||
            ((nrows == 16) && (ncols == 1)))
          ok = true;
        
        if (!ok)
          {
            mexWarnMsgTxt("Expected a 1 x 16 or 16 x 1 array");
            return;
          }

        double *p = mxGetPr(prhs[1]);
#ifdef DEBUG
        mexPrintf("set_state: ");
        for (int i = 0; i < 16; i++)
          mexPrintf("%f ", p[i]);
        mexPrintf("\n");
#endif
        model.SetXYZ(p[0], p[1], p[2]);
        model.SetRPY(p[3], p[4], p[5]);
        model.SetWorldLinearVelocity(p[6], p[7], p[8]);
        model.SetBodyAngularVelocity(p[9], p[10], p[11]);
        model.SetRotorVelocity(p[12], p[13], p[14], p[15]);
        return;
      }
    else if (strcmp("reset", buf) == 0)
      {
        model.ResetSimulation();
        return;
      }
    else if (strcmp("set_model", buf) == 0)
      {
        char model_buf[256];
        if (mxGetString(prhs[1], model_buf, 256) != 0)
          {
            mexWarnMsgTxt("Expected a string input");
            return;
          }
        
        model.LoadModel(model_buf);
        return;
      }
    else
      {
        mexWarnMsgTxt("Unhandled Input.");
        return;
      }
  }
}
