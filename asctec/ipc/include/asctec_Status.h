#ifndef __IPC_BRIDGE_MATLAB_ASCTEC_STATUS__
#define __IPC_BRIDGE_MATLAB_ASCTEC_STATUS__
#include <ipc_bridge_matlab/ipc_bridge_matlab.h>
#include <ipc_bridge/msgs/asctec_Status.h>

namespace ipc_bridge_matlab
{
  namespace asctec
  {
    namespace Status
    {
      static mxArray* ProcessMessage(const ipc_bridge::asctec::Status &msg)
      {
        const char *fields[] = {"cpu_load", "voltage"};
        const int nfields = sizeof(fields)/sizeof(*fields);
        mxArray *out = mxCreateStructMatrix(1, 1, nfields, fields);

        mxSetField(out, 0, "cpu_load", mxCreateDoubleScalar(msg.cpu_load));
        mxSetField(out, 0, "voltage", mxCreateDoubleScalar(msg.voltage));

        return out;
      }

      static int ProcessArray(const mxArray *a,
                              ipc_bridge::asctec::Status &msg)
      {
        mxArray *field;

        msg.cpu_load = mxGetScalar(mxGetField(a, 0, "cpu_load"));
        msg.voltage = mxGetScalar(mxGetField(a, 0, "voltage"));

        return SUCCESS;
      }

      static void Cleanup(ipc_bridge::asctec::Status &msg)
      {
        return;
      }
    }
  }
}
#endif
