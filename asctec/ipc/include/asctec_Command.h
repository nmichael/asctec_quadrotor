#ifndef __IPC_BRIDGE_MATLAB_ASCTEC_COMMAND__
#define __IPC_BRIDGE_MATLAB_ASCTEC_COMMAND__
#include <ipc_bridge_matlab/ipc_bridge_matlab.h>
#include <ipc_bridge/msgs/asctec_Command.h>

#include <rosgraph_msgs_Header.h>

namespace ipc_bridge_matlab
{
  namespace asctec
  {
    namespace Command
    {
      static mxArray* ProcessMessage(const ipc_bridge::asctec::Command &msg)
      {
        const char *fields[] = {"header", "thrust",
                                "roll", "pitch", "yaw",
                                "kp_roll", "kd_roll",
                                "kp_pitch", "kd_pitch",
                                "kp_yaw", "kd_yaw",
                                "p", "q", "r",
                                "z_correction", "r_correction", "p_correction"};
        const int nfields = sizeof(fields)/sizeof(*fields);
        mxArray *out = mxCreateStructMatrix(1, 1, nfields, fields);

        mxSetField(out, 0, "header",
                   ipc_bridge_matlab::Header::ProcessMessage(msg.header));

        mxSetField(out, 0, "thrust", mxCreateDoubleScalar(msg.thrust));
        mxSetField(out, 0, "roll", mxCreateDoubleScalar(msg.roll));
        mxSetField(out, 0, "pitch", mxCreateDoubleScalar(msg.pitch));
        mxSetField(out, 0, "yaw", mxCreateDoubleScalar(msg.yaw));
        mxSetField(out, 0, "kp_roll", mxCreateDoubleScalar(msg.kp_roll));
        mxSetField(out, 0, "kd_roll", mxCreateDoubleScalar(msg.kd_roll));
        mxSetField(out, 0, "kp_pitch", mxCreateDoubleScalar(msg.kp_pitch));
        mxSetField(out, 0, "kd_pitch", mxCreateDoubleScalar(msg.kd_pitch));
        mxSetField(out, 0, "kp_yaw", mxCreateDoubleScalar(msg.kp_yaw));
        mxSetField(out, 0, "kd_yaw", mxCreateDoubleScalar(msg.kd_yaw));
        mxSetField(out, 0, "p", mxCreateDoubleScalar(msg.p));
        mxSetField(out, 0, "q", mxCreateDoubleScalar(msg.q));
        mxSetField(out, 0, "r", mxCreateDoubleScalar(msg.r));
        mxSetField(out, 0, "z_correction", mxCreateDoubleScalar(msg.z_correction));
        mxSetField(out, 0, "r_correction", mxCreateDoubleScalar(msg.r_correction));
        mxSetField(out, 0, "p_correction", mxCreateDoubleScalar(msg.p_correction));

        return out;
      }

      static int ProcessArray(const mxArray *a,
                              ipc_bridge::asctec::Command &msg)
      {
        mxArray *field;

        field = mxGetField(a, 0, "header");
        ipc_bridge_matlab::Header::ProcessArray(field, msg.header);

        msg.thrust = mxGetScalar(mxGetField(a, 0, "thrust"));
        msg.roll = mxGetScalar(mxGetField(a, 0, "roll"));
        msg.pitch = mxGetScalar(mxGetField(a, 0, "pitch"));
        msg.yaw = mxGetScalar(mxGetField(a, 0, "yaw"));
        msg.kp_roll = mxGetScalar(mxGetField(a, 0, "kp_roll"));
        msg.kd_roll = mxGetScalar(mxGetField(a, 0, "kd_roll"));
        msg.kp_pitch = mxGetScalar(mxGetField(a, 0, "kp_pitch"));
        msg.kd_pitch = mxGetScalar(mxGetField(a, 0, "kd_pitch"));
        msg.kp_yaw = mxGetScalar(mxGetField(a, 0, "kp_yaw"));
        msg.kd_yaw = mxGetScalar(mxGetField(a, 0, "kd_yaw"));
        msg.p = mxGetScalar(mxGetField(a, 0, "p"));
        msg.q = mxGetScalar(mxGetField(a, 0, "q"));
        msg.r = mxGetScalar(mxGetField(a, 0, "r"));
        msg.z_correction = mxGetScalar(mxGetField(a, 0, "z_correction"));
        msg.r_correction = mxGetScalar(mxGetField(a, 0, "r_correction"));
        msg.p_correction = mxGetScalar(mxGetField(a, 0, "p_correction"));

        return SUCCESS;
      }

      static void Cleanup(ipc_bridge::asctec::Command &msg)
      {
        ipc_bridge_matlab::Header::Cleanup(msg.header);

        return;
      }
    }
  }
}
#endif
