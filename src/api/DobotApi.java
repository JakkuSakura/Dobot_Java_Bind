package api;


import com.sun.jna.ptr.LongByReference;
import dobotdll.DobotDllLibrary;
import dobotdll.PTPCmd;

import static dobotdll.DobotDllLibrary.TRUE;


public class DobotApi {
    /**
     * Move to a point quickly
     * @param xyz to where to move
     * @return the index in the queue
     */
    public static long MoveQuickly(Vector3f xyz) {
        PTPCmd ptpCmd = new PTPCmd();
        ptpCmd.ptpMode = (byte) DobotDllLibrary.PTPMode.PTPMOVJXYZMode;
        ptpCmd.x = xyz.x;
        ptpCmd.y = xyz.y;
        ptpCmd.z = xyz.z;
        ptpCmd.r = 0;
        LongByReference longByReference = new LongByReference();
        DobotDllLibrary.INSTANCE.SetPTPCmd(ptpCmd, TRUE, longByReference);
        return longByReference.getValue();
    }

    /**
     * Move to a point slowly
     * @param xyz to where to move
     * @return the index in the queue
     */
    public static long MoveSlowly(Vector3f xyz) {
        PTPCmd ptpCmd = new PTPCmd();
        ptpCmd.ptpMode = (byte) DobotDllLibrary.PTPMode.PTPMOVLXYZMode;
        ptpCmd.x = xyz.x;
        ptpCmd.y = xyz.y;
        ptpCmd.z = xyz.z;
        ptpCmd.r = 0;
        LongByReference longByReference = new LongByReference();
        DobotDllLibrary.INSTANCE.SetPTPCmd(ptpCmd, TRUE, longByReference);
        return longByReference.getValue();
    }
}