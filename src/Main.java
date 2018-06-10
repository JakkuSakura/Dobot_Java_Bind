import java.util.Timer;
import java.util.TimerTask;

import api.DobotApi;
import api.Vector3f;
import com.sun.jna.ptr.LongByReference;
import dobotdll.*;

import static dobotdll.DobotDllLibrary.DobotConnect_NotFound;
import static dobotdll.DobotDllLibrary.DobotConnect_Occupied;
import static dobotdll.DobotDllLibrary.FALSE;
import static dobotdll.DobotDllLibrary.TRUE;

// tip: The demo must import Jna library, inner DobotDemo folder of this project
public class Main {
    public static void main(String[] args) {
        try {

            Main app = new Main();
            app.Start();

            while (true) {
                DobotApi.MoveQuickly(new Vector3f(260, 0, 50));
                DobotApi.MoveQuickly(new Vector3f(220, 0, 80));
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            DobotDllLibrary.INSTANCE.DisconnectDobot();
        }
    }

    private void Start() {

        int ret = DobotDllLibrary.INSTANCE.ConnectDobot("", 115200,"","");


        if (ret == DobotConnect_NotFound || ret == DobotConnect_Occupied) {
            Msg("Connect error, code:" + ret);
            return;
        }
        Msg("connect success code:" + ret);

        StartDobot();

        StartGetStatus();
    }

    /* (non-Java-doc)
     * @see java.lang.Object#Object()
     */
    public Main() {
        super();
    }

    private void StartDobot() {
        LongByReference lb = new LongByReference();
        EndEffectorParams endEffectorParams = new EndEffectorParams();
        endEffectorParams.xBias = 71.6f;
        endEffectorParams.yBias = 0;
        endEffectorParams.zBias = 0;
        DobotDllLibrary.INSTANCE.SetEndEffectorParams(endEffectorParams, FALSE, lb);
        JOGJointParams jogJointParams = new JOGJointParams();
        for (int i = 0; i < 4; i++) {
            jogJointParams.velocity[i] = 200;
            jogJointParams.acceleration[i] = 200;
        }
        DobotDllLibrary.INSTANCE.SetJOGJointParams(jogJointParams, FALSE, lb);

        JOGCoordinateParams jogCoordinateParams = new JOGCoordinateParams();
        for (int i = 0; i < 4; i++) {
            jogCoordinateParams.velocity[i] = 200;
            jogCoordinateParams.acceleration[i] = 200;
        }
        DobotDllLibrary.INSTANCE.SetJOGCoordinateParams(jogCoordinateParams, FALSE, lb);

        JOGCommonParams jogCommonParams = new JOGCommonParams();
        jogCommonParams.velocityRatio = 50;
        jogCommonParams.accelerationRatio = 50;
        DobotDllLibrary.INSTANCE.SetJOGCommonParams(jogCommonParams, FALSE, lb);

        PTPJointParams ptpJointParams = new PTPJointParams();
        for (int i = 0; i < 4; i++) {
            ptpJointParams.velocity[i] = 200;
            ptpJointParams.acceleration[i] = 200;
        }
        DobotDllLibrary.INSTANCE.SetPTPJointParams(ptpJointParams, FALSE, lb);

        PTPCoordinateParams ptpCoordinateParams = new PTPCoordinateParams();
        ptpCoordinateParams.xyzVelocity = 200;
        ptpCoordinateParams.xyzAcceleration = 200;
        ptpCoordinateParams.rVelocity = 200;
        ptpCoordinateParams.rAcceleration = 200;
        DobotDllLibrary.INSTANCE.SetPTPCoordinateParams(ptpCoordinateParams, FALSE, lb);

        PTPJumpParams ptpJumpParams = new PTPJumpParams();
        ptpJumpParams.jumpHeight = 20;
        ptpJumpParams.zLimit = 180;
        DobotDllLibrary.INSTANCE.SetPTPJumpParams(ptpJumpParams, FALSE, lb);

        DobotDllLibrary.INSTANCE.SetCmdTimeout(3000);
        DobotDllLibrary.INSTANCE.SetQueuedCmdClear();
        DobotDllLibrary.INSTANCE.SetQueuedCmdStartExec();
    }

    private void StartGetStatus() {
        Timer timerPos = new Timer();
        timerPos.schedule(new TimerTask() {
            public void run() {
                Pose pose = new Pose();
                DobotDllLibrary.INSTANCE.GetPose(pose);

                Msg("joint1Angle=" + pose.jointAngle[0] + "  "
                        + "joint2Angle=" + pose.jointAngle[1] + "  "
                        + "joint3Angle=" + pose.jointAngle[2] + "  "
                        + "joint4Angle=" + pose.jointAngle[3] + "  "
                        + "x=" + pose.x + "  "
                        + "y=" + pose.y + "  "
                        + "z=" + pose.z + "  "
                        + "r=" + pose.r + "  ");
            }
        }, 100, 500);//
    }

    private void Msg(String string) {
        System.out.println(string);
    }
}