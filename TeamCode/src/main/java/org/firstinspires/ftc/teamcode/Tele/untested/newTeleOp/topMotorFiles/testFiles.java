package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.topMotorFiles;

import com.qualcomm.robotcore.hardware.CRServo;

public class testFiles {
    static CRServo R =null;
    static CRServo R1 = null;
    static CRServo L = null;
    static CRServo L1 = null;
    public static void initMotors(CRServo right, CRServo right1, CRServo left, CRServo left1){
        R = right;
        R1 = right1;
        L = left;
        L1 = left1;
    }
    public static void moveOut() throws InterruptedException{
        L.setPower(1);
        L1.setPower(1);
        R.setPower(-1);
        R1.setPower(-1);
        Thread.sleep(300);
        L.setPower(0);
        L1.setPower(0);
        R.setPower(0);
        R1.setPower(0);
    }
}
