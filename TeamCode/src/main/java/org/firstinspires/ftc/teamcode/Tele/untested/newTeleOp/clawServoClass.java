package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.hardware.CRServo;

public class clawServoClass {

    static CRServo clawServo;

    public static void clawServoInit(CRServo cServo){
        clawServo = cServo;
        clawServo.setPower(0);
    }

    public static void spinClawServo(boolean up, boolean down) throws InterruptedException{
        if(up) {
            clawServo.setPower(-1);
            Thread.sleep(300);
            clawServo.setPower(0);
        } else if(down) {
            clawServo.setPower(1);
            Thread.sleep(300);
            clawServo.setPower(0);
        } else {
            clawServo.setPower(0);
        }
    }
}
