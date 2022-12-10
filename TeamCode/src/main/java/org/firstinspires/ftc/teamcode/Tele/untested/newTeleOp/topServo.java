package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.hardware.CRServo;

public class topServo {
    static CRServo topServo;

    public static void initializeTopServo(CRServo servo) {
        topServo = servo;
    }

    public static void manualSpin(Boolean left, Boolean right) {
        if(right) { //Right Dpad is pressed down
            topServo.setPower(0.8);
        } else if(left) { //Left Dpad is pressed down
            topServo.setPower(-0.8);
        } else { //No input
            topServo.setPower(0);
        }
    }

    public static void autoSpinOut() throws InterruptedException {
        topServo.setPower(0.8);
        Thread.sleep(500);
        topServo.setPower(0);
    }

    public static void autoSpinOrigin() throws InterruptedException {
        topServo.setPower(-0.8);
        Thread.sleep(500);
        topServo.setPower(0);
    }
}
