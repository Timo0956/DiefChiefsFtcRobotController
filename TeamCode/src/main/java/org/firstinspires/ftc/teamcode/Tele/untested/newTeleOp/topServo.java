package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class topServo {
    static CRServo TS = null;
    public static void initTopServo (CRServo topServo) {
        TS = topServo;
        TS.setPower(0);
    }
    public static void turnTSOpen() throws InterruptedException{
        TS.setPower(1);
        Thread.sleep(750);
        TS.setPower(0);
    }
    public static void turnTSClose() throws InterruptedException{
        TS.setPower(-1);
        Thread.sleep(750);
        TS.setPower(0);
    }
    public static void manualTSControl(boolean Left, boolean Right){
        if(Left){
            TS.setPower(-1);
        }
        else if (Right){
            TS.setPower(1);
        }
        else {
            TS.setPower(0);
        }
    }
}
