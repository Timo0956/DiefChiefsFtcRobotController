package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class servo180pullback {

    static CRServo placeHolder;
    static CRServo placeHolder1;


    public static void placeHolderServoInit(CRServo servo180, CRServo servo180Number2) {
        placeHolder = servo180;
        placeHolder1 = servo180Number2;
    }

   public static void pullIn() throws InterruptedException{
        placeHolder1.setPower(1);
        placeHolder.setPower(-1);
        Thread.sleep(300);
        placeHolder1.setPower(0);
        placeHolder.setPower(0);

    }
    public static void pushOut() throws InterruptedException{
        placeHolder.setPower(-1);
        placeHolder1.setPower(1);
        placeHolder.setPower(1);
        Thread.sleep(300);
        placeHolder1.setPower(0);
        placeHolder.setPower(0);


    }



}
