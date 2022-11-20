package org.firstinspires.ftc.teamcode.Auto.noGood;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;
import org.firstinspires.ftc.teamcode.Tele.untested.linSlideFiles.TwoStageLinSlideFile;

public class FunctionsPage {
    static DcMotor motorFrontLeft = null;
    static DcMotor motorBackLeft = null;
    static DcMotor motorFrontRight = null;
    static DcMotor motorBackRight = null;
    public static void forwardBackwardDrive (float power, long time) throws InterruptedException {
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        Thread.sleep(time);
    }
    public static void leftStrafe (float power, long time) throws InterruptedException{
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        Thread.sleep(time);
    }

    public static void rightStrafe (float power, long time) throws InterruptedException {
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(-power);
        motorBackRight.setPower(-power);
        Thread.sleep(time);
    }
   /* public static void toLowLinSlide(){
        TwoStageLinSlideFile.moveStates(0,true,false,0);
    }
    public static void toMidLinSlide(){
        TwoStageLinSlideFile.moveStates(0,false,true,0);
    }
    public static void toHighLinSlide(){
        TwoStageLinSlideFile.moveStates(1,false,false,0);
    }
    public static void toLowOffLinSlide(){TwoStageLinSlideFile.moveStates(0,false,false,1);} */
    public static void closeServo(){
        ServoTele.close(true);
    }
    public static void openServo(){
        ServoTele.open(true);
    }
}
