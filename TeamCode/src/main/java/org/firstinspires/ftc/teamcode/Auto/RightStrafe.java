package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tele.untested.ServoTele;
import org.firstinspires.ftc.teamcode.Tele.untested.TwoStageLinSlideFile;

@Autonomous(name = "rightStrafeAuto")
public class RightStrafe extends LinearOpMode{
    static Servo ClawL = null;
    static Servo ClawR = null;
    static DcMotor motorFrontLeft = null;
    static DcMotor motorBackLeft = null;
    static DcMotor motorFrontRight = null;
    static DcMotor motorBackRight = null;
    static DcMotor rightLinSlide = null;
    @Override
    public void runOpMode() throws InterruptedException{

        ClawL = hardwareMap.servo.get("clawServoL");
        ClawR = hardwareMap.servo.get("clawServoR");
        ServoTele.setServos(ClawL, ClawR);
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        rightLinSlide = hardwareMap.dcMotor.get("rightLinSlide");
        TwoStageLinSlideFile.setLSMotor(rightLinSlide);
        waitForStart();
        int power = 1;
        rightStrafe(power,1000);

    }
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
    public static void toLowLinSlide(){
        TwoStageLinSlideFile.moveStates(0,true,false);
    }
    public static void toMidLinSlide(){
        TwoStageLinSlideFile.moveStates(0,false,false);
    }
    public static void toHighLinSlide(){
        TwoStageLinSlideFile.moveStates(1,false,false);
    }
    public static void closeServo(){
        ServoTele.close();
    }
    public static void openServo(){
        ServoTele.open();
    }
}