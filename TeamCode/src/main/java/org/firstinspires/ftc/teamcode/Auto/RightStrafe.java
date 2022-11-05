package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;
import org.firstinspires.ftc.teamcode.Tele.untested.linSlideFiles.TwoStageLinSlideFile;

@Autonomous(name = "russianAutoBotCode") // Russian bot code simiarly
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
        float power = 1;
        closeServo();
        toLowOffLinSlide();
        leftStrafe(power,1500);
        forwardBackwardDrive(-power, 1000);
        leftStrafe(power,500);
        toMidLinSlide();
        forwardBackwardDrive(power, 300);
        openServo();
        forwardBackwardDrive(-power, 300);
        toLowOffLinSlide();
        leftStrafe(power, 500);
        moveLinSlidePosition(600, 0.9);
        forwardBackwardDrive(power, 1500);
        closeServo();
        forwardBackwardDrive(0.7, 200);
        moveLinSlidePosition(500, -0.9);
        forwardBackwardDrive(-power, 1500);
        rightStrafe(power, 500);
        toMidLinSlide();
        forwardBackwardDrive(power, 300);
        openServo();
        leftStrafe(power, 500);
    }



    public static void forwardBackwardDrive (double power, long time) throws InterruptedException {
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        Thread.sleep(time);
    }
    public static void leftStrafe (double power, long time) throws InterruptedException{
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        Thread.sleep(time);
    }
    public static void rightStrafe (double power, long time) throws InterruptedException {
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(-power);
        motorBackRight.setPower(-power);
        Thread.sleep(time);
    }
    public static void toLowLinSlide(){
        TwoStageLinSlideFile.moveStates(0,true,false,0);
    }
    public static void toMidLinSlide(){
        TwoStageLinSlideFile.moveStates(0,false,true,0);
    }
    public static void toHighLinSlide(){
        TwoStageLinSlideFile.moveStates(1,false,false,0);
    }
    public static void toLowOffLinSlide(){TwoStageLinSlideFile.moveStates(0,false,false,1);}
    public static void closeServo(){
        ServoTele.close(true);
    }
    public static void openServo(){
        ServoTele.open(true);
    }
    public static void moveLinSlidePosition (int position, double speed){
        rightLinSlide.setTargetPosition(position);
        rightLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinSlide.setPower(speed);

    }

}