package org.firstinspires.ftc.teamcode.Auto.hardCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;
import org.firstinspires.ftc.teamcode.Tele.untested.linSlideFiles.TwoStageLinSlideFile;

@Autonomous
public class startFromRight extends LinearOpMode{
    static Servo ClawL = null;
    static Servo ClawR = null;
    static DcMotor motorFrontLeft = null;
    static DcMotor motorBackLeft = null;
    static DcMotor motorFrontRight = null;
    static DcMotor motorBackRight = null;
    static DcMotor rightLinSlide = null;
    @Override
    public void runOpMode() throws InterruptedException{
        long msPerCm = 1500/89;
        double power = 0.5;
        ClawL = hardwareMap.servo.get("clawServoL");
        ClawR = hardwareMap.servo.get("clawServoR");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        rightLinSlide = hardwareMap.dcMotor.get("rightLinSlide");
        TwoStageLinSlideFile.setLSMotor(rightLinSlide);
        waitForStart();

        //sequence starts here
        closeServo();
        pause(700); //grab cone
        leftStrafe(-power,msPerCm*120);
        pause(100);
        moveLinSlidePosition(3000,0.9, 2000); //lift cone
        forwardBackwardDrive(power,msPerCm*10); //get to position
        pause(100);
        openServo(); // drop cone
        pause(100);
        forwardBackwardDrive(-power,msPerCm*10);
        pause(100);
        moveLinSlidePosition(0,0.9, 2000); // lower linslide
        rightStrafe(-power,msPerCm*36); //parking

    }
    public static void pause(long time)throws InterruptedException{
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        Thread.sleep(time);
    }
    public static void forwardBackwardDrive (double power, long time) throws InterruptedException {
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(-power);
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
    /*    public static void toLowLinSlide(){
            TwoStageLinSlideFile.moveStates(0,true,false,0);
        }
        public static void toMidLinSlide(){
            TwoStageLinSlideFile.moveStates(0,false,true,0);
        }
        public static void toHighLinSlide(){
            TwoStageLinSlideFile.moveStates(1,false,false,0);
        }
        public static void toLowOffLinSlide(){TwoStageLinSlideFile.moveStates(0,false,false,1);} */// commented out extra linside functions
    public static void closeServo(){
        // ServoTele.close(true);
        ClawL.setPosition(0.15);
        ClawR.setPosition(0.15);
    }
    public static void openServo(){
        //   ServoTele.open(true);
        ClawL.setPosition(0);
        ClawR.setPosition(0);
    }
    public static void moveLinSlidePosition (int position, double speed, long time)throws InterruptedException{
        rightLinSlide.setTargetPosition(position);
        rightLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinSlide.setPower(speed);
        Thread.sleep(time);
    }
}
