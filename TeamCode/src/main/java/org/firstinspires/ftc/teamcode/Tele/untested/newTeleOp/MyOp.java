package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import static org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele.setServos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.trash.ExtensionLinSlide;

//import org.firstinspires.ftc.teamcode.Tele.tested.initialize2023;


@TeleOp
public class MyOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor rightLinSlide = hardwareMap.dcMotor.get("rightLinSlide"); //defines our motors for LinSlide
        DcMotor leftLinSlide = hardwareMap.dcMotor.get("leftLinSlide");
        DcMotor HL = hardwareMap.dcMotor.get("HL");

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        DcMotor TopMotor = hardwareMap.dcMotor.get("TopMotor");

        Servo ClawServoL = hardwareMap.servo.get("clawServoL");
        Servo ClawServoR = hardwareMap.servo.get("clawServoR");

        CRServo ClawServo = hardwareMap.crservo.get("clawServo");
        //CRServo fourArmInnerRight = hardwareMap.crservo.get("fourArmInnerRight");
        Servo fourArmInnerLeft = hardwareMap.servo.get("fourArmInnerLeft");
        //CRServo fourArmOuterRight = hardwareMap.crservo.get("fourArmOuterRight");
        Servo fourArmOuterLeft = hardwareMap.servo.get("fourArmOuterLeft");


        clawServoClass.clawServoInit(ClawServo);
        servo180pullback.placeHolderServoInit(fourArmInnerLeft, fourArmOuterLeft);
        setServos(ClawServoL, ClawServoR);
        ExtensionLinSlide.initMotorsHoriLin(HL);
        topMotor.initTopMotor(TopMotor);

        //set zero power behavior to brake
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        TwoStageLinSlideFileNew.setLSMotor(rightLinSlide, leftLinSlide); //defines motors in terms of the seperate file
        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()){
            TwoStageLinSlideFileNew.linSlideDouble(gamepad1); //takes gamepad input
            topMotor.moveTopMotor(gamepad1.dpad_right, gamepad1.dpad_left);
            clawServoClass.spinClawServo(gamepad1.dpad_up,gamepad1.dpad_down);
            pieceTogether.pieceTogether(gamepad1);
            telemetry.addData("Position", rightLinSlide.getCurrentPosition());
            telemetry.addData("ServoPositionR", ClawServoR.getPosition());
            telemetry.addData("ServoPositionL", ClawServoL.getPosition());
            telemetry.update();
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x ;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) * 0.65/ denominator;
            double backLeftPower = (y - x + rx) * 0.65/ denominator;
            double frontRightPower = (y - x - rx) * 0.65/ denominator;
            double backRightPower = (y + x - rx)* 0.65/ denominator;
            motorFrontLeft.setPower(-frontLeftPower*0.6);
            motorBackLeft.setPower(-backLeftPower*0.6);
            motorFrontRight.setPower(-frontRightPower*0.6);
            motorBackRight.setPower(-backRightPower*0.6);

        }
    }
}
