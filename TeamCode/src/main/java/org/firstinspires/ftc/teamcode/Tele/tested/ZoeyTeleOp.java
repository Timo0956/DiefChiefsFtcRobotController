package org.firstinspires.ftc.teamcode.Tele.tested;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ZoeyTeleOp extends LinearOpMode {
    static DcMotor motorFrontLeft;
    static DcMotor motorFrontRight;
    static DcMotor motorBackLeft;
    static DcMotor motorBackRight;
    static DcMotor motorLinSlide;
    static Servo servoOne;
    static Servo servoTwo;

    public void runOpMode() throws InterruptedException{

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorLinSlide = hardwareMap.dcMotor.get("motorLinSlide");
        servoOne = hardwareMap.servo.get("servoOne");
        servoTwo = hardwareMap.servo.get("servoTwo");

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        ZoeyLinSlide.setLinSlide(motorLinSlide);
        ZoeyServos.setServos(servoOne, servoTwo);

        waitForStart();
        while(opModeIsActive()){
            ZoeyLinSlide.linSlideTransition(gamepad1);
            ZoeyServos.servoOpen(gamepad1.x);
            ZoeyServos.servoClose(gamepad1.y);

            double leftY = -gamepad1.left_stick_y;
            double leftX = gamepad1.left_stick_x;
            double rightX = gamepad1.right_stick_x;

            double denom = Math.max(1, Math.abs(leftX)+Math.abs(leftY)+Math.abs(rightX));
            double frontLeftPower = (leftY - leftX + rightX) /denom;
            double backLeftPower = (leftY + leftX - rightX) /denom;
            double frontRightPower = (leftY - leftX - rightX) /denom;
            double backRightPower = (leftY + leftX - rightX) /denom;

            motorFrontLeft.setPower(frontLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackLeft.setPower(backLeftPower);
            motorBackRight.setPower(backRightPower);
        }
    }
}
