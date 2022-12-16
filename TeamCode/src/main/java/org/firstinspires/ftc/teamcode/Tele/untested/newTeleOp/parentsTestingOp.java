package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.trash.ExtensionLinSlide;

@TeleOp
@Disabled
public class parentsTestingOp extends LinearOpMode {
    @Override

    public void runOpMode(){
        DcMotor rightLinslide = hardwareMap.dcMotor.get("rightLinslide");
        DcMotor leftLinslide = hardwareMap.dcMotor.get("leftLinslide");
        DcMotor TM = hardwareMap.dcMotor.get("topMotor");
        CRServo TS = hardwareMap.crservo.get("topServo");

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        TwoStageLinSlideFileNew.setLSMotor(rightLinslide, leftLinslide);
        topMotor.initTopMotor(TM);
        topServo.initializeTopServo(TS);

        waitForStart();
        while(opModeIsActive()) {
            TwoStageLinSlideFileNew.linSlideDouble(gamepad1);
            topMotor.moveTopMotor(gamepad1.dpad_right, gamepad1.dpad_left);
            topServo.manualSpin(gamepad1.dpad_down, gamepad1.dpad_up);

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
