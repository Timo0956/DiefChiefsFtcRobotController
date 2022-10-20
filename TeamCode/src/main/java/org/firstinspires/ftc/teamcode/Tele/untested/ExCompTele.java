package org.firstinspires.ftc.teamcode.Tele.untested;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Tele.tested.initialize2023;
import org.firstinspires.ftc.teamcode.Tele.untested.ServoTele;
import org.firstinspires.ftc.teamcode.Tele.untested.TwoStageLinSlideFile;

@TeleOp
public class ExCompTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor rightLinSlide = hardwareMap.dcMotor.get("rightLinSlide"); //defines our motors for LinSlide
        //DcMotor leftLinSlide = hardwareMap.dcMotor.get("leftLinSlide");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        Servo Clawservo = hardwareMap.servo.get("clawServo");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){
            TwoStageLinSlideFile.setLSMotor(rightLinSlide/*, leftLinSlide*/); //defines motors in terms of the seperate file
            TwoStageLinSlideFile.linSlideDouble(gamepad1); //takes gamepad input
            
            ServoTele.setServos(ClawServo);
            ServoTele.release(gamepad1.x);
            ServoTele.close(gamepad1.y);
                        
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(-frontLeftPower);
            motorBackLeft.setPower(-backLeftPower);
            motorFrontRight.setPower(-frontRightPower);
            motorBackRight.setPower(-backRightPower);
            
        }
    }
}
