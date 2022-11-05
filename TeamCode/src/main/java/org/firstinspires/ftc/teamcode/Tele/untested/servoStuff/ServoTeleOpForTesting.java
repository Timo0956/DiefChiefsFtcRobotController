package org.firstinspires.ftc.teamcode.Tele.untested.servoStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTeleOpForTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Servo ClawServoL = hardwareMap.servo.get("clawServoL");
        Servo ClawServoR = hardwareMap.servo.get("clawServoR");
        waitForStart();
        while(opModeIsActive()){
            ServoTele.setServos(ClawServoL, ClawServoR);
            ServoTele.open(gamepad1.x);
            ServoTele.close(gamepad1.y);
        }
    }
}
