package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.trash;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class fourarmtest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Servo right = hardwareMap.servo.get("right");
        Servo right1 = hardwareMap.servo.get("right1");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                right.setPosition(1);
            }
            else if(gamepad1.b){
                right.setPosition(0);
            }
        }
    }
}
