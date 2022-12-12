package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.fourArmTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class testingFourArm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        CRServo fourBarTest = hardwareMap.crservo.get("fourBarTestServo");
        waitForStart();
        if(gamepad1.a){
            fourBarTest.setPower(0.2);
        }
        else if(gamepad1.b){
            fourBarTest.setPower(-0.2);
        }
        else{
            fourBarTest.setPower(0);
        }
    }
}
