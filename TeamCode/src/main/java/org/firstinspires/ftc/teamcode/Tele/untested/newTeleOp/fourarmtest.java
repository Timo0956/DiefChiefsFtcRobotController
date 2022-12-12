package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class fourarmtest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        CRServo right = hardwareMap.crservo.get("right");
        CRServo left = hardwareMap.crservo.get("left");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                right.setPower(1);
                left.setPower(-1);

            }
            else if(gamepad1.b){
                right.setPower(-1);
                left.setPower(1);
            }
            else {
                right.setPower(0);
                left.setPower(0);
            }
        }
    }
}
