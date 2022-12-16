package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.trash;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;

@TeleOp
@Disabled
public class fourarmtest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Servo right = hardwareMap.servo.get("right");
        Servo right1 = hardwareMap.servo.get("right1");
        Servo c1 = hardwareMap.servo.get("c1");
        Servo c2 = hardwareMap.servo.get("c2");

        ServoTele.setServos(c1, c2);

        waitForStart();

        while(opModeIsActive()){
            ServoTele.open(gamepad1.x);
            ServoTele.close(gamepad1.y);

            if(gamepad1.a){
                right.setPosition(0.65);
                right1.setPosition(0.40);

            }
            else if(gamepad1.b){
                right.setPosition(0);
                right1.setPosition(1);

            }
            telemetry.addData("pos", right.getPosition());
            telemetry.addData("pos2", right1.getPosition());
            telemetry.addData("pos3", c1.getPosition());
            telemetry.addData("pos4", c2.getPosition());
            telemetry.update();
        }
    }
}
