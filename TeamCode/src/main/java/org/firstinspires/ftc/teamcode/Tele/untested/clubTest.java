package org.firstinspires.ftc.teamcode.Tele.untested;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class clubTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor jimmy = hardwareMap.dcMotor.get("jimmy");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.y){
                jimmy.setPower(1);
            }
            else{
                jimmy.setPower(0);
            }

        }


    }
}
