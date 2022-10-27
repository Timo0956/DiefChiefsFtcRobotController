package org.firstinspires.ftc.teamcode.Tele.untested;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class EncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor linSlideTest = hardwareMap.dcMotor.get("linSlideTest");
        linSlideTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideTest.setTargetPosition(2100);
        linSlideTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        telemetry.addData("Mode", "Starting");
        telemetry.update();
        linSlideTest.setPower(0.2);
        while(opModeIsActive() && linSlideTest.isBusy()){
            telemetry.addData("Current Position=",linSlideTest.getCurrentPosition() + "running="+linSlideTest.isBusy());
            telemetry.update();
            idle();
        }


    }
}
