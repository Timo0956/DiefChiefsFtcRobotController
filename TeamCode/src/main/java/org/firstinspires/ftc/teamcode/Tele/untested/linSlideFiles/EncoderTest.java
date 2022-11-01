package org.firstinspires.ftc.teamcode.Tele.untested.linSlideFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class EncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor linSlideTest = hardwareMap.dcMotor.get("linSlideTest");
        linSlideTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        telemetry.addData("Mode", "Ready2Start");
        telemetry.update();

        while(opModeIsActive()){
            linSlideTest.setTargetPosition(2100);
            linSlideTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linSlideTest.setPower(0.2);
            telemetry.addData("Current Position=",linSlideTest.getCurrentPosition() + "running="+linSlideTest.isBusy());
            telemetry.update();
        }
    }
}
