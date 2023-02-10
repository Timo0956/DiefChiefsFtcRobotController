package org.firstinspires.ftc.teamcode.Tele.untested.COMPCODEDON;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class encodertest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        DcMotor rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
        DcMotor frontEncoder = hardwareMap.dcMotor.get("frontEncoder");
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Right Encoder: ", rightEncoder.getCurrentPosition());
            telemetry.addData("Left Encoder: ",leftEncoder.getCurrentPosition() );
            telemetry.addData("Front Encoder: ", frontEncoder.getCurrentPosition());
            telemetry.update();

        }
    }

}
