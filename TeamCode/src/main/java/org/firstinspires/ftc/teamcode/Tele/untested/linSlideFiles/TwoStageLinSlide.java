package org.firstinspires.ftc.teamcode.Tele.untested.linSlideFiles;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TwoStageLinSlide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor rightLinSlide = hardwareMap.dcMotor.get("rightLinSlide"); //defines our motors for LinSlide
        //DcMotor leftLinSlide = hardwareMap.dcMotor.get("leftLinSlide");
        TwoStageLinSlideFile.setLSMotor(rightLinSlide/*, leftLinSlide*/); //defines motors in terms of the seperate file

        waitForStart();
        while (opModeIsActive()){
            TwoStageLinSlideFile.linSlideDouble(gamepad1); //takes gamepad input
        }
    }
}
