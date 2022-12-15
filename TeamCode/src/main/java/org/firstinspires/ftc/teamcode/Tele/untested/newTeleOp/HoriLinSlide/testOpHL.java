package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.HoriLinSlide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class testOpHL extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {

        DcMotor horizontal = hardwareMap.dcMotor.get("HL");

        testHorizontalLinSlide.initHori(horizontal);

        waitForStart();
        while(opModeIsActive()) {
            testHorizontalLinSlide.moveHorizontalLinManual(gamepad1.left_bumper, gamepad1.right_bumper);
        }

    }



}
