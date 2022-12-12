package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.topMotorFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.servo180pullback;

@TeleOp
public class switchstateTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor TopMotor1 = hardwareMap.dcMotor.get("TopMotor");
        CRServo fourArmInnerRight = hardwareMap.crservo.get("fourArmInnerRight");
        CRServo fourArmInnerLeft = hardwareMap.crservo.get("fourArmInnerLeft");
        CRServo fourArmOuterRight = hardwareMap.crservo.get("fourArmOuterRight");
        CRServo fourArmOuterLeft = hardwareMap.crservo.get("fourArmOuterLeft");
        topMotorSwitchState.initTopMotor(TopMotor1);
        testFiles.initMotors(fourArmInnerRight,fourArmOuterRight, fourArmInnerLeft, fourArmOuterLeft);
        waitForStart();
        testFiles.moveOut();

        while (opModeIsActive()){
            topMotorSwitchState.moveTopMotorStates(gamepad1.dpad_right, gamepad1.dpad_left);
            telemetry.addData("dpadStat", gamepad1.dpad_right );
            telemetry.update();
        }
    }
}
