package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.topMotorFiles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.HoriLinSlide.testHorizontalLinSlide;


@TeleOp
@Disabled
public class switchstateTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        //DcMotor TopMotor1 = hardwareMap.dcMotor.get("TopMotor");
        DcMotor hl = hardwareMap.dcMotor.get("hl");
        //CRServo fourArmInnerRight = hardwareMap.crservo.get("fourArmInnerRight");
        //CRServo fourArmInnerLeft = hardwareMap.crservo.get("fourArmInnerLeft");
        //CRServo fourArmOuterRight = hardwareMap.crservo.get("fourArmOuterRight");
        //Servo fourArmOuterLeft = hardwareMap.servo.get("fourArmOuterLeft");
        //topMotorSwitchState.initTopMotor(TopMotor1);
        //testFile2.init(fourArmOuterLeft);
        //testFiles.initMotors(fourArmInnerRight,fourArmOuterRight, fourArmInnerLeft, fourArmOuterLeft);
        testHorizontalLinSlide.initHori(hl);
        waitForStart();
        //testFile2.turnOpen();


        while (opModeIsActive()){
            //topMotorSwitchState.moveTopMotorStates(gamepad1.dpad_right, gamepad1.dpad_left);
            testHorizontalLinSlide.moveHorizontalLinManual(gamepad1.a,gamepad1.b);
            telemetry.addData("dpadStat", gamepad1.dpad_right);
            telemetry.update();
        }
    }
}
