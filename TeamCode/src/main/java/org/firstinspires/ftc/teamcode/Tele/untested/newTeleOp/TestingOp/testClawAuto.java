package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.TestingOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.clawServoClass;
import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.topMotor;

@TeleOp
public class testClawAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor TopMotor = hardwareMap.dcMotor.get("topMotor");
        CRServo ClawServo = hardwareMap.crservo.get("clawServo");
        topMotor.initTopMotor(TopMotor);
        clawServoClass.clawServoInit(ClawServo);
        waitForStart();
        topMotor.autoMoveToPosition();

    }
}
