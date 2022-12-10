package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.TestingOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.clawServoClass;
import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.topMotor;

@TeleOp
public class topmotorandintakeOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor TopMotor = hardwareMap.dcMotor.get("topMotor");
        CRServo ClawServo = hardwareMap.crservo.get("clawServo");
        topMotor.initTopMotor(TopMotor);
        clawServoClass.clawServoInit(ClawServo);
        waitForStart();
        int modes = 1;
        while(opModeIsActive()){
            topMotor.moveTopMotor(gamepad1.dpad_right, gamepad1.dpad_left);
            clawServoClass.spinClawServo(gamepad1.dpad_up, gamepad1.dpad_down);
            telemetry.addData("Pos", TopMotor.getCurrentPosition());
            telemetry.addData("dir", TopMotor.getDirection());
            telemetry.update();



        }

    }
}
