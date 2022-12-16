package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class topMotorBurst {
    static DcMotor TM;

    public static void initTopMotorForBurst(DcMotor top) {
        TM = top;
        TM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TM.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static void burstMotor() throws InterruptedException{
        TM.setTargetPosition(-225);
        TM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TM.setPower(-0.05);
        Thread.sleep(100);
        TM.setPower(0);
    }


    public static void moveTopMotor(boolean left, boolean right){
        if(left){
            TM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            TM.setPower(0.05);
        }
        else if (right){
            TM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            TM.setPower(-0.05);
        }
        else{
            TM.setPower(0);
        }
    }
}
