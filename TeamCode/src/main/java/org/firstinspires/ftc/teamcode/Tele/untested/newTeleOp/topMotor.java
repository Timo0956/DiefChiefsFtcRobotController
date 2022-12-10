package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

public class topMotor {
    static DcMotor TM = null;
    public static void initTopMotor (DcMotor topMotor) {
        TM = topMotor;
        TM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public static void moveTopMotor(boolean Right, boolean Left){
        if(Right){
            TM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            TM.setPower(0.05);
        }
        else if (Left){
            TM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            TM.setPower(-0.05);
        }
        else{
            TM.setPower(0);
        }
    }
    public static void autoMoveToPosition(){
        if(TM.getCurrentPosition() > -275) {
            TM.setTargetPosition(-275);
            TM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            TM.setPower(0.1);
        }
        else{
            TM.setPower(0);
        }
    }
    public static void autoMoveToOriginal(){
        if(TM.getCurrentPosition() < 0) {
            TM.setTargetPosition(0);
            TM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            TM.setPower(-0.1);
        }
        else{
            TM.setPower(0);
        }
    }
}
