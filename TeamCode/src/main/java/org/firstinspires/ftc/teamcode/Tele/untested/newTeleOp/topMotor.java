package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class topMotor {
    static DcMotor TM = null;
    static int position = -225;

    static Boolean burst = false;
    static double power = 0.05;

    public static void initTopMotor (DcMotor topMotor) {
        TM = topMotor;
        TM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TM.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public static void moveTopMotor(boolean Right, boolean Left){
        if(Right && TwoStageLinSlideFileNew.rightLinSlide.getCurrentPosition()< 1800){
            TM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            TM.setPower(0.05);
        }
        else if (Left && TwoStageLinSlideFileNew.rightLinSlide.getCurrentPosition()< 1800){
            TM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            TM.setPower(-0.05);
        }
        else{
            TM.setPower(0);
        }
    }

    public enum state {inFront, behind, toFront, toBehind}

    public static state states = state.inFront;


    public static void moveTopMotorRetainPosition(Boolean right, Boolean left) throws InterruptedException {
        switch(states) {
            case inFront:
                if(right) {
                    states = state.toBehind;
                }
                burst = true;
                while(burst){
                    if(TM.getCurrentPosition() < position){
                        TM.setTargetPosition(position);
                        TM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        TM.setPower(-power);
                        Thread.sleep (300);
                    }
                    else if (TM.getCurrentPosition() > position){
                        TM.setTargetPosition(position);
                        TM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        TM.setPower(power);
                        Thread.sleep(300);
                    }

                }
                break;

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
