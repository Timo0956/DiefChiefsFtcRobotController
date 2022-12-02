package org.firstinspires.ftc.teamcode.Tele.tested;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ZoeyLinSlide {
    static final int stageOne = 0;
    static final int stageTwo = 1750;
    static final int stageThree = 4000;
    static final double power = 0.9;

    public enum states {toStageOne, stageOne, toStageTwo, stageTwo, toStageThree, stageThree}
    public static states state = states.stageOne;
    static DcMotor motorLinSlide = null;

    public static void setLinSlide(DcMotor LS){
        motorLinSlide = LS;
        motorLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLinSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public static void linSlideTransition(Gamepad gamepad1){
        moveStates(gamepad1.left_trigger,gamepad1.left_bumper,gamepad1.right_bumper);
    }

    public static void moveStates(double LT, boolean LB, boolean RB){
        switch(state){
            case stageOne:
                if (RB){
                    state = state.toStageThree;
                } else if (LB){
                    state = state.toStageTwo;
                } else if (LT > 0.7){
                    state = state.toStageOne;
                }
            break;
            case stageTwo:
                if (RB){
                    state = state.toStageThree;
                } else if (LT > 0.7){
                    state = state.toStageOne;
                }
            break;
            case stageThree:
                if (LB){
                    state = state.toStageTwo;
                } else if (LT > 0.7){
                    state = state.toStageOne;
                } else if (RB){
                    state = state.toStageThree;
                }
            break;
            case toStageOne:
                if (motorLinSlide.getCurrentPosition() > stageOne){
                    motorLinSlide.setTargetPosition(stageOne);
                    motorLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorLinSlide.setPower(-power);
                } else {
                    state = states.stageOne;
                    motorLinSlide.setPower(0);
                }
            break;
            case toStageTwo:
                if (motorLinSlide.getCurrentPosition() > stageTwo){
                    motorLinSlide.setTargetPosition(stageTwo);
                    motorLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorLinSlide.setPower(-power);
                } else if (motorLinSlide.getCurrentPosition() < stageTwo){
                    motorLinSlide.setTargetPosition(stageTwo);
                    motorLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorLinSlide.setPower(power);
                } else {
                    state = states.stageTwo;
                    motorLinSlide.setPower(0);
                }
            break;
            case toStageThree:
                if (motorLinSlide.getCurrentPosition() < stageThree){
                    motorLinSlide.setTargetPosition(stageThree);
                    motorLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorLinSlide.setPower(power);
                } else {
                    state = states.stageThree;
                    motorLinSlide.setPower(0);
                }
            break;
        }
    }
}
