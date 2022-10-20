package org.firstinspires.ftc.teamcode.Tele.untested;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class TwoStageLinSlideFile {
    static final int low = 0; // declares encoder variables
    static final int mid = 2100;
    static final int high = 4200;
    public enum states {LOW, MEDIUM, HIGH, TOLOW, TOMEDIUM, TOHIGH} //state array for state machine
    public static states state = states.LOW;
    static DcMotor rightLinSlide = null; //DC Motors for lin slide
    //static DcMotor leftLinSlide = null;
    public static void setLSMotor (DcMotor rightLSMotor/*,DcMotor leftLSMotor*/){ //using encoders for motors
        rightLinSlide = rightLSMotor;
        //leftLinSlide = leftLSMotor;
        rightLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLinSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftLinSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public static void linSlideDouble(Gamepad gamepad1){ // using game pad for input to state machine
        moveStates(gamepad1.right_trigger, gamepad1.left_bumper, gamepad1.right_bumper);
    }
    public static void moveStates (float rightTrigger, boolean leftBumper, boolean rightBumper){ //Left Bumper goes to lowest state, Right Bumper goes to medium state, Right Trigger goes to High state
        switch(state){ //define state machine
            case LOW:
                rightLinSlide.setPower(0); //Linslide power 0
                //leftLinSlide.setPower(0);
                if (rightBumper){ // if right bumper clicked, go to mid state
                    state = states.TOMEDIUM;
                }
                else if (rightTrigger > 0.7){ // if right trigger clicked, go to high state
                    state = states.TOHIGH;
                }
                break;
            case MEDIUM:
                rightLinSlide.setPower(0);
                //leftLinSlide.setPower(0);
                if(leftBumper){
                    state = states.TOLOW;
                }
                else if(rightTrigger > 0.7){
                    state = states.TOHIGH;
                }
                break;
            case HIGH:
                rightLinSlide.setPower(0);
                //leftLinSlide.setPower(0);
                if(rightBumper){
                    state = states.TOMEDIUM;
                }
                else if(leftBumper){
                    state = states.LOW;
                }
                break;
            case TOLOW:
                if(rightLinSlide.getCurrentPosition() > low /*&& leftLinSlide.getCurrentPosition()>low*/){ // if right lin slide and left lin slide encoder is more than 0, go to 0
                    rightLinSlide.setPower(-0.9);
                    //leftLinSlide.setPower(-0.9);
                }
                else{
                    state = states.LOW; // anything else, set state to low and power to 0
                    rightLinSlide.setPower(0);
                    //leftLinSlide.setPower(0);
                }
                break;
            case TOMEDIUM:
                if(rightLinSlide.getCurrentPosition() < mid /*&& leftLinSlide.getCurrentPosition() < mid*/ ){
                    rightLinSlide.setPower(0.9);
                    //leftLinSlide.setPower(0.9);
                }
                else if (rightLinSlide.getCurrentPosition() > mid /*&& leftLinSlide.getCurrentPosition() > mid*/){
                    rightLinSlide.setPower(-0.9);
                    //leftLinSlide.setPower(-0.9);
                }
                else {
                    state = states.MEDIUM;
                    rightLinSlide.setPower(0);
                   // leftLinSlide.setPower(0);
                }
                break;
            case TOHIGH:
                if(rightLinSlide.getCurrentPosition() < high){
                    rightLinSlide.setPower(0.9);
                    //leftLinSlide.setPower(0.9);
                }
                else{
                    state = states.HIGH;
                    rightLinSlide.setPower(0);
                    //leftLinSlide.setPower(0);
                }
                break;
        }
    }
}
