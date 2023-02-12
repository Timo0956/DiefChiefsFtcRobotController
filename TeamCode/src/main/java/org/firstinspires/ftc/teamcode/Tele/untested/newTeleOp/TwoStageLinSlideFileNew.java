package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.HoriLinSlide.testHorizontalLinSlide;
import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.clawServoClass;
import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.dualServoForearm;
import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;

public class TwoStageLinSlideFileNew {
    static final int low = 0; // declares encoder variables
    static final int lowOff = 1850;
    static final int mid = 2900;
    static final int high = 3970;
    static final double power = 1;
    static final boolean mode = true;


    public enum states {LOW, LOWOFF, MEDIUM, HIGH,TOLOWOFF, TOLOW, TOMEDIUM, TOHIGH, CUSTOM} //state array for state machine
    public static states state = states.LOW;
    static DcMotor rightLinSlide = null; //DC Motors for lin slide
    static DcMotor leftLinSlide = null;
    public static void setLSMotor (DcMotor rightLSMotor,DcMotor leftLSMotor){ //using encoders for motors
        rightLinSlide = rightLSMotor;
        leftLinSlide = leftLSMotor;
        rightLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLinSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLinSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public static void linSlideDouble(Gamepad gamepad1){ // using game pad for input to state machine
        if(mode) {
            moveStates(gamepad1.right_trigger, gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.left_trigger, gamepad1.a, gamepad1.b);
        } else if (mode == false) {
            if(gamepad1.left_trigger>0.2 && rightLinSlide.getCurrentPosition()>=10){
                rightLinSlide.setPower(gamepad1.left_trigger*0.5);
            } else if (gamepad1.right_trigger>0.2 && rightLinSlide.getCurrentPosition()<=4000){
                rightLinSlide.setPower(gamepad1.right_trigger*0.5);
            } else {
                rightLinSlide.setPower(0);
            }
        }
    }
    public static void moveStates (float rightTrigger, boolean leftBumper, boolean rightBumper, float leftTrigger, boolean a, boolean b){ //Left Bumper goes to lowest state, Right Bumper goes to medium state, Right Trigger goes to High state
        switch(state){ //define state machine
            case LOW:
                rightLinSlide.setPower(0); //Linslide power 0
                leftLinSlide.setPower(0);
                if (rightBumper){ // if right bumper clicked, go to mid state
                    state = states.TOMEDIUM;
                }
                else if (rightTrigger > 0.7){ // if right trigger clicked, go to high state
                    state = states.TOHIGH;
                }
                else if (leftTrigger > 0.7){
                    state = states.TOLOWOFF;
                }else if (leftBumper){
                    state = states.TOLOW;
                }/* else if(a){
                    mode=true;
                } else if(b){
                    mode = false;
                }*/
                break;
            case LOWOFF:
                if(rightTrigger > 0.7) {
                    state = states.TOHIGH;
                }
                else if(rightBumper){
                    state = states.TOMEDIUM;
                }
                else if(leftBumper){
                    state = states.TOLOW;
                }
                else if (leftTrigger > 0.7){
                    state = states.TOLOWOFF;
                }
                break;
            case MEDIUM:
                rightLinSlide.setPower(0);
                leftLinSlide.setPower(0);
                if(leftBumper){
                    state = states.TOLOW;
                }
                else if(rightTrigger > 0.7){
                    state = states.TOHIGH;
                }
                else if (leftTrigger > 0.7){
                    state = states.TOLOWOFF;
                }
                else if(rightBumper){
                    state = states.TOMEDIUM;
                } else{
                    if (rightLinSlide.getCurrentPosition()<mid-20){
                        state = states.TOMEDIUM;
                    }
                }
                break;
            case HIGH:
                rightLinSlide.setPower(0);
                leftLinSlide.setPower(0);
                if(rightBumper){
                    state = states.TOMEDIUM;
                }
                else if(leftBumper){
                    state = states.TOLOW;
                }
                else if(leftTrigger>0.7){
                    state = states.TOLOWOFF;
                }
                else if(rightTrigger > 0.7){
                    state = states.TOHIGH;
                } else{
                    if (rightLinSlide.getCurrentPosition()<high-20){
                        state = states.TOHIGH;
                    }
                }
                break;
            /*case CUSTOM:
                if(rightBumper){
                    state = states.TOMEDIUM;
                }
                else if(leftBumper){
                    state = states.TOLOW;
                }
                else if(leftTrigger>0.7){
                    state = states.TOLOWOFF;
                }
                else if(rightTrigger > 0.7){
                    state = states.TOHIGH;
                }*/
            case TOLOW:
                if(rightLinSlide.getCurrentPosition() > low /*&& leftLinSlide.getCurrentPosition()>low*/){ // if right lin slide and left lin slide encoder is more than 0, go to
                    goPosition(-power, low);
                }
                else{
                    state = states.LOW; // anything else, set state to low and power to 0
                    rightLinSlide.setPower(0);
                    leftLinSlide.setPower(0);
                }
                break;
            case TOLOWOFF:
                if(rightLinSlide.getCurrentPosition() > lowOff){
                    goPosition(power, lowOff);

                }
                else if(rightLinSlide.getCurrentPosition() < lowOff){
                    goPosition(-power, lowOff);

                }
                else{
                    state=states.LOWOFF;
                    rightLinSlide.setPower(0);
                    leftLinSlide.setPower(0);
                }
                break;
            case TOMEDIUM:
                if(leftBumper){
                    state=states.TOLOW;
                }
                else if(rightLinSlide.getCurrentPosition() < mid /*&& leftLinSlide.getCurrentPosition() < mid*/ ){
                    goPosition(power, mid);

                }
                else if (rightLinSlide.getCurrentPosition() > mid /*&& leftLinSlide.getCurrentPosition() > mid*/){
                    goPosition(-power, mid);

                }
                else {
                    state = states.MEDIUM;
                    rightLinSlide.setPower(0);
                    leftLinSlide.setPower(0);
                }
                break;
            case TOHIGH:
                if(leftBumper){
                    state=states.TOLOW;
                }
                else if(rightLinSlide.getCurrentPosition() < high){
                    goPosition(power, high);
                }
                else{
                    state = states.HIGH;
                    rightLinSlide.setPower(0);
                    leftLinSlide.setPower(0);
                }
                break;
        }

    }
    public static void goPosition (double power, int position){
        rightLinSlide.setTargetPosition(position);
        rightLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinSlide.setPower(power);
        leftLinSlide.setTargetPosition(-position);
        leftLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinSlide.setPower(-power);
    }

    public static class pieceTogetherButItMakesSense {

        public static void goOut() throws InterruptedException {
            if(rightLinSlide.getCurrentPosition() < 1400){
                moveStates(0, false,true, 0, false, false);
            }
            dualServoForearm.out();
            testHorizontalLinSlide.moveHorizontalLinManual(false, true);
        }

        public static void intakeToJunction() throws InterruptedException{

            pieceTogetherButItMakesSense.intakeToJunction();
            Thread.sleep(1000);
            ServoTele.open(true);
            Thread.sleep(2000);
            goOut();
            Thread.sleep(500);
            moveStates(0,true,false,0,false,false);
            Thread.sleep(800);
            clawServoClass.spinClawServo(true, false);
            Thread.sleep(200);
            moveStates(0,false,true, 0, false, false);
            Thread.sleep(400);

        }
    }
}

