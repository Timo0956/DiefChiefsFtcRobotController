package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


public class servo180pullback {

    //static CRServo fourarmLowRight;
    static CRServo fourarmLowLeft;
    // static CRServo fourarmHighRight;
    static Servo fourarmHighLeft;

    static double outPos = 0;
    static double inPos = -1;



    public static void placeHolderServoInit(CRServo fourarmLL, Servo fourarmHL) {
        fourarmLowLeft = fourarmLL;
        //fourarmLowRight = fourarmLR;
        fourarmHighLeft = fourarmHL;
        //fourarmHighRight = fourarmHR;

    }
    public static void out() throws InterruptedException{
        fourarmHighLeft.setPosition(outPos);
        fourarmLowLeft.setPower(1);
        //fourarmHighRight.setPower(-1);
        //fourarmLowRight.setPower(-1);
        Thread.sleep(300);
        fourarmLowLeft.setPower(0);
        //fourarmHighRight.setPower(0);
        //fourarmLowRight.setPower(0);

    }
    public static void in() throws InterruptedException{
        fourarmHighLeft.setPosition(inPos);
        fourarmLowLeft.setPower(-1);
        //fourarmHighRight.setPower(-1);
        //fourarmLowRight.setPower(-1);
        Thread.sleep(300);
        fourarmLowLeft.setPower(0);
        //fourarmHighRight.setPower(0);
        //fourarmLowRight.setPower(0);

    }
   /* public static void pullOut() throws InterruptedException{
        fourarmHighLeft.setPower(1);
        fourarmLowLeft.setPower(1);
        fourarmHighRight.setPower(-1);
        //fourarmLowRight.setPower(-1);
        Thread.sleep(300);
        fourarmHighLeft.setPower(0);
        fourarmLowLeft.setPower(0);
        fourarmHighRight.setPower(0);
      //  fourarmLowRight.setPower(0);
        modeOfFourBar = 3;
    }
    public static void setToZero() throws InterruptedException{
        if(modeOfFourBar == 1){
            fourarmHighLeft.setPower(-1);
            fourarmLowLeft.setPower(-1);
            fourarmHighRight.setPower(1);
            //fourarmLowRight.setPower(1);
            Thread.sleep(200);
            fourarmHighLeft.setPower(0);
            fourarmLowLeft.setPower(0);
            fourarmHighRight.setPower(0);
          //  fourarmLowRight.setPower(0);

        }
        else if (modeOfFourBar == 2){
            fourarmHighLeft.setPower(-1);
            fourarmLowLeft.setPower(-1);
            fourarmHighRight.setPower(1);
            fourarmLowRight.setPower(1);
            Thread.sleep(600);
            fourarmHighLeft.setPower(0);
            fourarmLowLeft.setPower(0);
            fourarmHighRight.setPower(0);
            fourarmLowRight.setPower(0);
        }
        else if (modeOfFourBar == 3){
            fourarmHighLeft.setPower(-1);
            fourarmLowLeft.setPower(-1);
            fourarmHighRight.setPower(1);
            fourarmLowRight.setPower(1);
            Thread.sleep(800);
            fourarmHighLeft.setPower(0);
            fourarmLowLeft.setPower(0);
            fourarmHighRight.setPower(0);
            fourarmLowRight.setPower(0);
        }
        else{
            fourarmHighLeft.setPower(0);
            fourarmLowLeft.setPower(0);
            fourarmHighRight.setPower(0);
            fourarmLowRight.setPower(0);
        }
        modeOfFourBar = 0;

    }*/




}
