package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.hardware.CRServo;


public class servo180pullback {

    static CRServo fourarmLowRight;
    static CRServo fourarmLowLeft;
    static CRServo fourarmHighRight;
    static CRServo fourarmHighLeft;

    static int modeOfFourBar = 0;



    public static void placeHolderServoInit(CRServo fourarmLR, CRServo fourarmLL, CRServo fourarmHR, CRServo fourarmHL) {
        fourarmLowLeft = fourarmLL;
        fourarmLowRight = fourarmLR;
        fourarmHighLeft = fourarmHL;
        fourarmHighRight = fourarmHR;

    }
    public static String raiseUpOffGround() throws InterruptedException{
        fourarmHighLeft.setPower(1);
        fourarmLowLeft.setPower(1);
        fourarmHighRight.setPower(-1);
        fourarmLowRight.setPower(-1);
        Thread.sleep(200);
        fourarmHighLeft.setPower(0);
        fourarmLowLeft.setPower(0);
        fourarmHighRight.setPower(0);
        fourarmLowRight.setPower(0);
        modeOfFourBar = 1;
        return "done";

    }
    public static void raiseIntoIntake() throws InterruptedException{
        fourarmHighLeft.setPower(1);
        fourarmLowLeft.setPower(1);
        fourarmHighRight.setPower(-1);
        fourarmLowRight.setPower(-1);
        Thread.sleep(600);
        fourarmHighLeft.setPower(0);
        fourarmLowLeft.setPower(0);
        fourarmHighRight.setPower(0);
        fourarmLowRight.setPower(0);
        modeOfFourBar = 2;

    }
    public static void pullOut() throws InterruptedException{
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

    }




}
