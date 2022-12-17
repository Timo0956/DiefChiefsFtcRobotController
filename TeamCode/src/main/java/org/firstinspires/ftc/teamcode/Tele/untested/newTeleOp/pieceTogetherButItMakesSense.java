package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.HoriLinSlide.testHorizontalLinSlide;
import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;

public class pieceTogetherButItMakesSense {

    public static void goOut() throws InterruptedException {
        if(TwoStageLinSlideFileNew.rightLinSlide.getCurrentPosition() < 1400){
            TwoStageLinSlideFileNew.moveStates(0, false,true, 0, false, false);
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
        TwoStageLinSlideFileNew.moveStates(0,true,false,0,false,false);
        Thread.sleep(800);
        clawServoClass.spinClawServo(true, false);
        Thread.sleep(200);
        TwoStageLinSlideFileNew.moveStates(0,false,true, 0, false, false);
        Thread.sleep(400);

    }
}
