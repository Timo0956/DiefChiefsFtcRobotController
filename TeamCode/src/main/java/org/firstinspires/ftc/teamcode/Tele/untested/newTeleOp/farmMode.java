package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.HoriLinSlide.testHorizontalLinSlide;
import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;

public class farmMode {
    public static void initFarm() throws InterruptedException{
        horiFarm();
        Thread.sleep(500);
        pieceTogether.load();

    }
    public static void farm() throws InterruptedException{

        pieceTogether.upDrop();
        pieceTogether.armDown();
        Thread.sleep(250);
    }
    public static void horiFarm() throws InterruptedException{
        if(TwoStageLinSlideFileNew.state != TwoStageLinSlideFileNew.states.MEDIUM || topMotor.TM.getCurrentPosition()!=0 ){
            TwoStageLinSlideFileNew.moveStates(0, false,true, 0, false, false);
            topMotor.autoMoveToOriginal();
            Thread.sleep(500);
        }
        testHorizontalLinSlide.moveHorizontalLinManual(false, true); //includes opening claw
        Thread.sleep(700);
        ServoTele.close(true,350);
    }
}
