package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.HoriLinSlide.testHorizontalLinSlide;

public class pieceTogetherButItMakesSense {

    public static void load() throws InterruptedException {
        if(TwoStageLinSlideFileNew.rightLinSlide.getCurrentPosition() < 1400){
            TwoStageLinSlideFileNew.moveStates(0, false,true, 0, false, false);
        }
        dualServoForearm.out();
        testHorizontalLinSlide.moveHorizontalLinManual(false, true);
    }

}
