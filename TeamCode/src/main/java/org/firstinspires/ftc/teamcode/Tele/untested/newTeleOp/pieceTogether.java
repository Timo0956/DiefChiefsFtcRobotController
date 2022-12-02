package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;

public class pieceTogether {
    public static void pieceTogether(Gamepad g1) throws InterruptedException {

        ExtensionLinSlide.extendHori();
        ServoTele.close(g1.y);
        ExtensionLinSlide.retractHori();
        servo180pullback.placeHolder.setPosition(1);
        TwoStageLinSlideFileNew.moveStates(0, true,false, 0, false, false);
        clawServoClass.spinClawServo(true,false);
        TwoStageLinSlideFileNew.moveStates(0, false,false, 1, false, false);
        Thread.sleep(500);
        servo180pullback.placeHolder.setPosition(0);
        ServoTele.open(true);
    }
}
