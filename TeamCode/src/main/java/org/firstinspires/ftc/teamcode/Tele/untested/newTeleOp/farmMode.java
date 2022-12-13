package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.trash.ExtensionLinSlide;
import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;

public class farmMode {
    public static void farm() throws InterruptedException{
        ExtensionLinSlide.extendHori(); //includes opening claw
        ServoTele.close(true);
        Thread.sleep(500);
        pieceTogether.load();
        pieceTogether.upDrop();
        pieceTogether.armDown();
    }
}
