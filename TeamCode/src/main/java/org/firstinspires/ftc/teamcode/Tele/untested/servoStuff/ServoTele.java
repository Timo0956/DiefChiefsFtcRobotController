package org.firstinspires.ftc.teamcode.Tele.untested.servoStuff;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.TwoStageLinSlideFileNew;
import org.firstinspires.ftc.teamcode.Tele.untested.COMPCODEDON.newFarm;

//edit if required to adapt to 2 servos
public class ServoTele {
    static Servo ClawL;
    static Servo ClawR;
    static double startPos = 0;
    static double closePos = 0.32;
    // Initializing both servos (Declared in main Teleop)


    public static void setServos(Servo Cl, Servo Cr)
    {
        ClawL = Cl;
        ClawR = Cr;
    }
    public static void open(boolean x) throws InterruptedException {
        if(x) {
            ClawL.setPosition(startPos);
            ClawR.setPosition(startPos);

            //newFarm.moveFB(-0.5, 200);
            if (TwoStageLinSlideFileNew.state != TwoStageLinSlideFileNew.states.LOW){
                newFarm.pause(300);
                TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.TOLOW;
                //TwoStageLinSlideFileNew.moveStates(0,true,false, 0, false, false);
                Thread.sleep(200);
            }
        }
    }
    //closes claw
    public static void close(boolean y, int height) throws InterruptedException {
        if(y) {
            ClawL.setPosition(closePos);
            ClawR.setPosition(closePos);
            newFarm.pause(300);
            TwoStageLinSlideFileNew.goPosition(1,height);
            TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.LOWOFF;

            Thread.sleep(200);
        }
    }

}
