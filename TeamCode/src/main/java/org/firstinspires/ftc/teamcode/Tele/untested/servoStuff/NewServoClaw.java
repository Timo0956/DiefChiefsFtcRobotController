package org.firstinspires.ftc.teamcode.Tele.untested.servoStuff;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tele.untested.COMPCODEDON.newFarm;
import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.TwoStageLinSlideFileNew;

public class NewServoClaw {
    static Servo Rotate;
    public static void initServo(Servo servo) throws InterruptedException {
        Rotate = servo;

    }
    public static void openPosition(boolean x) throws InterruptedException {
        if(x) {
            Rotate.setPosition(1);
            //Thread.sleep(200);
            // ExCompTele.moveBackward(400);
            //newFarm.moveFB(-0.5, 200);
            if (TwoStageLinSlideFileNew.getLinPosition() > 450){
                //newFarm.pause(300);
                //TwoStageLinSlideFileNew.goPosition(-1,450);
                TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.TOLOW;
                //TwoStageLinSlideFileNew.moveStates(0,true,false, 0, false, false);
                Thread.sleep(400);
            }
        }
    }
    //closes claw
    public static void closePosition(boolean y, int height) throws InterruptedException {
        if(y) {

            TwoStageLinSlideFileNew.goPosition(-0.8,0);
            TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.CUSTOM2;
            Thread.sleep(300);
            Rotate.setPosition(0.2);
            newFarm.pause(600);
            TwoStageLinSlideFileNew.goPosition(-1,height);
            TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.LOW;
            Thread.sleep(200);
        }
    }




}
