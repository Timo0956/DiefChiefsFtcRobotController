package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.HoriLinSlide.testHorizontalLinSlide;
import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;

public class pieceTogether {
    public static void pieceTogether(Gamepad g1) throws InterruptedException {

        if(g1.b) {
            if(TwoStageLinSlideFileNew.state != TwoStageLinSlideFileNew.states.MEDIUM || topMotor.TM.getCurrentPosition()!=0 ){
                TwoStageLinSlideFileNew.moveStates(0, false,true, 0, false, false);
                topMotor.autoMoveToOriginal();
                Thread.sleep(500);
            }
            testHorizontalLinSlide.moveHorizontalLinManual(false, true);
            servo180pullback.out();
        }
        else if(g1.y){
            ServoTele.close(g1.y);

        } else if (g1.x) {
            ServoTele.open(g1.x);

        } else if (g1.a) {
            if(TwoStageLinSlideFileNew.state != TwoStageLinSlideFileNew.states.MEDIUM || topMotor.TM.getCurrentPosition()!=0 ){
                TwoStageLinSlideFileNew.moveStates(0, false,true, 0, false, false);
                topMotor.autoMoveToOriginal();
                Thread.sleep(500);
            }
            load();
        }


    }
    public static void load() throws InterruptedException{
        testHorizontalLinSlide.moveHorizontalLinManual(true, false);
        servo180pullback.in();
        ServoTele.open(true);
        TwoStageLinSlideFileNew.moveStates(0, true,false, 0, false, false);
        clawServoClass.spinClawServo(true,false);
        Thread.sleep(500);
        TwoStageLinSlideFileNew.moveStates(0, false,false, 1, false, false);

    }
    public static void upDrop() throws InterruptedException{ //farm only
        //TwoStageLinSlideFileNew.moveStates(0, true,false, 0, false, false);
        //Thread.sleep(300);
        //clawServoClass.spinClawServo(true, false);
        //clawServoClass.spinClawServo(false, false);
        //Thread.sleep(500);
        TwoStageLinSlideFileNew.moveStates(0, false,false, 1, false, false);
        topMotor.autoMoveToPosition();
        farmMode.horiFarm();
        //Thread.sleep(800);
        clawServoClass.spinClawServo(false, true);
        clawServoClass.spinClawServo(false,false);
        Thread.sleep(300);
    }
    public static void armDown() throws InterruptedException{ // farm only
        topMotor.autoMoveToOriginal();
        Thread.sleep(300);
        TwoStageLinSlideFileNew.moveStates(0, false,true, 0, false, false);
        load();

    }
   /* public static void returnToPosition() throws InterruptedException{

    }*/
}
