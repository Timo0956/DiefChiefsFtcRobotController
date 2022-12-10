package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;

public class pieceTogether {
    public static void pieceTogether(Gamepad g1) throws InterruptedException {

        if(g1.b) {
            if(TwoStageLinSlideFileNew.state == TwoStageLinSlideFileNew.states.LOW || topMotor.TM.getCurrentPosition()!=0 ){
                TwoStageLinSlideFileNew.moveStates(0, false,false, 1, false, false);
                topMotor.autoMoveToOriginal();
                Thread.sleep(500);
            }
            ExtensionLinSlide.extendHori();
            servo180pullback.pullOut();
        }
        else if(g1.y){
            ServoTele.close(g1.y);
            servo180pullback.raiseUpOffGround();

        } else if (g1.x) {
            ServoTele.open(g1.x);

        } else if (g1.a) {
            if(TwoStageLinSlideFileNew.state == TwoStageLinSlideFileNew.states.LOW){
                TwoStageLinSlideFileNew.moveStates(0, false,false, 1, false, false);
                Thread.sleep(500);
            }
            load();
        }


    }
    public static void load() throws InterruptedException{
        ExtensionLinSlide.retractHori();
        servo180pullback.raiseIntoIntake();
        TwoStageLinSlideFileNew.moveStates(0, true,false, 0, false, false);
        clawServoClass.spinClawServo(true,false);
        TwoStageLinSlideFileNew.moveStates(0, false,false, 1, false, false);
        Thread.sleep(500);
        servo180pullback.setToZero();
        ServoTele.open(true);
    }
    public static void upDrop() throws InterruptedException{ //farm only
        TwoStageLinSlideFileNew.moveStates(0, true,false, 0, false, false);
        Thread.sleep(300);
        clawServoClass.spinClawServo(true, false);
        clawServoClass.spinClawServo(false, false);
        Thread.sleep(500);
        TwoStageLinSlideFileNew.moveStates(0, false,false, 1, false, false);
        topMotor.autoMoveToPosition();
        Thread.sleep(1000);
        clawServoClass.spinClawServo(false, true);
        clawServoClass.spinClawServo(false,false);
        Thread.sleep(300);
    }
    public static void armDown() throws InterruptedException{ // farm only
        topMotor.autoMoveToOriginal();
        Thread.sleep(300);
        TwoStageLinSlideFileNew.moveStates(0, false,false, 1, false, false);

    }
   /* public static void returnToPosition() throws InterruptedException{

    }*/
}
