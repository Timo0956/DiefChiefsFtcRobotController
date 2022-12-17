package org.firstinspires.ftc.teamcode.Tele.untested.servoStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.jvm.ClassWriter;

import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.TwoStageLinSlideFileNew;
import org.firstinspires.ftc.teamcode.Tele.untested.oldOrUntestedCode.newFarm;

//edit if required to adapt to 2 servos
public class ServoTele {
    static Servo ClawL;
    static Servo ClawR;
    static double startPos = 0;
    static double closePos = 0.28;
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
                Thread.sleep(400);
                TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.TOLOW;
                //TwoStageLinSlideFileNew.moveStates(0,true,false, 0, false, false);
                Thread.sleep(400);
            }
        }
    }
    //closes claw
    public static void close(boolean y) throws InterruptedException {
        if(y) {
            ClawL.setPosition(closePos);
            ClawR.setPosition(closePos);
            Thread.sleep(400);
            TwoStageLinSlideFileNew.goPosition(1,350);
            Thread.sleep(400);
        }
    }

}
