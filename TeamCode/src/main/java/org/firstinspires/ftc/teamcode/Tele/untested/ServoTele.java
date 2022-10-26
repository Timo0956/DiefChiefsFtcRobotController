package org.firstinspires.ftc.teamcode.Tele.untested;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.jvm.ClassWriter;

public class ServoTele {

    static Servo ClawL;
    static Servo ClawR;
    static double startPos;
    static double closePos;

    static ElapsedTime runTime = null;
    
    //initializing both servos
    public static void setServos(Servo Cl, Servo Cr)
    {
        ClawL = Cl;
        ClawR = Cr;

        // Sets start and closed positions for the claws
        double startPosL = 0;
        double closePosL = ClawL.getPosition() - 0.5;
        double startPosR = 0;
        double closePosR = ClawR.getPosition() + 0.5;
        // Reversed because of flipped servo

        release(true);

        runTime = new ElapsedTime();
    }

    
    //opens claws when x is pressed
    public static void release(boolean x)
    {
        if(x){
            ClawL.setPosition(startPos);
            ClawR.setPosition(startPos);
        }
    }

    //closes claws when y is pressed
    public static void hold(boolean y)
    {
        if(y){
            ClawL.setPosition(closePos);
            ClawR.setPosition(closePos);
        }
    }

}
