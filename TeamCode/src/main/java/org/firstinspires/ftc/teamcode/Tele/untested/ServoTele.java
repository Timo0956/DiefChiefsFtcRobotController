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
        startPosL = ClawL.getPosition();
        closePosL = ClawL.getPosition() - 0.5;
        startPosR = ClawR.getPosition();
        closePosR = ClawR.getPosition() + 0.5;
        // Reversed because of flipped servo

        open();

        runTime = new ElapsedTime();
    }

    
    //opens claw when x is pressed
    public static void release(boolean x)
    {
        if(x){
             open();  
        }
    }
    //closes claw when y is pressed
    public static void hold(boolean y)
    {
        if(y){
            close();
        }
    }
    
    public static void open()
    {
        ClawL.setPosition(startPos);
        ClawR.setPosition(startPos);
    }
    
    public static void close()
    {
        ClawL.setPosition(closePos);
        ClawR.setPosition(closePos);
    }
}
