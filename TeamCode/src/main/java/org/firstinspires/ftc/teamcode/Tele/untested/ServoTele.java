package org.firstinspires.ftc.teamcode.Tele.untested;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.jvm.ClassWriter;

//edit if required to adapt to 2 servos
public class ServoTele {

    static Servo ClawL;
    static Servo ClawR;

    static double startPosL;
    static double closePosL;

    static double startPosR;
    static double closePosR;


    static ElapsedTime runTime = null;
    
    // Initializing both servos (Declared in main Teleop)
    public static void setServos(Servo Cl, Servo Cr){
        ClawL = Cl;
        ClawR = Cl;

        startPosL = ClawL.getPosition();
        closePosL = ClawL.getPosition()+0.3;
    // Reversed positions because of mirrored servos
        startPosR = ClawR.getPosition();
        closePosR = ClawR.getPosition()-0.3;

        open();

        runTime = new ElapsedTime();
        
    }

    
    //releases object when x is pressed
    public static void release(boolean x){
        open();
    }

    //closes claw
    public static void hold(boolean y){
        close();
    }

    public static void open(){
        ClawL.setPosition(startPosL);
        ClawR.setPosition(startPosR);
    }

    //closes claw
    public static void close(){
        ClawL.setPosition(closePosL);
        ClawR.setPosition(closePosR);
    }

}
