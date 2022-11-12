package org.firstinspires.ftc.teamcode.Tele.untested.servoStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.jvm.ClassWriter;

//edit if required to adapt to 2 servos
public class ServoTele {
    static Servo ClawL;
    static Servo ClawR;
    static double startPos = 0;
    static double closePos = 0.15;
    // Initializing both servos (Declared in main Teleop)


    public static void setServos(Servo Cl, Servo Cr)
    {
        ClawL = Cl;
        ClawR = Cr;
    }
    public static void open(boolean x){
        if(x) {
            ClawL.setPosition(startPos);
            ClawR.setPosition(startPos);
        }
    }

    //closes claw
    public static void close(boolean y){
        if(y) {
            ClawL.setPosition(closePos);
            ClawR.setPosition(closePos);
        }
    }

}
