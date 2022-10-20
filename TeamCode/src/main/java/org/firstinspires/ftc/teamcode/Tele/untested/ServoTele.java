package org.firstinspires.ftc.teamcode.Tele.untested;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//edit if required to adapt to 2 servos
public class ServoTele {
    static Servo ClawL;
   // static Servo ClawR;
    static double startPos = 1;
    static double closePos = 0.2; // adjust maybe
    //static servoController sCont = null;
    static ElapsedTime runTime = null;
    
    //initializing servos
    public static void setServos(Servo Cl/*, Servo Cr*/){
        ClawL = Cl;
        //ClawR = Cr;
        open();
        
     //   sCont = ClawL.getController(); //should declare for both servos
        
   //     sCont.pwmEnable(); //enabling just in case
        
        runTime = new ElapsedTime();
        
    }
    
    //releases object when x is pressed
    public static void release(boolean x){
        if(ClawL.getPosition() == -closePos /*&& ClawR.getPosition() == closePos*/ && x){
             open();  
        }
    }
    //holds object when y is pressed
    public static void hold(boolean y){
        if(ClawL.getPosition() == -startPos /*&& ClawR.getPosition() == startPos*/ && y){
            close();
        }
    }
    
    public static void open(){
        ClawL.setPosition(-startPos);
    //    ClawR.setPosition(startPos);
    }
    
    public static void close(){
        ClawL.setPosition(-closePos);
   //     ClawR.setPosition(closePos);
    }
}
