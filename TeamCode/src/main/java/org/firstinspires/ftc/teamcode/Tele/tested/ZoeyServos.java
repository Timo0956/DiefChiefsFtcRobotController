package org.firstinspires.ftc.teamcode.Tele.tested;


import com.qualcomm.robotcore.hardware.Servo;

public class ZoeyServos {
    static Servo clawOne;
    static Servo clawTwo;
    static double startPos = 0;
    static double finalPos = 0.5;

    public static void setServos(Servo c1, Servo c2){
        clawOne = c1;
        clawTwo = c2;
    }
    public static void servoOpen(boolean x){
        if (x){
            clawOne.setPosition(startPos);
            clawTwo.setPosition(startPos);
        }
    }
    public static void servoClose(boolean y){
        if (y) {
            clawOne.setPosition(finalPos);
            clawTwo.setPosition(finalPos);
        }
    }
}
