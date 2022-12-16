package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.hardware.Servo;

public class dualServoForearm {

    static Servo servo3A; //Right side attached to claw
    static Servo servo2A; //Left side attached to slide

    static double positionWhileOut = 0.65;
    static double positionWhileIn = 0;

    public static void initForearmServos(Servo servo3a, Servo servo2a) {
        servo3A = servo3a;
        servo2A = servo2a;
    }

    public static void out() {
        servo2A.setPosition(positionWhileOut);
    }

    public static void in() {
        servo2A.setPosition(positionWhileIn);
    }

}
