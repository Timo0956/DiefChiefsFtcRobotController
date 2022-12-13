package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.topMotorFiles;

import com.qualcomm.robotcore.hardware.Servo;

public class testFile2 {
    static Servo Left1 = null;
    public static void init(Servo L1){
        Left1 = L1;
    }
    public static void turnOpen(){
        Left1.setPosition(1);
    }
    public static void turnClosed(){
        Left1.setPosition(0);
    }
}
