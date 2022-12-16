package org.firstinspires.ftc.teamcode.Tele.untested.oldOrUntestedCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class newFarm {
    static DcMotor fl;
    static DcMotor bl;
    static DcMotor fr;
    static DcMotor br;
    static DcMotor rls;
    static DcMotor lls;
    static Servo lc;
    static Servo rc;

    public static void initFarmNew(DcMotor FL, DcMotor BL, DcMotor FR, DcMotor BR, DcMotor RLS, DcMotor LLS, Servo LC, Servo RC){
        fl = FL;
        fr = FR;
        bl = BL;
        br = BR;
        lls = LLS;
        rls = RLS;
        lc = LC;
        rc = RC;
    }

    public static void farmFromPark(){

    }
}
