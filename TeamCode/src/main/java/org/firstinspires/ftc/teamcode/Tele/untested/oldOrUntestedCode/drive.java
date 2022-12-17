package org.firstinspires.ftc.teamcode.Tele.untested.oldOrUntestedCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class drive {
    static DcMotor fl;
    static DcMotor bl;
    static DcMotor fr;
    static DcMotor br;

    public static void initDrive(DcMotor FL, DcMotor BL, DcMotor FR, DcMotor BR){
        fl=FL;
        fr=FR;
        br=BR;
        bl=BL;
    }
    public static void driveTele(Gamepad gamepad1){
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x ;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx)/ denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx)/ denominator;
        double backRightPower = (y + x - rx)/ denominator;
        fl.setPower(frontLeftPower*0.6);
        bl.setPower(backLeftPower*0.6);
        fr.setPower(frontRightPower*0.6);
        br.setPower(backRightPower*0.6);
    }
}
