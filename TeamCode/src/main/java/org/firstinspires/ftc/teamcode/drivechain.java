package org.firstinspires.ftc.teamcode.Tele.untested;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class drivechain {
    static DcMotor motorFrontLeft = null;
    static DcMotor motorFrontRight = null;
    static DcMotor motorBackLeft = null;
    static DcMotor motorBackRight = null;

    public static void setDTMotors(DcMotor FL,DcMotor FR,DcMotor BL,DcMotor BR){
        motorFrontLeft=FL;
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight=FR;
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft = BL;
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight = BR;
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void controllerInput(Gamepad gamepad1){
        drive(gamepad1.left_stick_x, gamepad1.left_stick_y);
    }

    public static void drive(double joystickX, double joystickY){

    }


}
