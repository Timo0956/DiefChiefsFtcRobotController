package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.trash;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;

public class ExtensionLinSlide {
    static DcMotor HL = null;
    static int extendPos = 1900;
    static int closePos = 0;
    static double power = 0.9;

    public static void initMotorsHoriLin (DcMotor HoriLin){
        HL = HoriLin;
        HL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public static void extendHori () {
        ServoTele.open(true);

            if (HL.getCurrentPosition() < extendPos) {
                HL.setTargetPosition(extendPos);
                HL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HL.setPower(power);
            } else {
                HL.setPower(0);
            }

    }
    public static void retractHori () {

            if(HL.getCurrentPosition() > closePos){
                HL.setTargetPosition(closePos);
                HL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HL.setPower(-power);
            }
            else{
                HL.setPower(0);
            }

    }
    public static void manualExtension(boolean a, boolean b){

    }

}
