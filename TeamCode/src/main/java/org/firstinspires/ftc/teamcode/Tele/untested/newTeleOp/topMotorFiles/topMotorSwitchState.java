package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.topMotorFiles;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class topMotorSwitchState {

    public enum states {OPEN, CLOSE, TOOPEN, TOCLOSE} //state array for state machine
    public static states state = states.CLOSE;
    static DcMotor topMotor1 = null;
    public static void initTopMotor(DcMotor TopMotor2){
        topMotor1 = TopMotor2;
        topMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public static void moveTopMotorStates(boolean open, boolean close) throws InterruptedException{
        switch (state){
            case CLOSE:
                topMotor1.setPower(0);
                if(open){
                    state = states.TOOPEN;
                }
                break;
            case OPEN:
                topMotor1.setPower(0);
                if(close){
                    state = states.TOCLOSE;
                }
                break;
            case TOCLOSE:
                moveMotor(0.3, 0);
                state = states.CLOSE;
                break;
            case TOOPEN:
                moveMotor(-0.3, -225);
                state = states.OPEN;
        }

    }
    public static void moveMotor(double power, int position) throws InterruptedException{
        topMotor1.setTargetPosition(position);
        topMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topMotor1.setPower(power);

    }
}
