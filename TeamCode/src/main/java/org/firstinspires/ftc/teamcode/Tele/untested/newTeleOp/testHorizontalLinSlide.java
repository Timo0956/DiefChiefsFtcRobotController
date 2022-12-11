package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class testHorizontalLinSlide extends LinearOpMode {
    static DcMotor linSlide;
    static int counter;
    static Boolean isOut;
    @Override

    public void runOpMode() throws InterruptedException {
        linSlide = hardwareMap.dcMotor.get("Horizontal LinSlide");
        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        counter = 0;
        isOut = false;

        while(opModeIsActive()) {
            if(counter == 0) {
                moveHorizontalLinAuto();
                moveHorizontalLinAuto();
                counter++;
            }
            moveHorizontalLinManual(gamepad1.left_bumper, gamepad1.right_bumper); //Placeholder Buttons
        }
    }

    public static void moveHorizontalLinAuto() {
        if(isOut) {
            linSlide.setTargetPosition(0);
            linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linSlide.setPower(-1);
            isOut = false;
        } else {
            linSlide.setTargetPosition(1000);
            linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linSlide.setPower(1);
            isOut = true;
        }
    }

    public enum states {in, out, goIn, goOut};
    public static states state = states.in;

    public static void moveHorizontalLinManual(Boolean left, Boolean right) {
        switch(state) {
            case in:
                if(right) {
                    state = states.goOut;
                }
                break;

            case out:
                if(left) {
                    state = states.goIn;
                }
                break;

            case goIn:
                if(linSlide.getCurrentPosition() > 0) {
                    linSlide.setTargetPosition(0);
                    linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linSlide.setPower(-1);

                } else {
                    linSlide.setPower(0);
                    state = states.in;
                }
                break;

            case goOut:
                if(linSlide.getCurrentPosition() < 1000) {
                    linSlide.setTargetPosition(1000);
                    linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linSlide.setPower(1);

                } else {
                    linSlide.setPower(0);
                    state = states.out;
                }
                break;
        }

    }

    /*public enum state {
        in {
            @Override
            public state in() {

            }
        }

        out {
            @Override
                    public state out() {

            }
        }

        goIn {
            @Override
                    public state goIn() {

            }
        }

        goOut {
            @Override
                    public state goOut() {

            }
        }
    }*/
}
