package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
public class testHorizontalLinSlide {
    static DcMotor linSlide;

    public enum states {in, out, goIn, goOut};
    public static states state = states.in;

    public void initHori(DcMotor HL){
        linSlide = HL;
    }

    public static void moveHorizontalLinManual(Boolean a, Boolean b) {
        switch(state) {
            case in:
                if(b) {
                    state = states.goOut;
                }
                break;

            case out:
                if(a) {
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
                if(linSlide.getCurrentPosition() < 2000) {
                    linSlide.setTargetPosition(2000);
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
