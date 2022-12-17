package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.HoriLinSlide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.TwoStageLinSlideFileNew;
import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.dualServoForearm;
import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.pieceTogether;
import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.pieceTogetherButItMakesSense;
import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.topMotor;
import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;

public class testHorizontalLinSlide {
    static DcMotor linSlide;

    public enum states {in, mid, out, goIn, goMid, goOut};
    public static states state = states.in;

    public static void initHori(DcMotor HL){
        linSlide = HL;
        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public static void moveHorizontalLinManual(Boolean a, Boolean b) throws InterruptedException {
        switch(state) {
            case in:
                if(b) {
                    if(TwoStageLinSlideFileNew.state != TwoStageLinSlideFileNew.states.MEDIUM || topMotor.TM.getCurrentPosition()!=0 ){
                        TwoStageLinSlideFileNew.moveStates(0, false,true, 0, false, false);
                        topMotor.autoMoveToOriginal();
                        Thread.sleep(1000);
                    }
                    dualServoForearm.out();
                    state = states.goMid;
                }
                break;
            case mid:
                if(b){
                    state = states.goOut;
                } else if (a){
                    if(TwoStageLinSlideFileNew.state != TwoStageLinSlideFileNew.states.MEDIUM || topMotor.TM.getCurrentPosition()!=0 ){
                        TwoStageLinSlideFileNew.moveStates(0, false,true, 0, false, false);
                        topMotor.autoMoveToOriginal();
                        state = states.goIn;
                        Thread.sleep(1000);
                    } else{
                        state = states.goIn;
                        Thread.sleep(1000);
                        //pieceTogether.load();
                    }
                    //Thread.sleep(1000);

                    pieceTogetherButItMakesSense.intakeToJunction();


                }
                break;
            case out:
                if(a) {
                    if(TwoStageLinSlideFileNew.state != TwoStageLinSlideFileNew.states.MEDIUM || topMotor.TM.getCurrentPosition()!=0 ){
                        TwoStageLinSlideFileNew.moveStates(0, false,true, 0, false, false);
                        topMotor.autoMoveToOriginal();
                        state = states.goIn;
                        Thread.sleep(1000);
                    } else{
                        state = states.goIn;
                        Thread.sleep(1000);
                        //pieceTogether.load();
                    }
                    pieceTogetherButItMakesSense.intakeToJunction();
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

            case goMid:
                if(linSlide.getCurrentPosition() < 1000) {
                    linSlide.setTargetPosition(1000);
                    linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linSlide.setPower(1);
                } else if(linSlide.getCurrentPosition() > 1000) {
                    linSlide.setTargetPosition(1000);
                    linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linSlide.setPower(-1);
                } else {
                    linSlide.setPower(0);
                    state = states.mid;
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
