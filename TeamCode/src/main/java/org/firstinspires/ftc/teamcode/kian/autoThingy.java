package org.firstinspires.ftc.teamcode.kian;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous
public class autoThingy extends LinearOpMode {
    static DcMotor motorFL;
    static DcMotor motorFR;
    static DcMotor motorBL;
    static DcMotor motorBR;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runMotorsTime(1, 1, 1, 1, 1000); //for ward
        runMotorsTime(-1, -1, -1, -1, 1000); //back ward
        runMotorsEncoder(1, 1, 1, 1, 10000); //for ward
    }

    public void runMotorsTime(double FLpower, double FRpower, double BLpower, double BRpower, long time) {
        motorFL.setPower(FLpower);
        motorFR.setPower(FRpower);
        motorBL.setPower(BLpower);
        motorBR.setPower(BRpower);
        sleep(time);
    }

    public void runMotorsEncoder(double FLpower, double FRpower, double BLpower, double BRpower, int distance) {
        motorFL.setTargetPosition(distance);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(motorFL.getCurrentPosition() != motorFL.getTargetPosition()) {
            motorFL.setPower(FLpower);
            motorFR.setPower(FRpower);
            motorBL.setPower(BLpower);
            motorBR.setPower(BRpower);
        }

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
}
