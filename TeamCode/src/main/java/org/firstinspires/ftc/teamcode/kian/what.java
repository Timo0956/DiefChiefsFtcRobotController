package org.firstinspires.ftc.teamcode.kian;


import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class what extends LinearOpMode {
    static DcMotor motorFL;
    static DcMotor motorFR;
    static DcMotor motorBL;
    static DcMotor motorBR;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = true;

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        gravity = imu.getLinearAcceleration();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        waitForStart();

        while(opModeIsActive()) {
            drive(-gamepad1.right_stick_y);
            turn(gamepad1.left_stick_x);
            slide(gamepad1.right_stick_x);
        }
    }

    public void slide(double speed) throws InterruptedException {
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(-speed);
        motorBR.setPower(-speed);

        for(int i = 0; i < 500; i++) {
            Log.v("X", "" + angles.thirdAngle);
            Log.v("Y", "" + angles.secondAngle);
            Log.v("Z", "" + angles.firstAngle);

            Thread.sleep(20);
        }


    }

    public void turn(double speed) throws InterruptedException {
        motorFL.setPower(speed);
        motorFR.setPower(-speed);
        motorBL.setPower(speed);
        motorBR.setPower(-speed);

        for(int i = 0; i < 500; i++) {
            Log.v("X", "" + angles.thirdAngle);
            Log.v("Y", "" + angles.secondAngle);
            Log.v("Z", "" + angles.firstAngle);

            Thread.sleep(20);
        }
    }

    public void drive(double speed) throws InterruptedException {
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);

        for(int i = 0; i < 500; i++) {
            Log.v("X", "" + angles.thirdAngle);
            Log.v("Y", "" + angles.secondAngle);
            Log.v("Z", "" + angles.firstAngle);

            Thread.sleep(20);
        }
    }
}
