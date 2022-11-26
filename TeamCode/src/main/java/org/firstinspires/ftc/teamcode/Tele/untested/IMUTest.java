package org.firstinspires.ftc.teamcode.Tele.untested;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@TeleOp
public class IMUTest extends LinearOpMode {

    static DcMotor motorFrontLeft = null;
    static DcMotor motorBackLeft = null;
    static DcMotor motorFrontRight = null;
    static DcMotor motorBackRight = null;
    final static double power = 1;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() throws InterruptedException{

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        gravity = imu.getLinearAcceleration();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("IMU Status", imu.getSystemStatus());
        telemetry.addData("Calibration Status", imu.getCalibrationStatus());

        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            //turn(power);
            //moveForwardBack(power);
            //moveSideways(power);

            halt();
        }


    }
    public void turn(double power) throws InterruptedException{
        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);

        for(int i=0; i<20; i++){
            telemetry.addData("Heading (Z)"+i*50, angles.firstAngle); //Z
            telemetry.addData("Roll (Y)"+i*50, angles.secondAngle); // Y
            telemetry.addData("Pitch (X)"+i*50, angles.thirdAngle); // X
            Log.v("Heading (Z)"+i*50,""+angles.firstAngle);
            Log.v("Roll (Y)"+i*50,""+angles.secondAngle);
            Log.v("Pitch (Z)"+i*50,""+angles.thirdAngle);
            telemetry.update();
            Thread.sleep(50);

        }
    }

    public void moveForwardBack(double power) throws InterruptedException{
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);

        for(int i=0; i<20; i++){
            telemetry.addData("Forward(Z)"+i*50, gravity.zAccel); //Z
//            telemetry.addData("Sideways(Y)"+i*50, gravity.yAccel); // Y
//            telemetry.addData("Up (X)"+i*50, gravity.xAccel); // X
            telemetry.update();
            Log.v("Forward(Z)"+i*50,""+gravity.zAccel);
            Thread.sleep(50);

        }
    }

    public void moveSideways(double power) throws InterruptedException{
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorBackRight.setPower(power);

        for(int i=0; i<20; i++){
//            telemetry.addData("Forward(Z)"+i*50, gravity.zAccel); //Z
            telemetry.addData("Sideways(Y)"+i*50, gravity.yAccel); // Y
//            telemetry.addData("Up (X)"+i*50, gravity.xAccel); // X
            Log.v("Sideways(Y)"+i*50,""+gravity.yAccel);
            telemetry.update();
            Thread.sleep(50);

        }
    }
    public void halt(){
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
}
