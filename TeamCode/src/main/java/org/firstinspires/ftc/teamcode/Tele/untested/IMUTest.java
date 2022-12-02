package org.firstinspires.ftc.teamcode.Tele.untested;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Func;
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

        gravity = imu.getAcceleration();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("IMU Status", imu.getSystemStatus());
        telemetry.addData("Calibration Status", imu.getCalibrationStatus());
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.x){
                turn(power);
            } else if(gamepad1.b){
                moveForwardBack(power);
            } else if (gamepad1.a){
                moveSideways(power);
            }

           // halt();
        }


    }
    public void turn(double power) throws InterruptedException{
        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
        telemetry.addData("Status", " Run Turn");
        telemetry.update();
        for(int i=0; i<50; i++){
            telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.thirdAngle);
                        }
                    });
          //  telemetry.addData("Heading (Z)"+i*20, angles.firstAngle); //Z
         //   telemetry.addData("Roll (Y)"+i*20, angles.secondAngle); // Y
            //telemetry.addData("Pitch (X)"+i*20, angles.thirdAngle); // X
           // Log.v("Heading (Z)"+i*20,""+angles.firstAngle);
           // Log.v("Roll (Y)"+i*20,""+angles.secondAngle);
            //Log.v("Pitch (Z)"+i*20,""+angles.thirdAngle);
//            telemetry.update();
            Thread.sleep(20);

        }
        halt();
    }

    public void moveForwardBack(double power) throws InterruptedException{
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
        telemetry.addData("Status", " Forward Backward");
        telemetry.update();
        for(int i=0; i<50; i++){
            telemetry.addLine()
                    .addData("grvty", new Func<String>() {
                        @Override public String value() {
                            return gravity.toString();
                        }
                    })
                    .addData("mag", new Func<String>() {
                        @Override public String value() {
                            return String.format(Locale.getDefault(), "%.3f", gravity.zAccel);
                        }
                    });
//            telemetry.addData("Forward(Z)"+i*20, gravity.zAccel); //Z
//            telemetry.addData("Sideways(Y)"+i*50, gravity.yAccel); // Y
//            telemetry.addData("Up (X)"+i*50, gravity.xAccel); // X
//            telemetry.update();
          //  Log.v("Forward(Z)"+i*20,""+gravity.zAccel);
            Thread.sleep(20);

        }
        halt();
    }

    public void moveSideways(double power) throws InterruptedException{
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorBackRight.setPower(power);

        for(int i=0; i<50; i++){
            telemetry.addLine()
                    .addData("grvty", new Func<String>() {
                        @Override public String value() {
                            return gravity.toString();
                        }
                    })
                    .addData("mag", new Func<String>() {
                        @Override public String value() {
                            return String.format(Locale.getDefault(), "%.3f", gravity.yAccel);
                        }
                    });
//            telemetry.addData("Forward(Z)"+i*50, gravity.zAccel); //Z
//            telemetry.addData("Sideways(Y)"+i*20, gravity.yAccel); // Y
//       //     telemetry.addData("Up (X)"+i*50, gravity.xAccel); // X
//            Log.v("Sideways(Y)"+i*20,""+gravity.yAccel);
//            telemetry.update();
            Thread.sleep(20);

        }
        halt();
    }
    public void halt(){
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
