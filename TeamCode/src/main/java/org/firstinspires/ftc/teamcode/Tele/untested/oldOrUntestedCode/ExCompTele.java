package org.firstinspires.ftc.teamcode.Tele.untested.oldOrUntestedCode;

import static org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele.setServos;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.Tele.tested.initialize2023;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Tele.untested.linSlideFiles.TwoStageLinSlideFile;
import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.TwoStageLinSlideFileNew;
import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;

import java.util.Locale;

@TeleOp
public class ExCompTele extends LinearOpMode {
    BNO055IMU imu; //Gets the IMU
    Acceleration acceleration = new Acceleration(); //Gets the acceleration
    Orientation angles = new Orientation(); //Gets the heading in degrees

    @Override
    public void runOpMode() throws InterruptedException{

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        DcMotor rightLinSlide = hardwareMap.dcMotor.get("rightLinSlide"); //defines our motors for LinSlide
        DcMotor leftLinSlide = hardwareMap.dcMotor.get("leftLinSlide");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        Servo ClawServoL = hardwareMap.servo.get("clawServoL");
        Servo ClawServoR = hardwareMap.servo.get("clawServoR");

        //set zero power behavior to brake
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        acceleration = imu.getLinearAcceleration();

        drive.initDrive(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight);
        setServos(ClawServoL, ClawServoR);
        TwoStageLinSlideFileNew.setLSMotor(rightLinSlide,leftLinSlide);//defines motors in terms of the seperate file
        newFarm.initFarmNew(imu,acceleration,angles,motorFrontLeft,motorBackLeft,motorFrontRight,motorBackRight,rightLinSlide,leftLinSlide);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){
            composeTelemetry();
            telemetry.addData("Position", rightLinSlide.getCurrentPosition());
            telemetry.addData("ServoPositionR", ClawServoR.getPosition());
            telemetry.addData("ServoPositionL", ClawServoL.getPosition());
            telemetry.update();

            newFarm.farmFromPark(gamepad1.a);
            TwoStageLinSlideFileNew.linSlideDouble(gamepad1); //takes gamepad input
            ServoTele.open(gamepad1.x);
            ServoTele.close(gamepad1.y);
            drive.driveTele(gamepad1);

            
        }
    }
    public void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            acceleration  = imu.getLinearAcceleration();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

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

        telemetry.addLine()
                .addData("acc", new Func<String>() {
                    @Override public String value() {
                        return acceleration.toString();
                    }
                })
                .addData("Z", new Func<String>() {
                    @Override public String value() {
                        Log.v("zac",""+acceleration.zAccel);
                        return String.format(Locale.getDefault(), "%.3f",acceleration.zAccel);
                    }
                })
                .addData("Y", new Func<String>() {
                    @Override public String value() {
                        Log.v("yac",""+acceleration.zAccel);
                        return String.format(Locale.getDefault(), "%.3f",acceleration.yAccel);
                    }
                })
                .addData("X", new Func<String>() {
                    @Override public String value() {
                        Log.v("xac",""+acceleration.zAccel);
                        return String.format(Locale.getDefault(), "%.3f",acceleration.xAccel);
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
