package org.firstinspires.ftc.teamcode.Tele.tested;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class ZoeyTest extends LinearOpMode{
    //declaring variables for motors
    static DcMotor motorFrontLeft;
    static DcMotor motorFrontRight;
    static DcMotor motorBackLeft;
    static DcMotor motorBackRight;

    //declares variables for IMU
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() throws InterruptedException{

        //parameters in IMU
        BNO055IMU.Parameters imuPar = new BNO055IMU.Parameters();
        imuPar.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuPar.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuPar.loggingEnabled = true;

        //hardware mapping IMU + parameters are passed
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuPar);

        //gravity is equal to acceleration, angles are equal to movement
        gravity = imu.getLinearAcceleration();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //hardware mapping motors
        //names need to match up exactly on the phone
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        //when the power is set to 0, motors wont move
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.x){

            }
        }



    }
}
