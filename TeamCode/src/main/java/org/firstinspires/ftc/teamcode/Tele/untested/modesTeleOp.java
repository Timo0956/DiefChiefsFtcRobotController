package org.firstinspires.ftc.teamcode.Tele.untested;

import static org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele.setServos;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Tele.untested.linSlideFiles.TwoStageLinSlideFile;
import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;

@TeleOp
public class modesTeleOp extends LinearOpMode {
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    @Override

    public void runOpMode() throws InterruptedException{
        DcMotor rightLinSlide = hardwareMap.dcMotor.get("rightLinSlide"); //defines our motors for LinSlide
        //DcMotor leftLinSlide = hardwareMap.dcMotor.get("leftLinSlide");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo ClawServoL = hardwareMap.servo.get("clawServoL");
        Servo ClawServoR = hardwareMap.servo.get("clawServoR");
        setServos(ClawServoL, ClawServoR);
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        int Modes = 1;
        int slideModes = 1;

        TwoStageLinSlideFile.setLSMotor(rightLinSlide/*, leftLinSlide*/); //defines motors in terms of the seperate file
        waitForStart();
        while(opModeIsActive()){
            if(gamepad2.a){
                Modes = 1;
            }
            if(gamepad2.b){
                Modes = 2;
            }
            if(gamepad2.x && rightLinSlide.getCurrentPosition() <= 0){
                slideModes = 1;
            }
            if(gamepad2.y && rightLinSlide.getCurrentPosition() <=0){
                slideModes = 2;
            }
            if(Modes == 1){
                telemetry.addData("Mode = ", "Manual");
                telemetry.update();
                if(slideModes == 1){
                    telemetry.addData("Slide Mode = ", "Switch State");
                    telemetry.update();
                    TwoStageLinSlideFile.linSlideDouble(gamepad1); //takes gamepad input
                }
                if (slideModes == 2){
                    telemetry.addData("Slide Mode = ", "Manual");
                    telemetry.update();
                    if (gamepad1.right_trigger > 0.3&&rightLinSlide.getCurrentPosition() <=4000){
                        rightLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightLinSlide.setPower(1);
                    }
                    if(gamepad1.left_trigger>0.3 && rightLinSlide.getCurrentPosition() >=0){
                        rightLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightLinSlide.setPower(1);
                    }
                    if(gamepad1.right_bumper &&rightLinSlide.getCurrentPosition() <=4000){
                        rightLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightLinSlide.setPower(0.4);
                    }
                    if(gamepad1.left_bumper&&rightLinSlide.getCurrentPosition() >=0){
                        rightLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightLinSlide.setPower(0.4);
                    }
                }
                ServoTele.open(gamepad1.x);
                ServoTele.close(gamepad1.y);
                telemetry.addData("Position", rightLinSlide.getCurrentPosition());
                telemetry.addData("ServoPositionR", ClawServoR.getPosition());
                telemetry.addData("ServoPositionL", ClawServoL.getPosition());
                telemetry.update();
                double speedPosition = -gamepad2.left_stick_y;
                double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x ;
                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx)/ denominator;
                double backLeftPower = (y - x + rx)/ denominator;
                double frontRightPower = (y - x - rx)/ denominator;
                double backRightPower = (y + x - rx)/ denominator;
                if(speedPosition == 0){
                    motorFrontLeft.setPower(frontLeftPower*0.65);
                    motorBackLeft.setPower(backLeftPower*0.65);
                    motorFrontRight.setPower(frontRightPower*0.65);
                    motorBackRight.setPower(backRightPower*0.65);
                }
                else{
                    motorFrontLeft.setPower(-frontLeftPower*speedPosition);
                    motorBackLeft.setPower(-backLeftPower*speedPosition);
                    motorFrontRight.setPower(-frontRightPower*speedPosition);
                    motorBackRight.setPower(-backRightPower*speedPosition);
                }

            }
            if(Modes == 2){
                telemetry.addData("Mode = ", "Farm");
                telemetry.update();
                if(gamepad1.a){
                    spin180();
                }

            }
        }

    }
    private double getAngle(){

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    private void resetAngle(){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    public void spin180(){
        resetAngle();
        int anglevalid = 0;
        do{
            motorFrontLeft.setPower(1);
            motorBackLeft.setPower(1);
            motorBackRight.setPower(-1);
            motorFrontRight.setPower(-1);
            if(getAngle() == 180){
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                motorFrontRight.setPower(0);
                anglevalid = 1;
            }
        } while (anglevalid == 0);
        resetAngle();
    }

}
