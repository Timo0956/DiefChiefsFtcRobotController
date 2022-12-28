package org.firstinspires.ftc.teamcode.Tele.untested.COMPCODEDON;

import static org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele.setServos;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.Tele.tested.initialize2023;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.TwoStageLinSlideFileNew;
import org.firstinspires.ftc.teamcode.Tele.untested.oldOrUntestedCode.drive;
import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;

@TeleOp
public class ExCompTele extends LinearOpMode {
    static BNO055IMU imu; //Gets the IMU
    Acceleration acceleration = new Acceleration(); //Gets the acceleration
    static Orientation lastAngles = new Orientation(); //Gets the heading in degrees
    static double globalAngle;
    static DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
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
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
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

        lastAngles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        acceleration = imu.getLinearAcceleration();

        drive.initDrive(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight);
        setServos(ClawServoL, ClawServoR);
        TwoStageLinSlideFileNew.setLSMotor(rightLinSlide,leftLinSlide);//defines motors in terms of the seperate file
        newFarm.initFarmNew(imu,acceleration,lastAngles,motorFrontLeft,motorBackLeft,motorFrontRight,motorBackRight,rightLinSlide,leftLinSlide);
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){

            telemetry.addData("Position", rightLinSlide.getCurrentPosition());
            telemetry.addData("ServoPositionR", ClawServoR.getPosition());
            telemetry.addData("ServoPositionL", ClawServoL.getPosition());
            telemetry.update();

            newFarm.farmFromPark(gamepad1.a, gamepad2.dpad_up, gamepad2.dpad_down);
            TwoStageLinSlideFileNew.linSlideDouble(gamepad1); //takes gamepad input
            ServoTele.open(gamepad1.x);
            ServoTele.close(gamepad1.y);


            double speedfactor = Math.abs(gamepad2.left_stick_y);
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x ;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx)/ denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx)/ denominator;
            double backRightPower = (y + x - rx)/ denominator;
            if(speedfactor<0.05) {
                motorFrontLeft.setPower(frontLeftPower * 0.6);
                motorBackLeft.setPower(backLeftPower * 0.6);
                motorFrontRight.setPower(frontRightPower * 0.6);
                motorBackRight.setPower(backRightPower * 0.6);
            } else if (speedfactor>=0.05 && TwoStageLinSlideFileNew.state== TwoStageLinSlideFileNew.states.HIGH){
                motorFrontLeft.setPower(Math.min(frontLeftPower * speedfactor, 0.7));
                motorBackLeft.setPower(Math.min(backLeftPower * speedfactor,0.7));
                motorFrontRight.setPower(Math.min(frontRightPower * speedfactor,0.7));
                motorBackRight.setPower(Math.min(backRightPower * speedfactor,0.7));
            } else if (speedfactor>=0.05 && TwoStageLinSlideFileNew.state != TwoStageLinSlideFileNew.states.HIGH){
                motorFrontLeft.setPower(Math.min(frontLeftPower * speedfactor, 1));
                motorBackLeft.setPower(Math.min(backLeftPower * speedfactor, 1));
                motorFrontRight.setPower(Math.min(frontRightPower * speedfactor,1));
                motorBackRight.setPower(Math.min(backRightPower * speedfactor,1 ));
            }
            if(gamepad1.dpad_right){
                rotate(157,1);
            }
            
        }
    }
    private static void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private static double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

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

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public static void rotate(int degrees, double power) throws InterruptedException
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        motorFrontLeft.setPower(leftPower);
        motorBackLeft.setPower(leftPower);
        motorFrontRight.setPower(rightPower);
        motorBackRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {}

            while  (getAngle() > degrees) {}
        }
        else    // left turn.
            while (getAngle() < degrees) {}

        // turn the motors off.
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        Thread.sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }


}
