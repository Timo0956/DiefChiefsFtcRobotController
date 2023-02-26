package org.firstinspires.ftc.teamcode.Tele.untested.COMPCODEDON;

import static org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele.setServos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.Tele.tested.initialize2023;
import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.TwoStageLinSlideFileNew;
import org.firstinspires.ftc.teamcode.Tele.untested.oldOrUntestedCode.drive;
import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.NewServoClaw;
import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@TeleOp
public class ExCompTele extends LinearOpMode {
    static BNO055IMU imu; //Gets the IMU
    static Acceleration acceleration = new Acceleration(); //Gets the acceleration
    static Orientation lastAngles = new Orientation(); //Gets the heading in degrees
    static double globalAngle;
    static SampleMecanumDrive drivetrain;
    //static double[] V = {0,0,0}; //The speed of the robot in m/s (z,x,y)
    //static double[] D = {0,0,0}; //The displacement of the robot in meters (z,x,y)
    //static int[] adjustments = {0,0,0}; //backward/forward, left/right, rotation
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

        Pose2d farmOne = new Pose2d(-12*2.54,-36*2.54,Math.toRadians(-90));
        Pose2d farmTwo = new Pose2d(-36*2.54,-36*2.54,Math.toRadians(-90));
        Pose2d farmThree = new Pose2d(-60*2.54,-36*2.54,Math.toRadians(-90));
        Pose2d farmSetting = null;

        DcMotor rightLinSlide = hardwareMap.dcMotor.get("rightLinSlide"); //defines our motors for LinSlide
        DcMotor leftLinSlide = hardwareMap.dcMotor.get("leftLinSlide");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        //Servo ClawServoL = hardwareMap.servo.get("clawServoL");
        //Servo ClawServoR = hardwareMap.servo.get("clawServoR");
        Servo spinServo = hardwareMap.servo.get("spinServo");
        //set zero power behavior to brake
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int LSCount = 5;

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        lastAngles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        acceleration = imu.getLinearAcceleration();

        drive.initDrive(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight);

        //setServos(ClawServoL, ClawServoR);
        TwoStageLinSlideFileNew.setLSMotor(rightLinSlide,leftLinSlide);//defines motors in terms of the seperate file
        NewServoClaw.initServo(spinServo);
        newFarm.initFarmNew(imu,acceleration,lastAngles,motorFrontLeft,motorBackLeft,motorFrontRight,motorBackRight,rightLinSlide,leftLinSlide);
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        waitForStart();
        if (isStopRequested()) return;
        spinServo.setPosition(1);
        TwoStageLinSlideFileNew.moveStates(0,true,false,0,false,false);
        while (opModeIsActive()){

            telemetry.addData("LSCount", LSCount);
            telemetry.addData("LSPosition", rightLinSlide.getCurrentPosition());
            telemetry.addData("ServoPos ", spinServo.getPosition());
            //telemetry.addData("ServoPositionR", ClawServoR.getPosition());
            //telemetry.addData("ServoPositionL", ClawServoL.getPosition());
            telemetry.update();

            //newFarm.farmFromPark(gamepad1.a, farmSetting);
            TwoStageLinSlideFileNew.linSlideDouble(gamepad1); //takes gamepad input
            NewServoClaw.openPosition(gamepad1.x);
            if (TwoStageLinSlideFileNew.state == TwoStageLinSlideFileNew.states.LOW) {
                NewServoClaw.closePosition(gamepad1.y, 350);
            } else if (TwoStageLinSlideFileNew.state == TwoStageLinSlideFileNew.states.CUSTOM) {
                NewServoClaw.closePosition(gamepad1.y, 1750);

            }
            ///NewServoClaw.closePosition(gamepad1.y, 400);


            //double speedfactor = Math.abs(gamepad2.left_stick_y);
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x ;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx)/ denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx)/ denominator;
            double backRightPower = (y + x - rx)/ denominator;
            if(!gamepad2.y&&!gamepad2.x) {
                motorFrontLeft.setPower(frontLeftPower * 0.6);
                motorBackLeft.setPower(backLeftPower * 0.6);
                motorFrontRight.setPower(frontRightPower * 0.6);
                motorBackRight.setPower(backRightPower * 0.6);
            } else if (gamepad2.y && TwoStageLinSlideFileNew.state== TwoStageLinSlideFileNew.states.HIGH){
                motorFrontLeft.setPower(frontLeftPower * 0.6);
                motorBackLeft.setPower(backLeftPower * 0.6);
                motorFrontRight.setPower(frontRightPower * 0.6);
                motorBackRight.setPower(backRightPower * 0.6);
            } else if (gamepad2.y && TwoStageLinSlideFileNew.state != TwoStageLinSlideFileNew.states.HIGH){
                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);
            } else if (gamepad2.x){
                motorFrontLeft.setPower(frontLeftPower * 0.3);
                motorBackLeft.setPower(backLeftPower * 0.3);
                motorFrontRight.setPower(frontRightPower * 0.3);
                motorBackRight.setPower(backRightPower * 0.3);
            }
            if(gamepad1.dpad_right){
                // TwoStageLinSlideFileNew.moveStates(1,false,false,0, false,false);
                TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.HIGH;
                TwoStageLinSlideFileNew.goPosition(1, 4050);
                //Thread.sleep(300);
                rotate(156,1);
                //Thread.sleep(200);

            }
            else if (gamepad1.dpad_left){
                rotate(156,1);
            }
            if(gamepad2.right_trigger>0.7){
                rightLinSlide.setTargetPosition(rightLinSlide.getCurrentPosition() - 100);
                rightLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLinSlide.setPower(0.8);
                leftLinSlide.setTargetPosition(leftLinSlide.getCurrentPosition() - 100);
                leftLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLinSlide.setPower(-0.8);
            }
            if(gamepad2.left_trigger > 0.7){
                rightLinSlide.setPower(0);
                leftLinSlide.setPower(0);
                rightLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightLinSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftLinSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad2.a){
                switch (LSCount){
                    case 1:
                        Thread.sleep(100);
                        break;

                    case 2:
                        TwoStageLinSlideFileNew.goPosition(-0.8,0);
                        TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.LOW;
                        Thread.sleep(100);
                        LSCount = 1;
                        break;

                    case 3:
                        TwoStageLinSlideFileNew.goPosition(-0.8,150);
                        TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.CUSTOM;
                        Thread.sleep(100);
                        LSCount = 2;
                        break;

                    case 4:
                        TwoStageLinSlideFileNew.goPosition(-0.8,300);
                        TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.CUSTOM;
                        Thread.sleep(100);
                        LSCount = 3;
                        break;

                    case 5:
                        TwoStageLinSlideFileNew.goPosition(-0.8,450);
                        TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.CUSTOM;
                        Thread.sleep(100);
                        LSCount = 4;
                        break;

                }
            } else if (gamepad2.b){
                switch (LSCount){
                    case 1:
                        TwoStageLinSlideFileNew.goPosition(0.8,150);
                        TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.CUSTOM;
                        Thread.sleep(100);
                        LSCount = 2;
                        break;

                    case 2:
                        TwoStageLinSlideFileNew.goPosition(0.8,300);
                        TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.CUSTOM;
                        Thread.sleep(100);
                        LSCount = 3;
                        break;

                    case 3:
                        TwoStageLinSlideFileNew.goPosition(0.8,450);
                        TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.CUSTOM;
                        Thread.sleep(100);
                        LSCount = 4;
                        break;

                    case 4:
                        TwoStageLinSlideFileNew.goPosition(0.8,600);
                        TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.CUSTOM;
                        Thread.sleep(100);
                        LSCount = 5;
                        break;

                    case 5:
                        TwoStageLinSlideFileNew.goPosition(0.8,600);
                        TwoStageLinSlideFileNew.state = TwoStageLinSlideFileNew.states.CUSTOM;
                        Thread.sleep(100);
                        break;

                }
            }
         /*   if(gamepad2.a){ //farm from FAR
                farmPos(-48*2.54,-36*2.54,farmThree);
                farmSetting = farmThree;
            }
            else if (gamepad2.b){ //farm from MIDDLE
                farmPos(-24*2.54,-36*2.54,farmTwo);
                farmSetting = farmTwo;
            }
            else if (gamepad2.x){ // farm from CLOSE
                farmPos(0,-36*2.54,farmOne);
                farmSetting = farmOne;
            }else if (gamepad2.y){ // reset farming position
                farmSetting = null;
            }
            /*if(gamepad2.dpad_down){
                adjustments[0]--;
                calculateAdj();
                while(gamepad2.dpad_down){}
            } else if(gamepad2.dpad_up){
                adjustments[0]++;
                calculateAdj();
                while(gamepad2.dpad_up){}
            } else if(gamepad2.dpad_left){
                adjustments[1]--;
                calculateAdj();
                while(gamepad2.dpad_left){}
            } else if(gamepad2.dpad_right){
                adjustments[1]++;
                calculateAdj();
                while(gamepad2.dpad_right){}
            } else if(gamepad2.a){
                adjustments[2]--;
                calculateAdj();
                while(gamepad2.a){}
            } else if(gamepad2.b){
                adjustments[2]++;
                calculateAdj();
                while(gamepad2.b){}
            }*/

        }
    }
    /*  public static void farmPos(double xcm, double ycm, Pose2d farmPosition){
          Trajectory goFarm = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                  .splineTo(new Vector2d(-xcm/2.54,0),0)
                  .splineTo(new Vector2d(-xcm/2.54,-ycm/2.54),0)
                  .splineTo(new Vector2d((-xcm/2.54)-12,-ycm/2.54),0)
                  .splineToLinearHeading(farmPosition,0)
                  .build();
          drivetrain.followTrajectory(goFarm);
      }*/
    public static void moveBackward (long time)throws InterruptedException{
        motorBackLeft.setPower(-0.5);
        motorFrontLeft.setPower(-0.5);
        motorFrontRight.setPower(-0.5);
        motorBackRight.setPower(-0.5);
        Thread.sleep(time);
    }
    private static void resetAngleAcc()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;

       /* V[0] = 0;
        V[1] = 0;
        V[2] = 0;
        D[0] = 0;
        D[1] = 0;
        D[2] = 0;*/

    }
   /* private static void calculateAdj() throws InterruptedException {//this function lets us adjust our farming during operation

        if(adjustments[0]==0 && adjustments[1]==0 && adjustments[2]==0) return;

        double denominator = Math.max(Math.abs(adjustments[0]) + Math.abs(adjustments[1]) + Math.abs(adjustments[2]), 1);
        double fl = (adjustments[0] + adjustments[1] + adjustments[2])/denominator;
        double bl = (adjustments[0] - adjustments[1] + adjustments[2])/denominator;
        double fr = (adjustments[0] - adjustments[1] - adjustments[2])/denominator;
        double br = (adjustments[0] + adjustments[1] - adjustments[2])/denominator;
        double avg = (adjustments[0] + adjustments[1] + adjustments[2])/3;

        motorFrontLeft.setPower(fl);
        motorBackLeft.setPower(bl);
        motorFrontRight.setPower(fr);
        motorBackRight.setPower(br);
        Thread.sleep((long) (avg*300));

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);

        adjustments[0] = 0;
        adjustments[1] = 0;
        adjustments[2] = 0;
        avg = 0;


    } */

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
  /*  private static double[] getDist() throws InterruptedException {
        acceleration = imu.getLinearAcceleration();

        V[0] += (acceleration.zAccel/100);
        V[1] += (acceleration.yAccel/100);
        V[2] += (acceleration.xAccel/100); //Divides the acceleration by time (Or multiplied by 0.01s) to get the velocity
        D[0] += V[0]/100;
        D[1] += V[1]/100;
        D[2] += V[2]/100;//Divides the velocity by time (Or multiplied by 0.01s) to get the distance
        Thread.sleep(10); //Stops the loop for the amount of time in the brackets in miliseconds

        double[] distance = {D[0],D[1],D[2]};
        return distance;
    } */

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public static double checkDirection()
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

    public static void rotateIMU(int degrees, double power)
    {
        double  leftPower, rightPower;
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
        // Thread.sleep(300);
        // reset angle tracking on new heading.
    }
    public static void rotate(int degrees, double speed){
        resetAngleAcc();
        rotateIMU(degrees, speed);
        rotateIMU(degrees, 0.5 * speed);
        resetAngleAcc();
    }
   /* public static void moveFB(double power, double dist) throws InterruptedException {

        resetAngleAcc();

        if (dist < 0)
        {
            // move back.
            while (getDist()[0] == 0||getDist()[1] == 0||getDist()[2] == 0) {
                double correction = checkDirection();
                motorFrontLeft.setPower(power-correction);
                motorBackLeft.setPower(power-correction);
                motorFrontRight.setPower(power+correction);
                motorBackRight.setPower(power+correction);
            }

            while  (getDist()[0] > dist||getDist()[1] > dist||getDist()[2] > dist) {
                double correction = checkDirection();
                motorFrontLeft.setPower(power-correction);
                motorBackLeft.setPower(power-correction);
                motorFrontRight.setPower(power+correction);
                motorBackRight.setPower(power+correction);
            }
        }
        else    // move forward.
            while (getDist()[0] < dist||getDist()[1] < dist||getDist()[2] < dist) {
                double correction = checkDirection();
                motorFrontLeft.setPower(power-correction);
                motorBackLeft.setPower(power-correction);
                motorFrontRight.setPower(power+correction);
                motorBackRight.setPower(power+correction);
            }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);

        resetAngleAcc();
    }
    public static void strafeSideToSide(double power, double yDisplacement) throws InterruptedException{
        resetAngleAcc();




        if(yDisplacement > 0){
            while(yDisplacement>getDist()[0]||yDisplacement > getDist()[1]||yDisplacement > getDist()[2]){
                double correction = checkDirection();
                motorBackRight.setPower(power + correction);
                motorBackLeft.setPower(power - correction);
                motorFrontRight.setPower(-power + correction);
                motorFrontLeft.setPower(-power - correction);
            }
            while (yDisplacement == 0){
                double correction = checkDirection();
                motorBackRight.setPower(power + correction);
                motorBackLeft.setPower(power - correction);
                motorFrontRight.setPower(-power + correction);
                motorFrontLeft.setPower(-power - correction);
            }
        }
        else if (yDisplacement < 0){
            while (yDisplacement<getDist()[0]||yDisplacement < getDist()[1]||yDisplacement < getDist()[2]){
                double correction = checkDirection();
                motorBackRight.setPower(power + correction);
                motorBackLeft.setPower(power - correction);
                motorFrontRight.setPower(-power + correction);
                motorFrontLeft.setPower(-power - correction);
            }
        }

        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);

        resetAngleAcc();

    }*/

}
