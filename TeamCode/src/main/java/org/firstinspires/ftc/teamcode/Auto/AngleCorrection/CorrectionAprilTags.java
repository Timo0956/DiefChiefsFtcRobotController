/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
//170-196

package org.firstinspires.ftc.teamcode.Auto.AngleCorrection;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Auto.AngleCorrection.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Tele.untested.linSlideFiles.TwoStageLinSlideFile;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class CorrectionAprilTags extends LinearOpMode
{
    static Servo ClawL = null;
    static Servo ClawR = null;
    static DcMotor motorFrontLeft = null;
    static DcMotor motorBackLeft = null;
    static DcMotor motorFrontRight = null;
    static DcMotor motorBackRight = null;
    static DcMotor rightLinSlide = null;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int one = 8;
    int two = 9;
    int three = 10;// Tag ID 18 from the 36h11 family

    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    boolean                 aButton, bButton, touched;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        long msPerCm = 1500/89;
        double power = 0.5;
        ClawL = hardwareMap.servo.get("clawServoL");
        ClawR = hardwareMap.servo.get("clawServoR");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        rightLinSlide = hardwareMap.dcMotor.get("rightLinSlide");

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TwoStageLinSlideFile.setLSMotor(rightLinSlide);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == one || tag.id == two || tag.id == three)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if( tagOfInterest.id == one){
            closeServo();
            pause(700); //grab cone
            moveLinSlidePosition(100,0.9, 900);
            pause(100);
            leftStrafe(-power, msPerCm*15);
            pause(100);
            forwardBackwardDrive(-power,msPerCm*45); //103
            /*pause(100);
            moveLinSlidePosition(3000,0.9, 2000); //lift cone
            forwardBackwardDrive(power,msPerCm*13); //get to position
            pause(100);
            openServo(); // drop cone
            pause(100);
            forwardBackwardDrive(-power,msPerCm*13);
            pause(100);
            moveLinSlidePosition(0,0.9, 2000); // lower linslide*/
     /*   leftStrafe(-power,msPerCm*28); // into position for moving forward
        pause(100);
        moveLinSlidePosition(400,0.9, 0); // move ls into position
        openServo();
        forwardBackwardDrive(power,msPerCm*202); // move to stack
        pause(1000);
        closeServo();
        pause(1000);
        moveLinSlidePosition(1000,0.9, 2000);
        forwardBackwardDrive(-power,msPerCm*205);
        pause(500);
        rightStrafe(-power,msPerCm*33);
        pause(100);
        moveLinSlidePosition(3000,0.9, 2000);
        pause(200);
        forwardBackwardDrive(power,msPerCm*8);
        pause(200);
        openServo();
        pause(200);
        forwardBackwardDrive(-power,msPerCm*10);
        pause(200);
        moveLinSlidePosition(0,0.9, 2000);
        pause(200);

      */
            //rightStrafe(-power,msPerCm*35); //parking
            pause(1000);
            leftStrafe(-power, msPerCm*70);
            forwardBackwardDrive(power,msPerCm*5);
            openServo();
            moveLinSlidePosition(00,0.9, 900);
        }
        else if (tagOfInterest.id == two){
            closeServo();
            pause(700); //grab cone
            moveLinSlidePosition(100,0.9, 900);
            pause(100);
            leftStrafe(-power,msPerCm*65); //103
            openServo();
            moveLinSlidePosition(00,0.9, 900);
         /*   pause(100);
            moveLinSlidePosition(3000,0.9, 2000); //lift cone
            forwardBackwardDrive(power,msPerCm*13); //get to position
            pause(100);
            openServo(); // drop cone
            pause(100);
            forwardBackwardDrive(-power,msPerCm*10);
            pause(100);
            moveLinSlidePosition(0,0.9, 2000); // lower linslide*/
     /*   leftStrafe(-power,msPerCm*28); // into position for moving forward
        pause(100);
        moveLinSlidePosition(400,0.9, 0); // move ls into position
        openServo();
        forwardBackwardDrive(power,msPerCm*202); // move to stack
        pause(1000);
        closeServo();
        pause(1000);
        moveLinSlidePosition(1000,0.9, 2000);
        forwardBackwardDrive(-power,msPerCm*205);
        pause(500);
        rightStrafe(-power,msPerCm*33);
        pause(100);
        moveLinSlidePosition(3000,0.9, 2000);
        pause(200);
        forwardBackwardDrive(power,msPerCm*8);
        pause(200);
        openServo();
        pause(200);
        forwardBackwardDrive(-power,msPerCm*10);
        pause(200);
        moveLinSlidePosition(0,0.9, 2000);
        pause(200);

      */
           // rightStrafe(-power,msPerCm*30); //parking


        }
        else if (tagOfInterest.id == three){
            closeServo();
            pause(700); //grab cone
            moveLinSlidePosition(100,0.9, 900);
            pause(100);
            forwardBackwardDrive(power,msPerCm*45);
            pause(500);
            /*moveLinSlidePosition(3000,0.9, 2000); //lift cone
            forwardBackwardDrive(power,msPerCm*13); //get to position
            pause(100);
            openServo(); // drop cone
            pause(100);
            forwardBackwardDrive(-power,msPerCm*10);
            pause(100);
            moveLinSlidePosition(0,0.9, 2000); // lower linslide*/
     /*   leftStrafe(-power,msPerCm*28); // into position for moving forward
        pause(100);
        moveLinSlidePosition(400,0.9, 0); // move ls into position
        openServo();
        forwardBackwardDrive(power,msPerCm*202); // move to stack
        pause(1000);
        closeServo();
        pause(1000);
        moveLinSlidePosition(1000,0.9, 2000);
        forwardBackwardDrive(-power,msPerCm*205);
        pause(500);
        rightStrafe(-power,msPerCm*33);
        pause(100);
        moveLinSlidePosition(3000,0.9, 2000);
        pause(200);
        forwardBackwardDrive(power,msPerCm*8);
        pause(200);
        openServo();
        pause(200);
        forwardBackwardDrive(-power,msPerCm*10);
        pause(200);
        moveLinSlidePosition(0,0.9, 2000);
        pause(200);

      */
            //rightStrafe(-power,msPerCm*22); //parking
            pause(100);
            leftStrafe(-power, msPerCm*70);
            openServo();
            moveLinSlidePosition(00,0.9, 900);
        }
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
       // while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    public static void pause(long time)throws InterruptedException{
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        Thread.sleep(time);
    }
    public static void forwardBackwardDrive (double power, long time) throws InterruptedException {
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(-power);
        motorBackRight.setPower(power);
        Thread.sleep(time);
    }
    public static void leftStrafe (double power, long time) throws InterruptedException{
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        Thread.sleep(time);
    }
    public static void rightStrafe (double power, long time) throws InterruptedException {
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(-power);
        motorBackRight.setPower(-power);
        Thread.sleep(time);
    }
    /*    public static void toLowLinSlide(){
            TwoStageLinSlideFile.moveStates(0,true,false,0);
        }
        public static void toMidLinSlide(){
            TwoStageLinSlideFile.moveStates(0,false,true,0);
        }
        public static void toHighLinSlide(){
            TwoStageLinSlideFile.moveStates(1,false,false,0);
        }
        public static void toLowOffLinSlide(){TwoStageLinSlideFile.moveStates(0,false,false,1);} */// commented out extra linside functions
    public static void closeServo(){
        // ServoTele.close(true);
        ClawL.setPosition(0.15);
        ClawR.setPosition(0.15);
    }
    public static void openServo(){
        //   ServoTele.open(true);
        ClawL.setPosition(0);
        ClawR.setPosition(0);
    }
    public static void moveLinSlidePosition (int position, double speed, long time)throws InterruptedException{
        rightLinSlide.setTargetPosition(position);
        rightLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinSlide.setPower(speed);
        Thread.sleep(time);
    }
    private double getAngle()
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
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

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
}