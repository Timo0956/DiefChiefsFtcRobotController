package org.firstinspires.ftc.teamcode.Auto.RoadRunnerTesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Tele.untested.linSlideFiles.TwoStageLinSlideFile;
import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.TwoStageLinSlideFileNew;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class splinetest extends LinearOpMode {
    static Servo ClawL = null;
    static Servo ClawR = null;
    static DcMotor motorFrontLeft = null;
    static DcMotor motorBackLeft = null;
    static DcMotor motorFrontRight = null;
    static DcMotor motorBackRight = null;
    static DcMotor rightLinSlide = null;
    static DcMotor leftLinSlide = null;
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

    AprilTagDetection tagOfInterest = null;
    static SampleMecanumDrive drivetrain;

    @Override

    public void runOpMode() throws InterruptedException {
        /*long msPerCm = 1500/89;
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
        rightLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
        drivetrain = new SampleMecanumDrive(hardwareMap);
        ClawL = hardwareMap.servo.get("clawServoL");
        ClawR = hardwareMap.servo.get("clawServoR");
        rightLinSlide = hardwareMap.dcMotor.get("rightLinSlide");
        leftLinSlide = hardwareMap.dcMotor.get("leftLinSlide");

        TwoStageLinSlideFileNew.setLSMotor(rightLinSlide, leftLinSlide);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == one || tag.id == two || tag.id == three) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
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
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest.id == one) {
            closeServo();

        } else if (tagOfInterest.id == two) {
            closeServo();




        } else if (tagOfInterest.id == three) {
            closeServo();
            moveBackward(24 * 2.54);
            strafeLeft(60 * 2.54);
            moveLinSlidePosition(4050, 0.8, 600);
            moveForward(6 * 2.54);
            openServo();
            moveLinSlidePosition(600, -0.8, 600);
            moveBackward(6 * 2.54);
            strafeRight(1 * 2.54);
            moveForward(4 * 2.54);
            moveForward(1 * 2.54);
            closeServo();
            moveLinSlidePosition(600, -0.8, 600);
            moveBackward(3 * 2.54);
            moveLinSlidePosition(800, 0.8, 600);
            strafeRight(1 * 2.54);
            moveForward(1 * 2.54);
            openServo();
            moveBackward(1 * 2.54);
            strafeLeft(1 * 2.54);
            repeat(600);
            repeat(400);
            repeat(200);
            repeat(0);
            moveForward(2 * 2.54);
        }
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        // while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    /*  public static void pause(long time)throws InterruptedException{
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
    public static void closeServo() {
        // ServoTele.close(true);
        ClawL.setPosition(0.15);
        ClawR.setPosition(0.15);
    }

    public static void openServo() {
        //   ServoTele.open(true);
        ClawL.setPosition(0);
        ClawR.setPosition(0);
    }

    public static void moveLinSlidePosition(int position, double speed, long time) throws InterruptedException {
        rightLinSlide.setTargetPosition(position);
        rightLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinSlide.setPower(speed);
        leftLinSlide.setTargetPosition(-position);
        leftLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinSlide.setPower(-speed);
        Thread.sleep(time);
    }

    public static void moveForward(double cm) {
        Trajectory goForward = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(cm / 2.54)
                .build();
        drivetrain.followTrajectory(goForward);
    }

    public static void moveBackward(double cm) {
        Trajectory goBackward = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .back(cm / 2.54)
                .build();
        drivetrain.followTrajectory(goBackward);
    }

    public static void strafeLeft(double cm) {
        Trajectory goBackward = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeLeft(cm / 2.54)
                .build();
        drivetrain.followTrajectory(goBackward);
    }

    public static void strafeRight(double cm) {
        Trajectory goBackward = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeRight(cm / 2.54)
                .build();
        drivetrain.followTrajectory(goBackward);
    }

    public static void rotate(double degrees) {
        drivetrain.turn(Math.toRadians(degrees));
    }

    public static void repeat(int linslidePos) throws InterruptedException {
        moveForward(2 * 2.54);
        moveLinSlidePosition(linslidePos, 0.8, 600);
        moveForward(1 * 2.54);
        closeServo();
        moveLinSlidePosition(1100, 0.8, 600);
        moveBackward(3 * 2.54);
        rotate(180);
        strafeRight(1 * 2.54);
        moveLinSlidePosition(4050, 0.8, 600);
        moveForward(1 * 2.54);
        openServo();
        moveBackward(1 * 2.54);
        moveLinSlidePosition(0, -0.8, 600);
        strafeLeft(1 * 2.54);
        rotate(180);
    }
    public static void splineMethod(double x, double y, double starting_x, double starting_y){
        Trajectory traj = drivetrain.trajectoryBuilder(new Pose2d(starting_x,starting_y),0)
                .splineTo(new Vector2d(x, y), 0)
                .build();
        drivetrain.followTrajectory(traj);
    }
}