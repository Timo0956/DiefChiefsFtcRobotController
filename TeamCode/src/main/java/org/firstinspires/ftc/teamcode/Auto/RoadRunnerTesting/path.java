package org.firstinspires.ftc.teamcode.Auto.RoadRunnerTesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class path extends LinearOpMode {
    static SampleMecanumDrive drivetrain ;
    @Override
    public void runOpMode() throws InterruptedException{
        drivetrain = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        moveBackward(24*2.54);
        strafeLeft(60*2.54);
        //claw up
        moveForward(6*2.54);
        //drop cone
        //drop linslide
        moveBackward(6*2.54);
        strafeRight(1*2.54);
        moveForward(4*2.54);
        //raise linslide
        moveForward(1*2.54);
        // claw close
        //raise linslide
        moveBackward(3*2.54);
        // lower to position
        strafeRight(1*2.54);
        moveForward(1*2.54);
        //dropcone
        moveBackward(1*2.54);
        strafeLeft(1*2.54);
        repeat(600);
        repeat(400);
        repeat(200);
        repeat(0);
    }
    public static void moveForward (double cm){
        Trajectory goForward = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                .forward(cm / 2.54)
                .build();
        drivetrain.followTrajectory(goForward);
    }
    public static void moveBackward (double cm){
        Trajectory goBackward = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                .back(cm / 2.54)
                .build();
        drivetrain.followTrajectory(goBackward);
    }
    public static void strafeLeft (double cm){
        Trajectory goBackward = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                .strafeLeft(cm / 2.54)
                .build();
        drivetrain.followTrajectory(goBackward);
    }
    public static void strafeRight (double cm){
        Trajectory goBackward = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                .strafeRight(cm / 2.54)
                .build();
        drivetrain.followTrajectory(goBackward);
    }
    public static void rotate (double degrees){
        drivetrain.turn(degrees);
    }
    public static void repeat(double linslidePos){
        moveForward(2*2.54);
        //move linslide to linslidePos
        moveForward(1*2.54);
        //close claw
        //movelinslideUpToPosition
        moveBackward(3*2.54);
        rotate(180);
        strafeRight(1*2.54);
        //raise linslide
        moveForward(1*2.54);
        //drop
        moveBackward(1*2.54);
        //linslide down
        strafeLeft(1*2.54);
        rotate(180);
    }

}
