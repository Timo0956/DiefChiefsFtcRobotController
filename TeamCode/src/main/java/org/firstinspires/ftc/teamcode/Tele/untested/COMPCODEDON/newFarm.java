package org.firstinspires.ftc.teamcode.Tele.untested.COMPCODEDON;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.TwoStageLinSlideFileNew;
import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;

public class newFarm {
    static double[] heading = {90, 90, 90}; //The heading of the robot in degrees
    static double[] velocity = {0,0,0}; //The speed of the robot in m/s (z,x,y)
    static double[] distance = {0,0,0}; //The displacement of the robot in meters (z,x,y)

    static BNO055IMU imu;
    static Acceleration acc;
    static Orientation deg;
    static DcMotor fl;
    static DcMotor bl;
    static DcMotor fr;
    static DcMotor br;
    static DcMotor rls;
    static DcMotor lls;
    //static Servo lc;
    //static Servo rc;

    public static void initFarmNew(BNO055IMU IMU, Acceleration ACC, Orientation DEG, DcMotor FL, DcMotor BL, DcMotor FR, DcMotor BR, DcMotor RLS, DcMotor LLS){
        imu = IMU;
        acc = ACC;
        deg = DEG;
        fl = FL;
        fr = FR;
        bl = BL;
        br = BR;
        lls = LLS;
        rls = RLS;
        //lc = LC;
        //rc = RC;

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        //lls.setDirection(DcMotorSimple.Direction.REVERSE);

        deg   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        acc  = imu.getLinearAcceleration();

    }

    public static void farmFromPark(boolean a, Pose2d FP)throws InterruptedException{
        if(a) {
            //ExCompTele.farmPos(0,0,FP);
            moveFB(0.75, 300);
            ServoTele.close(true, 350);
            pause(200);
            TwoStageLinSlideFileNew.goPosition(0.9, 350);
            pause(350);
            moveFB(-0.75, 300);
            TwoStageLinSlideFileNew.goPosition(0.9, 4100);
            //turn(1, 767);
            ExCompTele.rotate(155,1);
            pause(400);
            moveFB(0.5, (int) Math.round(320/0.5));
            pause(1100);
            ServoTele.open(true);
            pause(500);
            moveFB(-0.5, (int) Math.round(320/0.5));
            TwoStageLinSlideFileNew.goPosition(-0.9, 0);
            //turn(1, 767);
            ExCompTele.rotate(155,1);
            Thread.sleep(1200);

        }


    }
    public static void moveFB(double power, int time)throws InterruptedException{
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
            Thread.sleep(time);
           // updateDistMoved();

        pause(200);
    }
    public static void moveLR(double power, int time)throws InterruptedException{ // positive = right, negative = left
            fl.setPower(power);
            fr.setPower(-power);
            bl.setPower(-power);
            br.setPower(power);
        Thread.sleep(time);

        //updateDistMoved();

        pause(200);

    }
  /*  public static void turn(double power, int time) throws InterruptedException { //positive = right, negative = left
            fl.setPower(1);
            fr.setPower(-1);
            bl.setPower(1);
            br.setPower(-1);
        Thread.sleep(time);

        //updateDistMoved();

        pause(200);

    } */
    public static void pause(int time) throws InterruptedException {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
      /*  velocity[0] = 0;
        velocity[1] = 0;
        velocity[2] = 0; //Divides the acceleration by time (Or multiplied by 0.01s) to get the velocity
        distance[0] = 0;
        distance[1] = 0;
        distance[2] = 0; */
        Thread.sleep(time);
    }
    /*public static void updateDistMoved() throws InterruptedException {
        heading[0] = deg.firstAngle+90;
        heading[1] = deg.secondAngle+90;
        heading[2] = deg.thirdAngle+90;
        velocity[0] += (acc.zAccel/100);
        velocity[1] += (acc.yAccel/100);
        velocity[2] += (acc.xAccel/100); //Divides the acceleration by time (Or multiplied by 0.01s) to get the velocity
        distance[0] += velocity[0]/100;
        distance[1] += velocity[1]/100;
        distance[2] += velocity[2]/100;//Divides the velocity by time (Or multiplied by 0.01s) to get the distance
        Thread.sleep(10); //Stops the loop for the amount of time in the brackets in miliseconds
    }*/
}
