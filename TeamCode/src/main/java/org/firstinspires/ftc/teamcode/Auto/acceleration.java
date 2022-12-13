package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@TeleOp
public class acceleration extends LinearOpMode {

    BNO055IMU imu; //Gets the IMU
    Acceleration acceleration; //Gets the acceleration
    Orientation angles; //Gets the heading in degrees

    public static double heading = 0; //The heading of the robot in degrees
    public static double[] velocity = {0,0,0}; //The speed of the robot in m/s (z,x,y)
    public static double[] distance = {0,0,0}; //The displacement of the robot in meters (z,x,y)

    public void runOpMode() throws InterruptedException { //The acceleration method accessed from within the other autonomous code that runs the robot during the autonomous session

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //Sets up the parameters
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES; //Gets the angle units in degrees from the IMU
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; //Gets the units of acceleration from the IMU in m/s^2
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu"); //Gets the IMU hardware map to know which port to read from
        imu.initialize(parameters); //Initializes the IMU at the set parameters above

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets the angles of the IMU in the control hub
        acceleration = imu.getLinearAcceleration(); //Gets the acceleration of the control hub

        //Just putting here for later to use. Need to delay to not make it think velocity is extremely fast
 //     heading = formatAngle(angles.angleUnit, angles.firstAngle); //Gets the heading of the robot to make sure it turns accurately during the autonomous time.
        heading = angles.firstAngle; //change which angle if needed

        //delay by 1/4 of a second and 1/4 the accel to get velocity
        while(true) { //While the variable is true it will continue looping the code
            velocity[0] = (acceleration.zAccel/100);
            velocity[1] = (acceleration.yAccel/100);
            velocity[2] = (acceleration.xAccel/100); //Divides the acceleration by time (Or multiplied by 0.01s) to get the velocity
            distance[0] = velocity[0]/100;
            distance[1] = velocity[1]/100;
            distance[2] = velocity[2]/100;//Divides the velocity by time (Or multiplied by 0.01s) to get the distance
            Thread.sleep(10); //Stops the loop for the amount of time in the brackets in miliseconds
        }

    }
}

