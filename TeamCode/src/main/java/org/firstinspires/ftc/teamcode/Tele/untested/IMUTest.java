package org.firstinspires.ftc.teamcode.Tele.untested;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@TeleOp
public class IMUTest extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;
    // Acceleration gravity;


    @Override
    public void runOpMode() throws InterruptedException{

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
         // parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        while(opModeIsActive())
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("IMU Status", imu.getSystemStatus());
            telemetry.addData("Calibration Status", imu.getCalibrationStatus());
            telemetry.addData("Heading (Z)", angles.firstAngle); //Z
            telemetry.addData("Roll (Y)", angles.secondAngle); // Y
            telemetry.addData("Pitch (X)", angles.thirdAngle); // X

            telemetry.update();

        }




    }
}
