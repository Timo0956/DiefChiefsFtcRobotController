package org.firstinspires.ftc.teamcode.Auto.AngleCorrection;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class camera extends LinearOpMode {
    OpenCvWebcam webcam1 = null;
    static String position = null;
    @Override
    public void runOpMode() throws InterruptedException{

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam1.setPipeline(new cameraPipeline());
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam1.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        waitForStart();
        while (opModeIsActive()){
            while(position != "middle"){
                if(position == "right"){
                    telemetry.addData("Object is: ", "right, move left");
                    telemetry.update();
                }
                else if (position == "left"){
                    telemetry.addData("Object is", "left, move right");
                    telemetry.update();
                }
            }
            telemetry.addData("Object is: ", "middle");
            telemetry.update();
        }


    }
    public static void posRobot(String pos){
        if (pos == "left"){
            position = "Left";
        }
        else if (pos == "right"){
            position = "Right";
        }
        else if (pos == "middle"){
            position = "Middle";
        }
    }


}
