import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class camera extends LinearOpMode {
    OpenCvWebcam webcam1 = null;
    static String position = null;
    @Override
    public void runOpMode() throws InterruptedException{

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam1.setPipeline(new CameraPipelineV4());
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam1.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Direction:", camerapipeline3.get());
            telemetry.update();
        }
       /* while (opModeIsActive()){
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
        }*/


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
    static class camerapipeline3 extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat middleCrop;
        double leftavgfin;
        double rightavgfin;
        double middleavgfin;
        Mat output = new Mat();
        private static volatile TYPE type;
        Scalar rectColor = new Scalar(255.0,0.0,0.0);
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input,YCbCr, Imgproc.COLOR_RGB2YCrCb);

            Rect leftRect = new Rect(1,1,269,359);
            Rect rightRect = new Rect(371,1,269,359);
            Rect middleRect = new Rect(270,1,98,359);


            input.copyTo(output);
            Imgproc.rectangle(output,leftRect,rectColor,2);
            Imgproc.rectangle(output,rightRect,rectColor,2);
            Imgproc.rectangle(output,middleRect,rectColor,2);

            leftCrop =YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            middleCrop =YCbCr.submat(middleRect);

            Core.extractChannel(leftCrop,leftCrop,2);
            Core.extractChannel(rightCrop,rightCrop,2);
            Core.extractChannel(middleCrop,middleCrop,2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar middleavg = Core.mean(middleCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];
            middleavgfin = middleavg.val[0];

            if (leftavgfin>rightavgfin && leftavgfin >middleavgfin){
                type = TYPE.Left;
            }
            else if (rightavgfin > leftavgfin && rightavgfin > middleavgfin){
                type = TYPE.Right;
            }
            else if(middleavgfin>rightavgfin&&middleavgfin>leftavgfin){
                type = TYPE.Middle;
            }
            return(output);

        }
        public static TYPE get(){
            return type;
        }
        enum TYPE{
            Right, Left, Middle
        }

    }
    /*class camerapipeline4 extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat middleCrop;
        double leftavgfin;
        double rightavgfin;
        double middleavgfin;
        Mat output = new Mat();

        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            Rect rightRect = new Rect(1, 1, 269, 359);
            Rect leftRect = new Rect(371, 1, 269, 359);
            Rect middleRect = new Rect(270, 1, 100, 359);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, rightRect, rectColor, 2);
            Imgproc.rectangle(output, middleRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            middleCrop = YCbCr.submat(middleRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);
            Core.extractChannel(middleCrop, middleCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar middleavg = Core.mean(middleCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];
            middleavgfin = middleavg.val[0];

            if (leftavgfin > rightavgfin && leftavgfin > middleavgfin) {
                telemetry.addLine("left");
            } else if (rightavgfin > leftavgfin && rightavgfin >middleavgfin ) {
                telemetry.addLine("right");
            } else if (middleavgfin > leftavgfin && middleavgfin > rightavgfin ){
                telemetry.addLine("middle");
            }
            telemetry.update();
            return (output);
        }
    }*/
    static class CameraPipelineV4 extends OpenCvPipeline {
        public enum Position{
            LEFT, CENTER, RIGHT
        }
        Scalar BLUE = new Scalar(0,0,255);
        Scalar RED = new Scalar (255,0,0);
        static final Point topLeftFirstArea_Little = new Point (0,0);
        static final Point topLeftSecondArea_Little = new Point (110,0);
        static final Point topLeftThirdArea_Little = new Point (210,0);
        private volatile Position position;
        Point areaPoint1Top = new Point(topLeftFirstArea_Little.x, topLeftFirstArea_Little.y);
        Point areaPoint1Bottom = new Point(109,239);
        Point areaPoint2Top = new Point (topLeftSecondArea_Little.x,topLeftSecondArea_Little.y);
        Point areaPoint2Bottom = new Point (209,239);
        Point areaPoint3Top = new Point (topLeftThirdArea_Little.x, topLeftThirdArea_Little.y);
        Point areaPoint3Bottom = new Point(319,239);

        Mat regionLeft,regionMiddle,regionRight;
        Mat YCbCr = new Mat();
        Mat Cb = new Mat();

        int avg1,avg2,avg3;
        void extractCB(Mat input){
            Imgproc.cvtColor(input,YCbCr, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCbCr, Cb,2);
        }
        public void init(Mat firstFrame){
            extractCB(firstFrame);
            regionLeft = Cb.submat(new Rect (areaPoint1Top,areaPoint1Bottom));
            regionMiddle = Cb.submat(new Rect (areaPoint2Top,areaPoint2Bottom));
            regionRight = Cb.submat(new Rect (areaPoint3Top,areaPoint3Bottom));

        }
        public Mat processFrame(Mat input){
            extractCB(input);

            avg1 = (int)Core.mean(regionLeft).val[0];
            avg2 = (int)Core.mean(regionMiddle).val[0];
            avg3 = (int)Core.mean(regionRight).val[0];

            Imgproc.rectangle(input,areaPoint1Top,areaPoint1Bottom,BLUE);
            Imgproc.rectangle(input,areaPoint2Top,areaPoint2Bottom,BLUE);
            Imgproc.rectangle(input,areaPoint3Top,areaPoint3Bottom,BLUE);

            int avgOneTwo = Math.max(avg1, avg2);
            int avgMax = Math.max(avgOneTwo,avg3);

            if (avgMax == avg1){
                position = Position.LEFT;
                Imgproc.rectangle(input,areaPoint1Top,areaPoint1Bottom,RED,-1);
            }
            if(avgMax == avg2){
                position = Position.CENTER;
                Imgproc.rectangle(input,areaPoint2Top,areaPoint2Bottom,RED,-1);
            }
            if(avgMax == avg3){
                position = Position.RIGHT;
                Imgproc.rectangle(input,areaPoint3Top,areaPoint3Bottom,RED,-1);

            }
            return input;
        }


    }




}