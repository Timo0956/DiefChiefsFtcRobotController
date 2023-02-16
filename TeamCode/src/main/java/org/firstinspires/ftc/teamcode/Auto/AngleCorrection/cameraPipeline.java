package org.firstinspires.ftc.teamcode.Auto.AngleCorrection;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class cameraPipeline extends OpenCvPipeline {
    String Position;
    Mat YCbCr = new Mat();
    Mat leftCrop;
    Mat middleCrop;
    Mat rightCrop;
    double leftavgfin;
    double rightavgfin;
    double middleavgfin;
    Mat outPut = new Mat();
    Scalar rectColor = new Scalar(255.0,0.0,0.0);


    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
        telemetry.addLine("running");

        Rect leftRect = new Rect(1,1,389,448);
        Rect middleRect = new Rect (390,1,20,448);
        Rect rightRect = new Rect (411,1,389,448);


        input.copyTo(outPut);
        Imgproc.rectangle(outPut, leftRect, rectColor, 2);
        Imgproc.rectangle(outPut, middleRect, rectColor, 2);
        Imgproc.rectangle(outPut, rightRect, rectColor, 2);

        leftCrop = YCbCr.submat(leftRect);
        rightCrop = YCbCr.submat(rightRect);
        middleCrop = YCbCr.submat(middleRect);

        Core.extractChannel(leftCrop,leftCrop,2);
        Core.extractChannel(rightCrop,rightCrop,2);
        Core.extractChannel(middleCrop, middleCrop,2);

        Scalar leftavg = Core.mean(leftCrop);
        Scalar rightavg = Core.mean(rightCrop);
        Scalar middleavg = Core.mean(middleCrop);

        leftavgfin = leftavg.val[0];
        rightavgfin = rightavg.val[0];
        middleavgfin = middleavg.val[0];

        if (leftavgfin > middleavgfin && leftavgfin > rightavgfin){
            telemetry.addLine("Left");
            camera.posRobot("left");
        }
        else if (rightavgfin > middleavgfin && rightavgfin > leftavgfin){
            telemetry.addLine("right");
            camera.posRobot("right");

        }
        else if (middleavgfin > leftavgfin && middleavgfin > rightavgfin){
            telemetry.addLine("middle");
            camera.posRobot("middle");
        }

        return (outPut);
    }

}
