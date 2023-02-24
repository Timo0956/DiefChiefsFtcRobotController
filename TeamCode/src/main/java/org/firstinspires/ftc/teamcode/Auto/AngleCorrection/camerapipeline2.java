/*package org.firstinspires.ftc.teamcode.Auto.AngleCorrection;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class camerapipeline2 extends OpenCvPipeline {
    Mat YCbCr = new Mat();
    Mat leftCrop;
    Mat rightCrop;
    double leftavgfin;
    double rightavgfin;
    Mat output = new Mat();
    Scalar rectColor = new Scalar(255.0,0.0,0.0);
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input,YCbCr, Imgproc.COLOR_RGB2YCrCb);
        telemetry.addLine("pipe running");


    }
}*/
