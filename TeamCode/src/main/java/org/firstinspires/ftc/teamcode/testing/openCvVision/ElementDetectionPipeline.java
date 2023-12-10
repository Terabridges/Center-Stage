package org.firstinspires.ftc.teamcode.testing.openCvVision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ElementDetectionPipeline extends OpenCvPipeline {

    public enum Position {
        LEFT,
        CENTER,
        RIGHT
    }

    int WIDTH = 320;
    int HEIGHT = 240;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.rectangle(
                input,
                new Point(WIDTH/4,HEIGHT/4),
                new Point(WIDTH*3/4,WIDTH*3/4),
                new Scalar(0, 255, 255)
        );
        return input;
    }
}
