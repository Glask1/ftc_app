package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.invictarobotics.invictavision.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.core.Core.findNonZero;

public class InvictaCV extends OpenCVPipeline {

    ElapsedTime frametime = new ElapsedTime();
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)
    private Mat workingMat = new Mat(); // Used for preprocessing and working with (blurring as an example)
    public double fps;
    public double error;

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {

        Point imageCenter = new Point(rgba.width() / 2, rgba.height() / 2);

        fps = 1/frametime.seconds();
        frametime.reset();

        rgba.copyTo(displayMat);
        rgba.copyTo(workingMat);
        rgba.release();

        Imgproc.cvtColor(workingMat, workingMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(workingMat, new Scalar(0, 62, 190),
                new Scalar(68, 255, 255), workingMat);
        MatOfPoint points = new MatOfPoint();
        Core.findNonZero(workingMat,points);
        Rect rc = Imgproc.boundingRect(points);
        Imgproc.rectangle(displayMat, rc.tl(), rc.br(), new Scalar(255,0,0),4);
        Point center = new Point(rc.x + rc.width / 2, rc.y + rc.height / 2);
        Imgproc.circle(displayMat, center, 2, new Scalar(0,0,255),4);
        error = imageCenter.y - center.y;

        return displayMat;

    }

}
