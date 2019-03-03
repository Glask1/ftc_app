package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.invictarobotics.invictavision.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class InvictaCVHull extends OpenCVPipeline {

    private ElapsedTime frametime = new ElapsedTime();
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)
    private Mat workingMat = new Mat();
    public double fps;
    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    public double error;
    public double area = 0;
    public boolean found = false;

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
        contours.clear();

        Imgproc.findContours(workingMat, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        if(contours.size() > 0) {
        // Finds the largest contour within the contour list
        double maxVal = 0;
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
        {
            double contourArea = Imgproc.contourArea(contours.get(contourIdx));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }

            MatOfInt hull = new MatOfInt();
            MatOfPoint tempContour = contours.get(maxValIdx);
            Imgproc.convexHull(tempContour, hull, false); // O(N*Log(N))

            ArrayList<Point> hullpoints = new ArrayList<>();

            int index = (int) hull.get(((int) hull.size().height) - 1, 0)[0];
            Point pt, pt0 = new Point(tempContour.get(index, 0)[0], tempContour.get(index, 0)[1]);
            hullpoints.add(pt0);
            for (int j = 0; j < hull.size().height - 1; j++) {
                index = (int) hull.get(j, 0)[0];
                pt = new Point(tempContour.get(index, 0)[0], tempContour.get(index, 0)[1]);
                hullpoints.add(pt);
                Imgproc.line(displayMat, pt0, pt, new Scalar(255, 0, 100), 8);
                pt0 = pt;
            }

            area = 0;
            for (int i = 0; i < hullpoints.size(); i++){
                int next_i = (i+1)%(hullpoints.size());
                double dX   = hullpoints.get(next_i).x - hullpoints.get(i).x;
                double avgY = (hullpoints.get(next_i).y + hullpoints.get(i).y)/2;
                area += dX*avgY;  // This is the integration step.
            }

            double averageX = 0;
            double averageY = 0;
            for(Point p: hullpoints) {
                averageX += p.x;
                averageY += p.y;
            }
            averageX /= hullpoints.size();
            averageY /= hullpoints.size();

            Point center = new Point(averageX, averageY);
            Imgproc.circle(displayMat, center, 2, new Scalar(0,0,255),4);

            if(Math.abs(area) > 500) {
                found = true;
                error = imageCenter.y - center.y;
            } else {
                found = false;
            }

        }

        return displayMat;

    }

}
