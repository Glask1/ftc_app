package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

import static org.opencv.core.CvType.CV_8U;
import static org.opencv.core.CvType.CV_8UC1;

public class InvictaVision extends VuforiaLocalizerImpl {

    public ArrayList<Point> mineralOrder = new ArrayList<>();
    public Double averageGoldX = null;
    private Grip grip = new Grip();
    public double resizeImageWidth;

    public boolean pan = true;

    private boolean isDisabled = true;

    private Thread workerThread;

    private Grip.BlurType BLUR_TYPE = Grip.BlurType.get("Median Filter");

    private double[] BLOBS_CIRCULARITY = {0.0, 1.0};
    private double[] HSV_THRESHOLD_HUE = {0, 68};
    private double[] HSV_THRESHOLD_SATURATION = {62, 255};
    private double[] HSV_THRESHOLD_VALUE = {190, 255.0};

    public InvictaVision(Parameters p) {
        super(p);
    }

    public void disable() {
        isDisabled = true;
    }

    public void enable() {
        isDisabled = false;
    }

    public void start() {
        workerThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!workerThread.isInterrupted() && !isDisabled) {
                        render();
                }

            }
        });
        workerThread.setName("InvictaVision");
        workerThread.start();
    }

    private void render() {
        if (!isDisabled) {

            //Creating an empty bitmap to store what we fetch from vuforia.
            Bitmap bm = null;

            //We dequeue the top of the queue, getting the most recent frame. Then convert to BMP. If it is null, then we do it agane until it is not null.
            if (!getFrameQueue().isEmpty()) {
                try {
                    bm = convertFrameToBitmap(getFrameQueue().take());
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                //Converting the BMP to Matrix for use with OpenCV
                Mat m = new Mat(bm.getWidth(), bm.getHeight(), CvType.CV_8UC1);
                Utils.bitmapToMat(bm, m);

                //Here we resize the image using resizeImage taken from GRIP
                resizeImageWidth = m.width() / 4;
                double resizeImageHeight = m.height() / 4;
                Mat resizeImageOutput = new Mat();
                grip.resizeImage(m, resizeImageWidth, resizeImageHeight, Imgproc.INTER_NEAREST, resizeImageOutput);

                //Blurring the image
                //Mat blurOutput = new Mat();
                //double BLUR_RADIUS = 6;
                //grip.blur(m, BLUR_TYPE, BLUR_RADIUS, blurOutput);

                //Finding blobs
                //MatOfKeyPoint findBlobsOutput = new MatOfKeyPoint();
                //double BLOBS_MIN_AREA = 250;
                //grip.findBlobs(blurOutput, BLOBS_MIN_AREA, BLOBS_CIRCULARITY, false, findBlobsOutput);

                //Applying HSV threshold
                Mat hsvThresholdOutput = new Mat();
                grip.hsvThreshold(resizeImageOutput, HSV_THRESHOLD_HUE, HSV_THRESHOLD_SATURATION, HSV_THRESHOLD_VALUE, hsvThresholdOutput);

                //Blurring the image
                Mat blurOutput = new Mat();
                double BLUR_RADIUS = 12;
                grip.blur(hsvThresholdOutput, BLUR_TYPE, BLUR_RADIUS, blurOutput);

                //Finding blobs
                MatOfKeyPoint findBlobsGoldOutput = new MatOfKeyPoint();
                grip.findBlobs(blurOutput, 1, BLOBS_CIRCULARITY, false, findBlobsGoldOutput);

                //We turn our Blob mat to an array for use
                //KeyPoint[] silverPoints = findBlobsOutput.toArray();
                KeyPoint[] goldPoints = findBlobsGoldOutput.toArray();
                if(goldPoints.length > 0)
                    pan = false;

                //Creating arraylists to manipulate elements easier
                //ArrayList<KeyPoint> silverBalls = new ArrayList<>();
                ArrayList<KeyPoint> goldBalls = new ArrayList<>(Arrays.asList(goldPoints));
                //ArrayList<Point> minerals = new ArrayList<>();

                //Sorting goldBalls from greatest to least
                Collections.sort(goldBalls, new Comparator<KeyPoint>() {
                    public int compare(KeyPoint keyPoint, KeyPoint t1) {
                        return -1 * Integer.compare((int) keyPoint.size, (int) t1.size);
                    }
                });

                //If the size is greater than 0, we add the first ball to minerals
                if (goldBalls.size() > 0) {
                    pan = false;
                    Point n = new Point(goldBalls.get(0).pt.x, 0);
                    //minerals.add(n);
                    averageGoldX = goldBalls.get(0).pt.x;
                } else {
                    pan = true;
                }
            /*
            //Deleting extra blobs that may be overlapping
            for (KeyPoint kp : silverPoints) {
                double distSq = Math.pow(kp.pt.x - averageGoldX, 2) + Math.pow(kp.pt.y - 20, 2);
                double radSq = Math.pow(kp.size / 2 + 25, 2);
                if (distSq > radSq) {
                    silverBalls.add(kp);
                }
            }

            //sorting the blobs by size, from greatest to least
            Collections.sort(silverBalls, new Comparator<KeyPoint>() {
                public int compare(KeyPoint keyPoint, KeyPoint t1) {
                    return -1 * Integer.compare((int) keyPoint.size, (int) t1.size);
                }
            });

            //Keeping the two largest blobs
            if (silverBalls.size() > 1) {
                silverBalls.subList(2, silverBalls.size()).clear();
            }

            //Adding the two largest to our final list of minerals
            for (KeyPoint b : silverBalls) {
                Point n = new Point(b.pt.x, 0);
                minerals.add(n);
            }

            //Sorting minerals X values from least to greatest
            Collections.sort(minerals, new Comparator<Point>() {
                public int compare(Point keyPoint, Point t1) {
                    return Integer.compare((int) keyPoint.x, (int) t1.x);
                }
            });

            mineralOrder = minerals;
            */
            }
        }
    }
}
