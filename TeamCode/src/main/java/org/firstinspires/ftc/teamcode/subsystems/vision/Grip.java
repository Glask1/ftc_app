package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.graphics.Bitmap;
import android.os.Environment;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;

import static java.lang.Math.floor;

class Grip {
    /**
     * Softens an image using one of several filters.
     *
     * @param input        The image on which to perform the blur.
     * @param type         The blurType to perform.
     * @param doubleRadius The radius for the blur.
     * @param output       The image in which to store the output.
     */
    void blur(Mat input, BlurType type, double doubleRadius,
              Mat output) {
        int radius = (int) (doubleRadius + 0.5);
        int kernelSize;
        switch (type) {
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input, output, new Size(kernelSize, kernelSize), radius);
                break;
            case MEDIAN:
                kernelSize = 2 * radius + 1;
                Imgproc.medianBlur(input, output, kernelSize);
                break;
            case BILATERAL:
                Imgproc.bilateralFilter(input, output, -1, radius, radius);
                break;
        }
    }

    /**
     * Detects groups of pixels in an image.
     *
     * @param input       The image on which to perform the find blobs.
     * @param minArea     The minimum size of a blob that will be found
     * @param circularity The minimum and maximum circularity of blobs that will be found
     * @param darkBlobs   The boolean that determines if light or dark blobs are found.
     * @param blobList    The output where the MatOfKeyPoint is stored.
     */
    void findBlobs(Mat input, double minArea, double[] circularity,
                   Boolean darkBlobs, MatOfKeyPoint blobList) {
        FeatureDetector blobDet = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
        try {
            File tempFile = File.createTempFile("config", ".xml");

            String config = "<?xml version=\"1.0\"?>\n" +
                    "<opencv_storage>\n" +
                    "<thresholdStep>10.</thresholdStep>\n" +
                    "<minThreshold>50.</minThreshold>\n" +
                    "<maxThreshold>220.</maxThreshold>\n" +
                    "<minRepeatability>2</minRepeatability>\n" +
                    "<minDistBetweenBlobs>10.</minDistBetweenBlobs>\n" +
                    "<filterByColor>1</filterByColor>\n" +
                    "<blobColor>" +
                    (darkBlobs ? 0 : 255) +
                    "</blobColor>\n" +
                    "<filterByArea>1</filterByArea>\n" +
                    "<minArea>" +
                    minArea +
                    "</minArea>\n" +
                    "<maxArea>" +
                    Integer.MAX_VALUE +
                    "</maxArea>\n" +
                    "<filterByCircularity>1</filterByCircularity>\n" +
                    "<minCircularity>" +
                    circularity[0] +
                    "</minCircularity>\n" +
                    "<maxCircularity>" +
                    circularity[1] +
                    "</maxCircularity>\n" +
                    "<filterByInertia>1</filterByInertia>\n" +
                    "<minInertiaRatio>0.1</minInertiaRatio>\n" +
                    "<maxInertiaRatio>" + Integer.MAX_VALUE + "</maxInertiaRatio>\n" +
                    "<filterByConvexity>1</filterByConvexity>\n" +
                    "<minConvexity>0.95</minConvexity>\n" +
                    "<maxConvexity>" + Integer.MAX_VALUE + "</maxConvexity>\n" +
                    "</opencv_storage>\n";

            FileWriter writer;
            writer = new FileWriter(tempFile, false);
            writer.write(config);
            writer.close();
            blobDet.read(tempFile.getPath());
        } catch (IOException e) {
            e.printStackTrace();
        }

        blobDet.detect(input, blobList);
    }

    //GRIP Function
    void resizeImage(Mat input, double width, double height,
                     int interpolation, Mat output) {
        Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
    }

    //GRIP Function
    void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                      Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }

    //Taken from StackOverflow, added in random image naming so it doesnt overwrite
    private boolean saveImage(Bitmap finalBitmap) {

        String root = Environment.getExternalStorageDirectory().toString();
        File myDir = new File(root + "/saved_images");
        myDir.mkdirs();
        double random = floor(Math.random() * 500 + 1);
        String fname = "tmp" + random + ".jpg";
        File file = new File(myDir, fname);
        try {
            FileOutputStream out = new FileOutputStream(file);
            finalBitmap.compress(Bitmap.CompressFormat.JPEG, 90, out);
            out.flush();
            out.close();
            return true;

        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    /**
     * An indication of which type of filter to use for a blur.
     * Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
     */
    public enum BlurType {
        BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
        BILATERAL("Bilateral Filter");

        private final String label;

        BlurType(String label) {
            this.label = label;
        }

        public static BlurType get(String type) {
            if (BILATERAL.label.equals(type)) {
                return BILATERAL;
            } else if (GAUSSIAN.label.equals(type)) {
                return GAUSSIAN;
            } else if (MEDIAN.label.equals(type)) {
                return MEDIAN;
            } else {
                return BOX;
            }
        }

        @Override
        public String toString() {
            return this.label;
        }
    }

}
