package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


public class OpenCVRed{

    //Focal points
    public static double cX = 0;
    public static double cY = 0;
    public static double width = 0;


    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.7;  //For future devs, replace with the actual width of the object in real-world units. Not necessary if you are only using object detection
    public static final double focalLength = 746.153846;  // Replace with the focal length of the camera in pixels (Shouldn't change if we are using the same camera)


    public static class RedBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect red regions
            Mat RedMask = preprocessFrame(input);

            // Find contours of the detected Red regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(RedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest Red contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a Red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);


            }
            if(largestContour != null && calculateWidth(largestContour)<90){
                cX = -10;
            }
            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            //In HSV form. Keep Saturation and Value the same for different colors
            Scalar lowerRed = new Scalar(100, 100, 100);
            Scalar upperRed = new Scalar(180, 255, 255);

            //Create a submatrix masking the contour
            Mat RedMask = new Mat();
            Core.inRange(hsvFrame, lowerRed, upperRed, RedMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(RedMask, RedMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(RedMask, RedMask, Imgproc.MORPH_CLOSE, kernel);

            return RedMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            //Loop through all contours to find the largest one, which is (most of the time) the prop
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            //OpenCV does math by calculating the width of the box of the contour
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    public static double getDistance(double width){
        //Uses distance using focal length. Yay physics and optics
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }


}