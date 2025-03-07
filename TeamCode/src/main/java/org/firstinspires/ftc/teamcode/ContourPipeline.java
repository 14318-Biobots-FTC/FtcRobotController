package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;



public class ContourPipeline extends OpenCvPipeline
{
    Scalar BLUE = new Scalar(196, 150, 255);

    // Pink, the default color                         Y      Cr     Cb    (Do not change Y)
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0, 50);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120, 130);
    // These values define the Range of color, for example green is a color "in between" lightgreen and darkgreen.

    // Green                                             Y      Cr     Cb
    // public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0.0, 0.0);
    // public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);
    // use this picture for you own color https://github.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022/blob/main/YCbCr.jpeg
    // Note that the Cr and Cb values range between 0-255. this means that the origin of the coordinate system is (128,128)

    public boolean error = false;
    public Exception debug;

    private int borderLeftX   = 0;   //amount of pixels from the left side of the cam to skip
    private int borderRightX  = 0;   //amount of pixels from the right of the cam to skip
    private int borderTopY    = 0;   //amount of pixels from the top of the cam to skip
    private int borderBottomY = 0;   //amount of pixels from the bottom of the cam to skip

    private int CAMERA_WIDTH = 640;
    private int CAMERA_HEIGHT = 360;

    private int loopcounter = 0;
    private int ploopcounter = 0;

    private Mat mat = new Mat();
    Mat output=new Mat();
    private Mat processed = new Mat();

    private Rect maxRect = new Rect();

    private double maxArea = 0;
    //changed from 0 to 9
    private boolean first = false;

    public void ConfigurePipeline(int borderLeftX, int borderRightX, int borderTopY, int borderBottomY, int CAMERA_WIDTH, int CAMERA_HEIGHT)
    {
        this.borderLeftX = borderLeftX;
        this.borderRightX = borderRightX;
        this.borderTopY = borderTopY;
        this.borderBottomY = borderBottomY;
        this.CAMERA_WIDTH = CAMERA_WIDTH;
        this.CAMERA_HEIGHT = CAMERA_HEIGHT;
    }

    public void ConfigureScalarLower(double Y, double Cr, double Cb) { scalarLowerYCrCb = new Scalar(Y, Cr, Cb); }
    public void ConfigureScalarUpper(double Y, double Cr, double Cb) { scalarUpperYCrCb = new Scalar(Y, Cr, Cb); }
    public void ConfigureScalarLower(int Y, int Cr, int Cb) { scalarLowerYCrCb = new Scalar(Y, Cr, Cb); }
    public void ConfigureScalarUpper(int Y, int Cr, int Cb) { scalarUpperYCrCb = new Scalar(Y, Cr, Cb); }

    @Override
    public Mat processFrame(Mat input)
    {

        input.copyTo(output);
        try
        {
            // Process Image
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
            Core.inRange(mat, scalarLowerYCrCb, scalarUpperYCrCb, processed);
            // Core.bitwise_and(input, input, output, processed);

            // Remove Noise
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
            // GaussianBlur
            Imgproc.GaussianBlur(processed, processed, new Size(3.0, 15.0), 0.00);
            // 15 --> 30.0 (height)
            // Find Contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            // Draw Contours
            Imgproc.drawContours(output, contours, -1, new Scalar(255, 0, 0));

            // Loop Through Contours
            for (MatOfPoint contour : contours)
            {
                Point[] contourArray = contour.toArray();

                // Bound Rectangle if Contour is Large Enough
                if (contourArray.length >= 15)
                {
                    MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                    Rect rect = Imgproc.boundingRect(areaPoints);

                    // if rectangle is larger than previous cycle or if rectangle is not larger than previous 6 cycles > then replace
                    if (rect.area() > maxArea
                            && rect.x > borderLeftX && rect.x + rect.width < CAMERA_WIDTH - borderRightX
                            && rect.y > borderTopY && rect.y + rect.height < CAMERA_HEIGHT - borderBottomY
                            || loopcounter - ploopcounter > 6)
                    {
                        maxArea = rect.area();
                        maxRect = rect;
                        ploopcounter++;
                        loopcounter = ploopcounter;
                        first = true;
                    }
                    areaPoints.release();
                }
                contour.release();
            }
            mat.release();
            processed.release();
            if (contours.isEmpty())
            {
                maxRect = new Rect();
            }
            // Draw Rectangles If Area Is At Least 500
            if (first && maxRect.area() > 500)
            {
                Imgproc.rectangle(output, maxRect, new Scalar(0, 255, 0), 2);
            }
            // Draw Borders
            Imgproc.rectangle(output, new Rect(borderLeftX, borderTopY, CAMERA_WIDTH - borderRightX - borderLeftX, CAMERA_HEIGHT - borderBottomY - borderTopY), BLUE, 2);
            // Display Data
            Imgproc.putText(output, "Area: " + getRectArea() + " Midpoint: " + getRectMidpointXY().x + " , " + getRectMidpointXY().y, new Point(5, CAMERA_HEIGHT - 5), 0, 0.6, new Scalar(255, 255, 255), 2);

            loopcounter++;
        } catch (Exception e) {
            debug = e;
            error = true;
        }
        //input.release();
        return output;
    }
    public int getRectHeight(){return maxRect.height;}
    public int getRectWidth(){ return maxRect.width; }
    public int getRectX(){ return maxRect.x; }
    public int getRectY(){ return maxRect.y; }
    public double getRectMidpointX(){ return getRectX() + (getRectWidth()/2.0); }
    public double getRectMidpointY(){ return getRectY() + (getRectHeight()/2.0); }
    public Point getRectMidpointXY(){ return new Point(getRectMidpointX(), getRectMidpointY());}
    public double getAspectRatio(){ return getRectArea()/(CAMERA_HEIGHT*CAMERA_WIDTH); }
    public double getRectArea(){ return maxRect.area(); }
}