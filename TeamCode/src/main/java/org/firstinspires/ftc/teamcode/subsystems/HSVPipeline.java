/**
 * Created by Ability Edge#18273
 * - Elior Yousefi, and Eitan Kravets
 */
package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.List;

@Config
public class HSVPipeline extends OpenCvPipeline
{

    // yellow
    public static double lowHLeftSleeve = 17;
    public static double highHLeftSleeve = 75;
    public static double lowSLeftSleeve = 97;
    public static double highSLeftSleeve = 217;
    public static double lowVLeftSleeve = 122;
    public static double highVLeftSleeve = 240;

    public static double lowHCenterSleeve = 17;
    public static double highHCenterSleeve = 75;
    public static double lowSCenterSleeve = 97;
    public static double highSCenterSleeve = 217;
    public static double lowVCenterSleeve = 122;
    public static double highVCenterSleeve = 240;

    public static double lowHRightSleeve = 17;
    public static double highHRightSleeve = 75;
    public static double lowSRightSleeve = 97;
    public static double highSRightSleeve = 217;
    public static double lowVRightSleeve = 122;
    public static double highVRightSleeve = 240;

    // creating a mast with the same resolution of the webcam for the place to display the detected team shipping element
    Mat maskLeft = new Mat(1280,720,0);//
    Mat maskCenter = new Mat(1280,720,0);//
    Mat maskRight = new Mat(1280,720,0);//
    Mat inputHSVLeft = new Mat(1280,720,0);
    Mat inputHSVCenter = new Mat(1280,720,0);
    Mat inputHSVRight = new Mat(1280,720,0);

    final Rect VISION_SECTION = new Rect(
            new Point(240,0),
            new Point(480,720));

    public static boolean DEBUG = true;

    public static double threshold = 1000;

    // the list of locations that can be
    public enum Location{
        Left,
        Center,
        Right,
        Not_Found
    }

    public Location locationToPark = Location.Not_Found;

    // color for the rectangles to show on the screen
    Scalar colorBarcodeRect = new Scalar(0, 255, 0);

    public Telemetry telemetry;

    boolean parkLeft, parkCenter, parkRight;

    @Override
    public Mat processFrame(Mat input) {
        // HSV low and high values for our team shipping element.
        Scalar lowValuesLeft = new Scalar(lowHLeftSleeve, lowSLeftSleeve, lowVLeftSleeve);
        Scalar highValuesLeft = new Scalar(highHLeftSleeve, highSLeftSleeve, highVLeftSleeve);

        Scalar lowValuesCenter = new Scalar(lowHCenterSleeve, lowSCenterSleeve, lowVCenterSleeve);
        Scalar highValuesCenter = new Scalar(highHCenterSleeve, highSCenterSleeve, highVCenterSleeve);

        Scalar lowValuesRight = new Scalar(lowHRightSleeve, lowSRightSleeve, lowVRightSleeve);
        Scalar highValuesRight = new Scalar(highHRightSleeve, highSRightSleeve, highVRightSleeve);

        Imgproc.cvtColor(input, inputHSVLeft, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, inputHSVCenter, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, inputHSVRight, Imgproc.COLOR_RGB2HSV);

        Core.inRange(inputHSVLeft, lowValuesLeft, highValuesLeft, maskLeft);
        Core.inRange(inputHSVCenter, lowValuesCenter, highValuesCenter, maskCenter);
        Core.inRange(inputHSVRight, lowValuesRight, highValuesRight, maskRight);

        Mat mask = null;

        // taking sections from the mask to another mat
        Mat left = maskLeft.submat(VISION_SECTION);
        Mat center = maskCenter.submat(VISION_SECTION);
        Mat right = maskRight.submat(VISION_SECTION);

        parkLeft = Core.countNonZero(left) > threshold;
        parkCenter = Core.countNonZero(center) > threshold;
        parkRight = Core.countNonZero(right) > threshold;

        // we release the mats for use.
        left.release();
        center.release();
        right.release();

        //checking which barcode is found.
        if(parkLeft && parkCenter && parkRight){
            // NOT FOUND
            setLocation(Location.Not_Found);
        }else if(parkLeft){
            setLocation(Location.Left);
        }else if(parkCenter){
            setLocation(Location.Center);
        }else if(parkRight){
            setLocation(Location.Right);
        }else{
            // NOT FOUND
            setLocation(Location.Not_Found);
        }

        Imgproc.cvtColor(maskLeft, maskLeft, Imgproc.COLOR_GRAY2RGB);
        Imgproc.cvtColor(maskCenter, maskCenter, Imgproc.COLOR_GRAY2RGB);
        Imgproc.cvtColor(maskRight, maskRight, Imgproc.COLOR_GRAY2RGB);

        // creating 3 rectangles at the mask mat with the color @colorBarcodeRect@ and the rectangle points LEFT/CENTER/RIGHT.
        Imgproc.rectangle(maskLeft, VISION_SECTION, colorBarcodeRect);
        Imgproc.rectangle(maskCenter, VISION_SECTION, colorBarcodeRect);
        Imgproc.rectangle(maskRight, VISION_SECTION, colorBarcodeRect);

        List<Mat> masks = Arrays.asList(new Mat[]{maskLeft, maskCenter, maskRight});
        Core.merge(masks, mask);

        return mask;
    }

    // getting location of the team shipping element.
    public Location getLocation() {
        return locationToPark;
    }

    public void setLocation(Location location) {
        this.locationToPark = location;
    }

    public void setTelementry(Telemetry telemetry){ this.telemetry = telemetry; }
}