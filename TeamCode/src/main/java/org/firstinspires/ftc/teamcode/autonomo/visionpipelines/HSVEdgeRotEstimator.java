package org.firstinspires.ftc.teamcode.autonomo.visionpipelines;
import java.util.ArrayList;
import org.opencv.core.Core;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.MatOfPoint;
import org.opencv.core.CvType;
import java.util.List;

public class HSVEdgeRotEstimator extends OpenCvPipeline {
    
    //color constants
    private static Scalar blue = new Scalar(7,197,235,255);
    private static Scalar red = new Scalar(150,0,0,255);
    private static Scalar green = new Scalar(0,255,0,255);
    private static Scalar white = new Scalar(255,255,255,255);
    /*
     * These are our variables that will be
     * modifiable from the variable tuner.
     *
     * Minimum and maximum on HSV sace for binary filter later
     */
    public Scalar lower = new Scalar(17, 0, 134);
    public Scalar upper = new Scalar(41, 255, 255);
	public int minsize = 150;
    
    /*
     * CAMERA INTRINSICS
     * static constants for camera distortion
     * UNITS ARE PIXELS
     * NOTE: this calibration is for the C920 webcam at 800x448.
     * You will need to do your own calibration for other configurations!
     */
    
    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;
    
    // UNITS ARE METERS
    //public static double TAG_SIZE = 0.166;
    
    /*
     * A good practice when typing EOCV pipelines is
     * declaring the Mats you will use here at the top
     * of your pipeline, to reuse the same buffers every
     * time. This removes the need to call mat.release()
     * with every Mat you create on the processFrame method,
     * and therefore, reducing the possibility of getting a
     * memory leak and causing the app to crash due to an
     * "Out of Memory" error.
     */
    
	private Mat blurredIn = new Mat();
	private Mat HSVmat = new Mat();
	private Mat binaryMat = new Mat();
	private Mat edges = new Mat();
	private List<MatOfPoint> contours = new ArrayList<>();
	private Mat hierarchy = new Mat();
	//private Mat maskedInputMat = new Mat();
	private Mat element1 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(20,20));
    
    private Mat cameraMatrix;
    private Pose pose = null;
    private double tagsizeX = 0.05;
    private double tagsizeY = 0.05;
	
	public HSVEdgeRotEstimator() {
        constructMatrix();
    }
    @Override
    public Mat processFrame(Mat input) {
        //blur image
		Imgproc.GaussianBlur(input, blurredIn, new Size(5, 5), 0);
		//convert to hsv color space
        Imgproc.cvtColor(blurredIn, HSVmat, Imgproc.COLOR_RGB2HSV);
		//binary filter
		Core.inRange(HSVmat, lower, upper, binaryMat);
		//filtering as to reduce noise
        //reduce size of "blobs" and then dilate them back
		Imgproc.erode(binaryMat, binaryMat, element1);
		Imgproc.dilate(binaryMat, binaryMat, element1);
		//find edges and contours
		Imgproc.Canny(binaryMat, edges, 15, 15*3);
		contours.clear();
		Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
		
        //if wanted, show only part of image inside filter
		//maskedInputMat.release();
		//Core.bitwise_and(input, input, maskedInputMat, binaryMat);
		for (int i = 0; i < contours.size(); i++) {
			Mat contour = contours.get(i);
			MatOfPoint mpoints = new MatOfPoint(contour);
			MatOfPoint2f contour2f = new MatOfPoint2f(mpoints.toArray());
			RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
			drawRotatedRect(rotatedRectFitToContour, input, new Scalar(255,0,255), new Scalar(255,132,0));
            Imgproc.drawContours(input, contours, i, new Scalar(255, 0, 0), -1);
			//Imgproc.rectangle (maskedInputMat, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), color, 1);
            //3D SOLVE PNP OPERATION
            pose = poseFromTrapezoid(rotatedRectFitToContour, cameraMatrix, tagsizeX, tagsizeY);
            drawAxisMarker(input, tagsizeY/2.0, 3, pose.rvec, pose.tvec, cameraMatrix);
        }
		
		
        //Core.inRange(ycrcbMat, lower, upper, binaryMat);
        //maskedInputMat.release();
        //Core.bitwise_and(input, input, maskedInputMat, binaryMat);
        return input;
    }
	static void drawRotatedRect(RotatedRect rect, Mat drawOn, Scalar lineColor, Scalar edgeColor)
    {
        /*
         * Draws a rotated rect by drawing each of the 4 lines individually
         */

        Point[] points = new Point[4];
        rect.points(points);

        for(int i = 0; i < 4; ++i)
        {
            Imgproc.line(drawOn, points[i], points[(i+1)%4], lineColor, 1);
            Imgproc.circle(drawOn, points[i], 5, edgeColor);
        }
    }
    public Mat getAnalysis(){
	    return pose.tvec;
    }
    void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0,0,0),
                new Point3(length,0,0),
                new Point3(0,length,0),
                new Point3(0,0,-length)
        );

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
    }
    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    class Pose
    {
        Mat rvec;
        Mat tvec;

        public Pose()
        {
            rvec = new Mat();
            tvec = new Mat();
        }

        public Pose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }
    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }
    
    Pose poseFromTrapezoid(RotatedRect rect, Mat cameraMatrix, double tagsizeX , double tagsizeY)
    {
        Point[] points = new Point[4];
        rect.points(points);
        // The actual 2d points of the rect detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the rect in an 'ideal projection'
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
        arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Using this information, actually solve for pose
        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }
}