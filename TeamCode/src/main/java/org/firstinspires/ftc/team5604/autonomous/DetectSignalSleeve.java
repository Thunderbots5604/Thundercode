/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.team5604.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */
@TeleOp
public class DetectSignalSleeve extends LinearOpMode
{
    OpenCvWebcam webcam;
    SignalPipeline pipeline;
    Telemetry telemetry;

    public DetectSignalSleeve(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SignalPipeline();
        this.telemetry = telemetry;
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    public int findRegion() {
        SignalPipeline.SignalRegion signal = pipeline.getAnalysis();
        int hValue = pipeline.getHValue();
        int location = 0;
        //Regions are 1-3 for convenience rather than the standard 0-2
        switch (signal) {
            case ONE:
                location = 1;
                break;
            case TWO:
                location = 2;
                break;
            case THREE:
                location = 3;
                break;
            default:
                location = 0;
                break;
        }
        telemetry.addData("Detected location", location);
        telemetry.addData("Detected hue", hValue);
        telemetry.update();
        return location;
    }

    @Override
    public void runOpMode()
    {
    }

    public static class SignalPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum SignalRegion
        {
            EMPTY,
            ONE,
            TWO,
            THREE
        }
        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar WHITE = new Scalar(255, 255, 255);

        /*
         * The core values which define the location and size of the sample regions
         */
        //Calibrate region
        static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(156,35);
        static final int REGION_WIDTH = 16;
        static final int REGION_HEIGHT = 24;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region_pointA = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x,
                REGION_TOPLEFT_ANCHOR_POINT.y);
        Point region_pointB = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Mat region;
        Mat HSV = new Mat();
        Mat H = new Mat();
        int avg;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile SignalRegion scan = SignalRegion.EMPTY;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToH(Mat input)
        {
            /*
            Imgproc.cvtColor(input, , Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, 1);
            Core.extractChannel(YCrCb, Cb, 2);
             */
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(HSV, H, 0);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToH(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            /*
            region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cr = Cr.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cr = Cr.submat(new Rect(region3_pointA, region3_pointB));
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
             */
            region = H.submat(new Rect(region_pointA, region_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToH(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            /*
            avg1Cr = (int) Core.mean(region1_Cr).val[0];
            avg2Cr = (int) Core.mean(region2_Cr).val[0];
            avg3Cr = (int) Core.mean(region3_Cr).val[0];
            avg1Cb = (int) Core.mean(region1_Cb).val[0];
            avg2Cb = (int) Core.mean(region2_Cb).val[0];
            avg3Cb = (int) Core.mean(region3_Cb).val[0];
             */
            avg = (int) Core.mean(region).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region_pointA, // First point which defines the rectangle
                    region_pointB, // Second point which defines the rectangle
                    WHITE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            /*
             * Find the max of the 3 averages
             */
            // The target variable is the target H or hue value: even though 60 is technically the correct color space for green, 50 is closer to the actual shade we have in lighting.

            int accRed = Math.min(avg, Math.abs(avg - 179));
            int accGreen = Math.abs(avg - 60);
            int accBlue = Math.abs(avg - 120);
            int minRedGreen = Math.min(accRed, accGreen);
            int min = Math.min(minRedGreen, accBlue);
            // Find which color the measured hue was closest to
            if (min == accRed) {
                scan = SignalRegion.ONE;
            } else if (min == accBlue) {
                scan = SignalRegion.TWO;
            } else if (min == accGreen) {
                scan = SignalRegion.THREE;
            }
            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public SignalRegion getAnalysis()
        {
            return scan;
        }
        public int getHValue() {
            return avg;
        }
    }
}