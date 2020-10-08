/*
 * Copyright (c) 2019 OpenFTC Team
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import static java.lang.Math.*;

@TeleOp
public class TestUvcPtz_v02 extends LinearOpMode
{
    OpenCvWebcam webcam;

    int resolutionWidth = 320;      // default settings, adjusted by user
    int resolutionHeight = 240;
    boolean useLimits = true;

    int maxPan;
    int minPan;
    int curPan = 0;
    int maxTilt;
    int minTilt;
    int curTilt = 0;
    int maxZoom;
    int minZoom;
    int curZoom = 0;

    PtzControl.PanTiltHolder curPanTilt = new PtzControl.PanTiltHolder();

    PtzControl ptzControl;


    @Override
    public void runOpMode()
    {
        telemetry.setMsTransmissionInterval(50);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(new SamplePipeline());

        while ( !gamepad1.y && !isStopRequested() ) {

            if (gamepad1.left_stick_y < -0.2)
                resolutionWidth += 1;
            else if (gamepad1.left_stick_y > 0.2)
                resolutionWidth -= 1;

            if (gamepad1.right_stick_y < -0.2)
                resolutionHeight += 1;
            else if (gamepad1.right_stick_y > 0.2)
                resolutionHeight -= 1;

            telemetry.addLine("Adjust webcam resolution with L & R joysticks");
            telemetry.addData("Width:", resolutionWidth);
            telemetry.addData("Height:", resolutionHeight);
            telemetry.addLine("Press Y to continue");
            telemetry.update();
            sleep(50);
    }
        webcam.startStreaming(resolutionWidth, resolutionHeight, OpenCvCameraRotation.UPRIGHT);

        while ( !gamepad1.x && !isStopRequested() ) {

            if (gamepad1.dpad_left)  useLimits = true;
            else if (gamepad1.dpad_right)  useLimits = false;

            telemetry.addLine("Use webcam-provided limits? Dpad L = Yes, R = No");
            telemetry.addData("Selection:", useLimits);
            telemetry.addLine("Press X to continue");
            telemetry.update();
            sleep(50);
        }

        telemetry.addLine("Ready to Start");
        telemetry.update();

        waitForStart();

        ptzControl = webcam.getPtzControl();

        PtzControl.PanTiltHolder maxPanTilt = ptzControl.getMaxPanTilt();
        maxPan = maxPanTilt.pan;
        maxTilt = maxPanTilt.tilt;
        PtzControl.PanTiltHolder minPanTilt = ptzControl.getMinPanTilt();
        minPan = minPanTilt.pan;
        minTilt = minPanTilt.tilt;
        maxZoom = ptzControl.getMaxZoom();
        minZoom = ptzControl.getMinZoom();

        while (!gamepad1.y && opModeIsActive()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addLine("Pan: LX; Tilt: LY; Zoom: RY");
            telemetry.addData("Pan", "Min:%d, Max:%d, Cur:%d", minPan, maxPan, curPan);
            telemetry.addData("Tilt", "Min:%d, Max:%d, Cur:%d", minTilt, maxTilt, curTilt);
            telemetry.addData("Zoom", "Min:%d, Max:%d, Cur:%d", minZoom, maxZoom, curZoom);
            telemetry.addData("Resolution", "Width:%d, Height:%d", resolutionWidth, resolutionHeight);
            telemetry.addData("Use webcam-provided limits?", useLimits);
            telemetry.addLine("Press Y to continue");
            telemetry.update();
            sleep(50);
        }

         while (opModeIsActive())
        {
            float changeTilt = gamepad1.left_stick_y;      // was negative
            float changePan = gamepad1.left_stick_x;
            float changeZoom = -gamepad1.right_stick_y;

            int changeTiltInt = (int) (changeTilt*800);
            int changePanInt = (int) (changePan*800);
            int changeZoomInt = (int) (changeZoom*2);      // was x10

            curTilt += changeTiltInt;
            curPan += changePanInt;
            curZoom += changeZoomInt;

            if (useLimits) {
                curPan = max(curPan, minPan);
                curPan = min(curPan, maxPan);

                curTilt = max(curTilt, minTilt);
                curTilt = min(curTilt, maxTilt);

                curZoom = max(curZoom, minZoom);
                curZoom = min(curZoom, maxZoom);
            }

            curPanTilt.pan = curPan;
            curPanTilt.tilt = curTilt;
            ptzControl.setPanTilt(curPanTilt);
            ptzControl.setZoom(curZoom);

            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addLine("Pan: LX; Tilt: LY; Zoom: RY");
            telemetry.addData("Pan", "Min:%d, Max:%d, Cur:%d", minPan, maxPan, curPan);
            telemetry.addData("Tilt", "Min:%d, Max:%d, Cur:%d", minTilt, maxTilt, curTilt);
            telemetry.addData("Zoom", "Min:%d, Max:%d, Cur:%d", minZoom, maxZoom, curZoom);
            telemetry.update();

            sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline
    {
        public Mat processFrame(Mat input)
        {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            return input;
        }
    }
}