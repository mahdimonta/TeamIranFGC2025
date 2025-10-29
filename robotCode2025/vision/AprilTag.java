package org.firstinspires.ftc.teamcode.robotCode2025.vision;


import android.util.Size;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robotCode2025.motorController.RobotController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import global.first.EcoEquilibriumGameDatabase;

public class AprilTag {


    CRServo upDown, rightLeft;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    double servoPowerX,servoPowerY;


    public AprilTagProcessor aprilTag;

    public VisionPortal visionPortal;

    public void initAprilTag(HardwareMap hardwareMap) {

        upDown = hardwareMap.get(CRServo  .class, "upDown");
        rightLeft = hardwareMap.get(CRServo.class, "rightLeft");

        upDown.setPower(0);

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(EcoEquilibriumGameDatabase.getEcoEquilibriumTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();


        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.setCameraResolution(new Size(640, 480));
            builder.enableLiveView(true);


        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);

        }


        builder.addProcessor(aprilTag);

        visionPortal = builder.build();


        // Set initial exposure for better detection
        setManualExposure(6, 24);  // Adjust based on lighting conditions

    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return; // Camera not ready
        }
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50); // Small delay to ensure mode switch
        }
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }


    // Helper method for sleep (copied from your code)
    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }





    //x axis pid
    double kpx = 0.03;
    double kix = 0.0;
    double kdx = 0.003 ;
    double integralX = 0;
    double lastErrorX = 0;

    public void trackingAprilTagAuto(Telemetry telemetry,Gamepad gamepad2) {

        AprilTagDetection firstTagDetected;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (gamepad2.dpad_up)
            upDown.setPower(-0.15);
        else if (gamepad2.dpad_down)
            upDown.setPower(0.15);
        else
            upDown.setPower(0);


        if (!currentDetections.isEmpty()) {


            firstTagDetected = currentDetections.get(0);
            if (firstTagDetected != null && firstTagDetected.metadata != null) {


                // last and best attempt

                // left and right

                double bearing = firstTagDetected.ftcPose.bearing;

                integralX += bearing;
                double derivative = bearing - lastErrorX;

                double output = (kpx * bearing) + (kix * integralX) + (kdx * derivative);

                output = Math.max(-1, Math.min(1, output));

                rightLeft.setPower(-output);

                lastErrorX = bearing;


                //attempt 1 and 2

                /*double tagx = firstTagDetected.center.x;
                double tagy = firstTagDetected.center.y;


                //tracking april tag in x axis

                if (tagx > 410) {
                    servoPowerX = servoPowerX + 0.009   ;

                    if(tagx < 470 )
                        servoPower = (tagx - 410 ) / 320;
                    if (tagx < 530 && tagx > 470)
                        servoPower = ( tagx - 410 ) / 370;
                    if (tagx <590 && tagx > 530)
                        servoPower = ( tagx - 410 ) / 390;
                    if (tagx <640 && tagx > 590)
                        servoPower = ( tagx - 410 ) / 430;



                }
                else if (tagx < 230) {
                    servoPowerX = servoPowerX - 0.009;
                    if(tagx > 170 )
                        servoPower = (230 - tagx) / 320;
                    if (tagx > 110 && tagx < 170)
                        servoPower = (230 - tagx) / 370;
                    if (tagx >50 && tagx < 110)
                        servoPower = (230 - tagx) / 390;
                    if (tagx >0 && tagx < 50)
                        servoPower = (230 - tagx) / 430;


                } else if (tagx > 230 && tagx < 410)
                    rightLeft.setPower(0);

                //normalization x axis
                if (servoPowerX > 1)
                    servoPowerX = 0;
                else if (servoPowerX < -1)
                    servoPowerX = 0;
                rightLeft.setPower(servoPowerX);


                //tracking apriltag in y axis
                //80 px is our safe zone which the upDown servo isn't work and move
                //when pixel of y axis is more than 280 we must rotate our servo into safe zone
                if (tagy > 320){
                    servoPowerY = servoPowerY + 0.001;

                }
                //when pixel of y axis is less than 200 we must rotate our servo into safe zone
                else if (tagy < 120){
                    servoPowerY = servoPowerY - 0.001;

                } else if (tagy < 300 && tagy > 140)
                    upDown.setPower(0);


                //normalization y axis
                if (servoPowerY > 1)
                    servoPowerY = 0;
                else if (servoPowerY < -1)
                    servoPowerY = 0;

                upDown.setPower(servoPowerY);
*/

                // attempt 3

                /*if( tagx < 320+50 && tagx >320 ){
                    servoPower = 0;
                } else if (tagx >320 -50 && tagx <320) {
                    servoPower = 0;
                }
                if( tagx < 320+90 && tagx >320 ){
                    servoPower = 0.01;
                } else if (tagx >320 -90 && tagx <320) {
                    servoPower = -0.01;
                }
                if (tagx < 320 + 150 && tagx > 320){
                    servoPower = 0.03;
                } else if (tagx > 320 - 150 && tagx < 320) {
                    servoPower = -0.03;
                }
                if (tagx < 320 + 300 && tagx > 320){
                    servoPower = 0.05;
                } else if (tagx > 320 - 300 && tagx < 320) {
                    servoPower = -0.05;
                }


                if(tagbearing > 3 || tagbearing < -3){
                    servoPower =( mideRange - tagbearing * 0.5)/180;
                }else {
                    servoPower = mideRange / 180;
                }*/



                telemetry.addData("tag", "lucked");
                telemetry.addData("bearing", bearing);
                telemetry.addData("y", firstTagDetected.center.y);


            }
        }
        else {
            rightLeft.setPower(0);
        }


    }

    public void trackingAprilTagHandHeld(Gamepad gamepad2 , Telemetry telemetry){
        if (gamepad2.dpad_up)
            upDown.setPower(0.15);
        else if (gamepad2.dpad_down)
            upDown.setPower(-0.15);
        else if(gamepad2.dpad_left)
            rightLeft.setPower(0.4);
        else if(gamepad2.dpad_right)
            rightLeft.setPower(-0.4);
        else{
            rightLeft.setPower(0);
            upDown.setPower(0);
        }

        AprilTagDetection firstTagDetected;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();


        if (!currentDetections.isEmpty()) {
            firstTagDetected = currentDetections.get(0);
            if (firstTagDetected != null && firstTagDetected.metadata != null) {
                if(
                        (firstTagDetected.center.x <= 360 && firstTagDetected.center.x >= 280)&&
                        (firstTagDetected.center.y >= 200 && firstTagDetected.center.y <= 280)
                ){
                    telemetry.addData("april tag ","is in safe zone");
                    telemetry.addData("y axis",firstTagDetected.center.y);
                    telemetry.addData("x axis",firstTagDetected.center.x);
                }else {
                    telemetry.addData("y axis",firstTagDetected.center.y);
                    telemetry.addData("x axis",firstTagDetected.center.x);
                    telemetry.addData("april tag","must be in y=200 to 280, x=280 to 360");
                }
            }

        }else telemetry.addData("april tag","isn't detected");



    }

    double KpX = 0.03;
    double KpZ = 0.02;
    double KpRot = 0.02;



    public void driveToTag(RobotController drive, Telemetry telemetry) {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            double errorRange;

            double errorX = tag.ftcPose.x;
            if(tag.ftcPose.range>=100)
               errorRange= tag.ftcPose.range;
            else
                errorRange=0;

            double errorBearing = tag.ftcPose.bearing;

            double strafe = -KpX * errorX;
            double forward = -KpZ * errorRange;
            double rotate = -KpRot * errorBearing;

            strafe = Math.max(-1, Math.min(1, strafe));
            forward = Math.max(-1, Math.min(1, forward));
            rotate = Math.max(-1, Math.min(1, rotate));

            drive.autopilotDrive(forward, strafe, rotate);

            telemetry.addData("errorX", errorX);
            telemetry.addData("errorZ", errorRange);
            telemetry.addData("bearing", errorBearing);
            telemetry.addData("strafePower", strafe);
            telemetry.addData("forwardPower", forward);
            telemetry.addData("rotatePower", rotate);
        } else {
            drive.autopilotDrive(0, 0, 0);
            telemetry.addData("Status", "No Tag Detected");
        }

        telemetry.update();
    }



}

