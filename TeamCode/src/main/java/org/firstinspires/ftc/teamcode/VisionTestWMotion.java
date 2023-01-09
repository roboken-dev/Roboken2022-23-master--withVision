package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Signal Sleeve Test with motion")
public class VisionTestWMotion extends LinearOpMode {

    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    minibot robot = new minibot();
    
    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ",sleeveDetection.getPosition());
            telemetry.update();
        }

        waitForStart();


        switch (sleeveDetection.getPosition()) {

            case LEFT:
                telemetry.addData("movement: ", "I will go left");
                telemetry.update();
                sleep(3000);
                break;

            case RIGHT:
                telemetry.addData("movement: ", "I will go right");
                telemetry.update();
                sleep(3000);
                break;

            case CENTER:
                telemetry.addData("movement: ", "I will go to center");
                telemetry.update();
                sleep(3000);
                break;

        }

    }
}
