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
            String Position = String.valueOf(sleeveDetection.getPosition());
            telemetry.addData("ROTATION: ", Position);
            telemetry.update();
        }

        waitForStart();
        sleep(500);
        String Position = String.valueOf(sleeveDetection.getPosition());
        if (Position == ": Right"){
            sleep(500);
            telemetry.addData("moving", "I will go to the right");
            telemetry.update();
            sleep(3000);
        }
        if (Position == ": Left"){
            sleep(500);
            telemetry.addData("moving", "I will go to the left");
            telemetry.update();
            sleep(3000);
        }
        if (Position == ": Center"){
            sleep(500);
            telemetry.addData("moving", "I will go to the center");
            telemetry.update();
            sleep(3000);
        }
    }
}
