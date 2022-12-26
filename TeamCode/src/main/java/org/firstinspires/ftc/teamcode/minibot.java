package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class minibot {

    public DcMotor motorFront;
    public DcMotor motorBack;
    public DcMotor motorLeft;
    public DcMotor motorRight;
    private ElapsedTime     runtime = new ElapsedTime();

    BNO055IMU imu;
    Orientation angles;

    static final double     SCALE_FACTOR = 75.0/75.0; //  if drive speed = .2 or .3 use 75.0/75.0;  .5 is 75.0/76.0 .4 is 75.0/75.5 if drive_speed = .1, use 1.0; if drive_speed = .3, use 75.0/77.0 note that .3 has hard time braking
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (SCALE_FACTOR * COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;

    public DcMotor arm;
    public Servo claw;
    public DcMotor spin;
    public DcMotor armspin;
    public Servo swipe;
    public CRServo blockspin;
    public DistanceSensor distanceSensor;
    public CRServo TESTING;
   // public ColorSensor sensorColor;
    //public DigitalChannel sensorTouch;


    HardwareMap hwMap = null;
    private ElapsedTime period =new ElapsedTime();

    public minibot(){}


    public void init(HardwareMap ahwMap, LinearOpMode opmode){

        hwMap=ahwMap;

        motorFront = hwMap.dcMotor.get("motorFront");
        motorBack = hwMap.dcMotor.get("motorBack");
        motorLeft = hwMap.dcMotor.get("motorLeft");
        motorRight =hwMap.dcMotor.get("motorRight");
        TESTING = hwMap.crservo.get("testing");

        arm = hwMap.dcMotor.get("arm");
        claw = hwMap.servo.get("claw");
        spin = hwMap.dcMotor.get("spin");
        armspin = hwMap.dcMotor.get("armspin");
        swipe = hwMap.servo.get("swipe");
        blockspin = hwMap.crservo.get("blockspin");
        distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");

        //sensorColor = hwMap.get(ColorSensor.class,"colorSensor");
        //sensorTouch = hwMap.get(DigitalChannel.class,"touchSensor");





        motorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armspin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



    }


/*
commented out methods need to be updated
 */

   /* public void encoderDrive(double speed,
                             double sideInches, double forwardInches,
                             double timeoutS, LinearOpMode opmode) throws InterruptedException {
        int newFrontTarget;
        int newLeftTarget;
        int newBackTarget;
        int newRightTarget;

        // Send telemetry message to signify robot waiting;
        opmode.telemetry.addData("Status", "Resetting Encoders");    //
        opmode.telemetry.update();

        initRunWithEncoder();

        // Send telemetry message to indicate successful Encoder reset
        opmode.telemetry.addData("Path0", "Starting at %7d :%7d",
                motorFront.getCurrentPosition(),
                motorBack.getCurrentPosition(),
                motorLeft.getCurrentPosition(),
                motorRight.getCurrentPosition());
        opmode.telemetry.update();


        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontTarget = (motorFront.getCurrentPosition() + (int) (sideInches * COUNTS_PER_INCH));
            newLeftTarget = (motorLeft.getCurrentPosition() + (int) (forwardInches * COUNTS_PER_INCH));
            newBackTarget = -(motorBack.getCurrentPosition() + (int) (sideInches * COUNTS_PER_INCH));
            newRightTarget = -(motorRight.getCurrentPosition() + (int) (forwardInches * COUNTS_PER_INCH));

            motorFront.setTargetPosition(newFrontTarget);
            motorBack.setTargetPosition(newBackTarget);
            motorLeft.setTargetPosition(newLeftTarget);
            motorRight.setTargetPosition(newRightTarget);

            opmode.telemetry.addData("Path1", "Running to %7d :%7d : %d : %d", newFrontTarget, newLeftTarget, newBackTarget, newRightTarget);
            opmode.telemetry.update();
            opmode.sleep(2000);

            // Turn On RUN_TO_POSITION
            motorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();

            motorFront.setPower(Math.abs(speed));
            motorBack.setPower(Math.abs(speed));
            motorLeft.setPower(Math.abs(speed));
            motorRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opmode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFront.isBusy() && motorLeft.isBusy() && motorBack.isBusy() && motorRight.isBusy())) {

                // Display it for the driver.
                opmode.telemetry.addData("Path1", "Running to %7d :%7d : %d : %d", newFrontTarget, newLeftTarget, newBackTarget, newRightTarget);
                opmode.telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d",
                        motorFront.getCurrentPosition(),
                        motorBack.getCurrentPosition(),
                        motorLeft.getCurrentPosition(),
                        motorRight.getCurrentPosition());
                opmode.telemetry.update();

            }

            // Stop all motion;
            motorFront.setPower(0);
            motorBack.setPower(0);
            motorLeft.setPower(0);
            motorRight.setPower(0);

            initRunWithoutEncoder();
            // Turn off RUN_TO_POSITION

        }
    }*/

   public void encoderSideDrive(double speed,
                            double sideInches,
                            double timeoutS, LinearOpMode opmode) throws InterruptedException {
       int newFrontTarget;
       int newBackTarget;

       // Send telemetry message to signify robot waiting;
       opmode.telemetry.addData("Status", "Resetting Encoders");    //
       opmode.telemetry.update();

       initRunWithEncoder();

       // Send telemetry message to indicate successful Encoder reset
       opmode.telemetry.addData("Path0", "Starting at %7d :%7d",
               motorFront.getCurrentPosition(),
               motorBack.getCurrentPosition(),
       opmode.telemetry.update());


       // Ensure that the opmode is still active
       if (opmode.opModeIsActive()) {

           // Determine new target position, and pass to motor controller
           newFrontTarget = (motorFront.getCurrentPosition() + (int) (sideInches * COUNTS_PER_INCH));
           newBackTarget = -(motorBack.getCurrentPosition() + (int) (sideInches * COUNTS_PER_INCH));

           motorFront.setTargetPosition(newFrontTarget);
           motorBack.setTargetPosition(newBackTarget);


           // Turn On RUN_TO_POSITION
           motorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           motorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           // reset the timeout time and start motion.
           runtime.reset();

           motorFront.setPower(Math.abs(speed));
           motorBack.setPower(-Math.abs(speed));

           // keep looping while we are still active, and there is time left, and both motors are running.
           // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
           // its target position, the motion will stop.  This is "safer" in the event that the robot will
           // always end the motion as soon as possible.
           // However, if you require that BOTH motors have finished their moves before the robot continues
           // onto the next step, use (isBusy() || isBusy()) in the loop test.
           while (opmode.opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (motorFront.isBusy() && motorBack.isBusy() )) {

               // Display it for the driver.
               opmode.telemetry.addData("Path1", "Running to %7d :%7d ", newFrontTarget, newBackTarget);
               opmode.telemetry.addData("Path2", "Running at %7d :%7d ",
                       motorFront.getCurrentPosition(),
                       motorBack.getCurrentPosition(),
               opmode.telemetry.update());

           }

           // Stop all motion;
           motorFront.setPower(0);
           motorBack.setPower(0);
           initRunWithoutEncoder();
           // Turn off RUN_TO_POSITION

       }
   }

    public void encoderForwardDrive(double speed,
                                 double forwardInches,
                                 double timeoutS, LinearOpMode opmode) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Send telemetry message to signify robot waiting;
        opmode.telemetry.addData("Status", "Resetting Encoders");    //
        opmode.telemetry.update();

        initRunWithEncoder();

        // Send telemetry message to indicate successful Encoder reset
        opmode.telemetry.addData("Path0", "Starting at %7d :%7d",
                motorLeft.getCurrentPosition(),
                motorRight.getCurrentPosition(),
                opmode.telemetry.update());


        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = (motorLeft.getCurrentPosition() + (int) (forwardInches * COUNTS_PER_INCH));
            newRightTarget = (motorRight.getCurrentPosition() + (int) (forwardInches * COUNTS_PER_INCH));

            motorLeft.setTargetPosition(newLeftTarget);
            motorRight.setTargetPosition(newRightTarget);


            // Turn On RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();

            motorLeft.setPower(Math.abs(speed));
            motorRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opmode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLeft.isBusy() && motorRight.isBusy() )) {

                // Display it for the driver.
                opmode.telemetry.addData("Path1", "Running to %7d :%7d ", newLeftTarget, newRightTarget);
                opmode.telemetry.addData("Path2", "Running at %7d :%7d ",
                        motorLeft.getCurrentPosition(),
                        motorRight.getCurrentPosition(),
                        opmode.telemetry.update());

            }

            // Stop all motion;
            motorLeft.setPower(0);
            motorRight.setPower(0);
            initRunWithoutEncoder();
            // Turn off RUN_TO_POSITION

        }
    }


        public void initRunWithEncoder()
        {
            motorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            motorFront.setDirection(DcMotor.Direction.REVERSE);
            motorLeft.setDirection(DcMotor.Direction.REVERSE);
            motorBack.setDirection(DcMotor.Direction.REVERSE);
            motorRight.setDirection(DcMotor.Direction.FORWARD);

            motorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeft.setDirection(DcMotor.Direction.REVERSE);
            motorRight.setDirection(DcMotor.Direction.FORWARD);
        }

        public void initRunWithoutEncoder ()
        {
            motorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motorFront.setDirection(DcMotor.Direction.REVERSE);
            motorLeft.setDirection(DcMotor.Direction.REVERSE);
            motorBack.setDirection(DcMotor.Direction.FORWARD);
            motorRight.setDirection(DcMotor.Direction.FORWARD);
        }

    public void driveForwardTime(double speed,long time) throws InterruptedException {
        motorLeft.setPower(speed);
        motorRight.setPower(speed);
        Thread.sleep(time);
        stopDriving();
    }



    public void driveForward(double speed)  {
        motorLeft.setPower(speed);
        motorRight.setPower(-speed);
    }

    public void driveLeft(double speed){
        motorFront.setPower(speed);
        motorBack.setPower(speed);
    }


/*
    public void driveUntilTouch(double speed, LinearOpMode opmode) throws InterruptedException
    {
        driveForward(speed);
        while (sensorTouch.getState()==true&&!opmode.isStopRequested())
        {
            if (sensorTouch.getState() == true) {
                opmode.telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                opmode.telemetry.addData("Digital Touch", "Is Pressed");
            }
            opmode.telemetry.update();
        }
        stopDriving();
    }

*/


/*
    public void driveUntilTouch(double speed, LinearOpMode opmode) throws InterruptedException
    {
        driveForward(speed);
        while (sensorTouch.getState()==true&&!opmode.isStopRequested())
        {
            if (sensorTouch.getState() == true) {
                opmode.telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                opmode.telemetry.addData("Digital Touch", "Is Pressed");
            }
            opmode.telemetry.update();
        }
        stopDriving();
    }

*/

    /* public void driveUntilColor(double speed, double colorValue, LinearOpMode opmode) throws InterruptedException {
        float hsvValues[]={300F,300F,300F};
        Color.RGBToHSV(sensorColor.red()*255, sensorColor.green()*255,sensorColor.blue()*255,hsvValues);

        driveForward(speed);
//&&hsvValues[0]>(colorValue-5.0)
        while (hsvValues[0]<(colorValue)&&!opmode.isStopRequested())
        {
            Color.RGBToHSV(sensorColor.red()*255, sensorColor.green()*255,sensorColor.blue()*255,hsvValues);
        }


        stopDriving();
    }
*/


    public void stopDriving()
    {
        motorFront.setPower(0);
        motorRight.setPower(0);
        motorLeft.setPower(0);
        motorBack.setPower(0);
    }
/*

    public void turnLeftTime(double speed,long time) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        rearLeft.setPower(-speed);
        rearRight.setPower(speed);
        Thread.sleep(time);
        stopDriving();


    }
*/

/*
    public void turnLeftAngle(double speed,int angleReading, LinearOpMode opmode) throws InterruptedException {

        angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Thread.sleep(500);
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        rearLeft.setPower(-speed);
        rearRight.setPower(-speed);
        while (angles.firstAngle < angleReading && !opmode.isStopRequested()){
            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            opmode.telemetry.addData("Heading",angles.firstAngle);
            opmode.telemetry.update();
        }
        stopDriving();
    }
*/

/*
    public void turnRightAngle(double speed,int angleReading, LinearOpMode opmode) throws InterruptedException {

        angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Thread.sleep(500);
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        rearLeft.setPower(speed);
        rearRight.setPower(speed);
        while (angles.firstAngle > angleReading && !opmode.isStopRequested()){
            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            opmode.telemetry.addData("Heading",angles.firstAngle);
            opmode.telemetry.update();
        }
        stopDriving();
    }
*/


}
