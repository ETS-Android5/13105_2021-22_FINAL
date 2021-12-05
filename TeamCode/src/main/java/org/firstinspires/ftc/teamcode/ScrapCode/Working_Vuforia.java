package org.firstinspires.ftc.teamcode.ScrapCode;/*
package org.firstinspires.ftc.teamcode.ScrapCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Subsytems.Standard_Bot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

//This OpMode was written for the VuforiaDemo Basics video. This demonstrates basic principles of
//using VuforiaDemo in FTC.

@Autonomous(name = "Vuforia_Tracking_Working")
public class Working_Vuforia extends LinearOpMode
{
    // Variables to be used for later
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target;
    private VuforiaTrackableDefaultListener listener;

    private double targetX = 1;
    private double targetY = -533;
    private double targetAngle = 1;
    private double correctionX = 0;
    private double correctionY = 0;
    private double correctionAngle = 0;
    private double gainY = 0;
    private double speedY = 0;
    private double gainX = 0;
    private double speedX = 0;
    private double gainAngle = 0;
    private double speedAngle = 0;

    public void encoderDrive( double power, double leftInches,
                              double rightInches, int timeout)
    {
        int newLeftTarget;
        int newRightTarget;
        double directionCorrection=0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFront.getCurrentPosition() + (int)(leftInches * 33);
            newRightTarget = rightFront.getCurrentPosition() + (int)(rightInches * 33);

            leftFront.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(newRightTarget);
            leftRear.setTargetPosition(newLeftTarget);
            rightRear.setTargetPosition(newRightTarget);

            runtime.reset();                           // reset the timeout time and start motion.
            leftFront.setPower(0.05);
            leftRear.setPower(0.05);
            rightFront.setPower(0.05);
            rightRear.setPower(0.05);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setVelocity(2400*power);
            leftRear.setVelocity(2400*power);
            rightFront.setVelocity(2400*power);
            rightRear.setVelocity(2400*power);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (leftFront.isBusy() && rightFront.isBusy()))
            {
                // Use the gyro if you are driving straight
                if (Math.abs(leftInches - rightInches) < 0.1) {
                    directionCorrection = getCorrection(leftInches < 0.0);
                    leftFront.setVelocity (2400*(power + directionCorrection));
                    leftRear.setVelocity (2400*(power + directionCorrection));
                    rightFront.setVelocity (2400*(power - directionCorrection));
                    rightRear.setVelocity (2400*(power - directionCorrection));
                }

                // Display it for the driver.
                //telemetry.addData("leftFront", leftFront.getVelocity());
                //telemetry.addData("rightFront", rightFront.getVelocity());
                //telemetry.addData("leftRear", leftRear.getVelocity());
                //telemetry.addData("rightRear", rightRear.getVelocity());
                //telemetry.addData("leftFront", leftFront.getPower());
                //telemetry.addData("rightFront", rightFront.getPower());
                //telemetry.addData("leftRear", leftRear.getPower());
                //telemetry.addData("rightRear", rightRear.getPower());
                //telemetry.update();
            }
            rightFront.setPower(0);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
            leftFront.setVelocity(0);
            leftRear.setVelocity(0);
            rightFront.setVelocity(0);
            rightRear.setVelocity(0);
        }
    }

    public void strafeDrive( double power, double frontInches,
                             double rearInches, int timeout)
    {
        int newLeftFrontTarget;
        int newLeftRearTarget;
        int newRightFrontTarget;
        int newRightRearTarget;
        double directionCorrection=0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() - (int)(frontInches * 50);
            newLeftRearTarget =  leftRear.getCurrentPosition() + (int)(frontInches * 50);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(frontInches * 50);
            newRightRearTarget = rightRear.getCurrentPosition() - (int)(frontInches * 50);

            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftRear.setTargetPosition(newLeftRearTarget);
            rightRear.setTargetPosition(newRightRearTarget);

            runtime.reset();                           // reset the timeout time and start motion.
            leftFront.setPower(0.05);
            leftRear.setPower(0.05);
            rightFront.setPower(0.05);
            rightRear.setPower(0.05);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setVelocity(2400*power);
            leftRear.setVelocity(2400*power);
            rightFront.setVelocity(2400*power);
            rightRear.setVelocity(2400*power);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (leftFront.isBusy() && rightFront.isBusy()))
            {
                // Use the gyro if you are driving straight
          if (Math.abs(leftInches - rightInches) < 0.1) {
            directionCorrection = getCorrection(leftInches < 0.0);
            leftFront.setVelocity (2400*(power + directionCorrection));
            leftRear.setVelocity (2400*(power + directionCorrection));
            rightFront.setVelocity (2400*(power - directionCorrection));
            rightRear.setVelocity (2400*(power - directionCorrection));
}
            }
            rightFront.setPower(0);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
            leftFront.setVelocity(0);
            leftRear.setVelocity(0);
            rightFront.setVelocity(0);
            rightRear.setVelocity(0);
        }
    }

    private double getCorrection(boolean reverse)
    {
        double correction, angle;
        angle = getAngle();
        if (angle == 0)
            correction = 0;             // no adjustment.
        else if (reverse)
            correction = - angle;        // reverse sign of angle for correction.
        else
            correction = angle;
        correction *= GAIN;
        return correction;
    }

    private double getAngle()
    {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void setupVuforia()
    {
        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        // Setup the target to be tracked
        target = visionTargets.get(0); // 0 corresponds to the wheels target
        target.setName("Wheels Target");
        target.setLocation(createMatrix(0, 0, 0, 90, 0, 0));

        // Set phone location on robot
        phoneLocation = createMatrix(0, -229, 0, -90, 0, 0);

        // Setup listener and inform it of phone information
        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    private String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }

    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    private static final String VUFORIA_KEY = "Acy/Mw7/////AAABmVeoJmayA0LFsPI//25wXiAgKvF8A1IMKKPVnU/YmD2SLqIfVxja/iMw9FGXlbh6ipPCe1BFslVFQra+jdueadfzzLqYiH9I9BAz0gOjuhBQB3bxgRnHI4nFWwaaRd0BYN0MYlgXZCcLjL5YdP/dnk3ffrlMlf4U5IK/yJpsxw6Eum84uoiFq7gnyOAcdJR2zXpF86/e/L0iPQrloOtqspc5Pp4u1ra2e6Sa3fWhHbB2g4wC4nYgi980JBGxZdvL1tkVoMqmbvrylRwF4Jsm7NsDKLjkZRnGULl/xXSFJ9ry45RAEGxh745HQRiJ2/lNpdabZpXONPi5xMscczlGvUpgCNebeumlqXrA7swmEsGE";
    private double robotX = 0;
    private double robotY = 0;
    private float robotAngle = 0;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;
    final double            GAIN = 0.05;
    private ElapsedTime     runtime = new ElapsedTime();

    Standard_Bot robot   = new Standard_Bot();   // Use a StandardBot's hardware

    DcMotorImplEx leftRear, rightRear, leftFront, rightFront;

    public void runOpMode() throws InterruptedException
    {

        robot.init(hardwareMap);
//        leftFront = (DcMotorImplEx)robot.StdLeftFront;
//        leftRear = (DcMotorImplEx)robot.StdLeftRear;
//        rightFront = (DcMotorImplEx)robot.StdRightFront;
//        rightRear = (DcMotorImplEx)robot.StdRightRear;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        setupVuforia();

        // We don't know where the robot is, so set it to the origin
        // If we don't include this, it would be null, which would cause errors later on
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        waitForStart();

        // Start tracking the targets
        visionTargets.activate();

        while(opModeIsActive())
        {
            // Ask the listener for the latest information on where the robot is
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            // The listener will sometimes return null, so we check for that to prevent errors
            if(latestLocation != null)
                lastKnownLocation = latestLocation;

            float[] coordinates = lastKnownLocation.getTranslation().getData();

            robotX = coordinates[0];
            robotY = coordinates[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            // Send information about whether the target is visible, and where the robot is
            telemetry.addData("Tracking " + target.getName(), listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));
            telemetry.addData("robotX", robotX);
            telemetry.addData("robotY", robotY);

            if (Math.abs(targetY - robotY)>10 && listener.isVisible())
                gainY = Math.abs((targetY-robotY)/targetY);
                gainY = gainY + 0.75;
                speedY = Math.abs((targetY-robotY)/targetY)*0.15;
                correctionY = (targetY - robotY)*1.25;
                correctionY = correctionY/25.4;
                encoderDrive(speedY, -correctionY, -correctionY, 5); //Correction Y needs to be negative because the camera is mounted off the back andthe y-axis of the robot is opposite of that of the picture's
                telemetry.addData("gainY", gainY);
                telemetry.addData("speedY", speedY);
                telemetry.addData("correctionY", correctionY);
                gainY = 0;
                speedY = 0;
                correctionY = 0;

                sleep(300);

            if (Math.abs(targetX - robotX)>10 && listener.isVisible())
                gainX = Math.abs((targetX-robotX)/targetX);
                gainX = gainX + 1;
                speedX = Math.abs((targetX-robotX)/targetX)*1.25;
                correctionX = ((targetX - robotX)*1.25);
                correctionX = (correctionX/25.4);
                strafeDrive(-0.2, correctionX, correctionX, 5);
                telemetry.addData("gainX", gainX);
                telemetry.addData("speedX", speedX);
                telemetry.addData("correctionX", correctionX);
                gainX = 0;
                speedX = 0;
                correctionX = 0;

            if (Math.abs(targetAngle - robotAngle)>10 && listener.isVisible())
                gainAngle = Math.abs((targetAngle-robotAngle)/targetAngle);
                gainAngle = gainAngle + 1;
                speedAngle = Math.abs((targetAngle-robotAngle)/targetAngle)*1.25;
                correctionAngle = ((targetAngle - robotAngle)*1.25);
                correctionAngle = (correctionAngle/25.4);
                strafeDrive(-0.2, correctionAngle, correctionAngle, 5);
                telemetry.addData("gainAngle", gainAngle);
                telemetry.addData("speedAngle", speedAngle);
                telemetry.addData("correctionAngle", correctionAngle);
                gainAngle = 0;
                speedAngle = 0;
                correctionAngle = 0;

            // Send telemetry and idle to let hardware catch up
            telemetry.update();
            idle();
        }
        rightFront.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}
 */