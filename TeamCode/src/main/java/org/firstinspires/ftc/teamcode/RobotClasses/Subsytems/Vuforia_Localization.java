package org.firstinspires.ftc.teamcode.RobotClasses.Subsytems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

//This OpMode was written for the VuforiaDemo Basics video. This demonstrates basic principles of
//using VuforiaDemo in FTC.

public class

Vuforia_Localization {
    // Variables to be used for later
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target;
    private VuforiaTrackableDefaultListener listener;

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

    Standard_Bot robot = new Standard_Bot();   // Use a StandardBot's hardware

    public void VuforiaOrient (double targetX, double targetY, double targetAngle) {
        double correctionX = 0;
        double correctionY = 0;
        double correctionAngle = 0;
        double gainY = 0;
        double speedY = 0;
        double gainX = 0;
        double speedX = 0;
        double gainAngle = 0;
        double speedAngle = 0;
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
        visionTargets.activate();
        OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

        if(latestLocation != null)
            lastKnownLocation = latestLocation;

        float[] coordinates = lastKnownLocation.getTranslation().getData();

        robotX = coordinates[0];
        robotY = coordinates[1];
        robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        if (Math.abs(targetY - robotY)>10 && listener.isVisible())
            gainY = Math.abs((targetY-robotY)/targetY);
        gainY = gainY + 0.75;
        speedY = Math.abs((targetY-robotY)/targetY)*0.15;
        correctionY = (targetY - robotY)*1.25;
        correctionY = correctionY/25.4;
        TankDrive.drive(-correctionY, speedY); //Correction Y needs to be negative because the camera is mounted off the back and the y-axis of the robot is opposite of that of the picture's
        gainY = 0;
        speedY = 0;
        correctionY = 0;

        if (Math.abs(targetX - robotX)>10 && listener.isVisible())
            gainX = Math.abs((targetX-robotX)/targetX);
        gainX = gainX + 1;
        speedX = Math.abs((targetX-robotX)/targetX)*1.25;
        correctionX = ((targetX - robotX)*1.25);
        correctionX = (correctionX/25.4);
        if (correctionX > 0) {
            TankDrive.turn(-90, speedX);
            TankDrive.drive(correctionX, speedX);
            TankDrive.turn(90, speedX);
        }
        else if(correctionX < 0) {
            TankDrive.turn(90, speedY);
            TankDrive.drive(correctionX, speedX);
            TankDrive.turn(-90, speedY);
        }
        gainX = 0;
        speedX = 0;
        correctionX = 0;

        if (Math.abs(targetAngle - robotAngle)>10 && listener.isVisible())
            gainAngle = Math.abs((targetAngle-robotAngle)/targetAngle);
        gainAngle = gainAngle + 1;
        speedAngle = Math.abs((targetAngle-robotAngle)/targetAngle)*1.25;
        correctionAngle = ((targetAngle - robotAngle)*1.25);
        TankDrive.turn(correctionAngle, speedAngle);
        gainAngle = 0;
        speedAngle = 0;
        correctionAngle = 0;
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
                multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
}
