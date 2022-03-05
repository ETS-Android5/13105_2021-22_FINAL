package org.firstinspires.ftc.teamcode.ScrapCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsytems.Standard_Bot;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsytems.TankDrive;

import java.util.List;

@Autonomous(name="Another_Test_Auto", group="Another_Test_Auto")
public class Another_Test_Auto extends LinearOpMode {

    Standard_Bot robot = new Standard_Bot();
    TankDrive drivetrain = new TankDrive();

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorImplEx frontLeft = null;
    private DcMotorImplEx backLeft = null;
    private DcMotorImplEx frontRight = null;
    private DcMotorImplEx backRight = null;
    private DcMotor intakeMotor = null;
    private DcMotor outtakeMotor = null;
    private DcMotor carouselMotor = null;
    private DcMotor capperMotor = null;

    private Servo outtakeServo = null;
    private Servo capperServo = null;

    String object = null, targetObject = "0 Blue Left";
    double currentLeft = 0, currentRight = 0, currentTop = 0, currentBottom = 0;
    double targetTop = 1;
    double targetBottom = 0;
    double targetLeft = -250;
    double targetRight = 0;
    double distance = 0;
    double angle = 0;
    int i = 0;

   // private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String TFOD_MODEL_ASSET = "model_20220305_105857.tflite";

    private static final String[] LABELS = {
            "BlueHub",
            "RedHub"
    };

    private static final String VUFORIA_KEY =
            "Acy/Mw7/////AAABmVeoJmayA0LFsPI//25wXiAgKvF8A1IMKKPVnU/YmD2SLqIfVxja/iMw9FGXlbh6ipPCe1BFslVFQra+jdueadfzzLqYiH9I9BAz0gOjuhBQB3bxgRnHI4nFWwaaRd0BYN0MYlgXZCcLjL5YdP/dnk3ffrlMlf4U5IK/yJpsxw6Eum84uoiFq7gnyOAcdJR2zXpF86/e/L0iPQrloOtqspc5Pp4u1ra2e6Sa3fWhHbB2g4wC4nYgi980JBGxZdvL1tkVoMqmbvrylRwF4Jsm7NsDKLjkZRnGULl/xXSFJ9ry45RAEGxh745HQRiJ2/lNpdabZpXONPi5xMscczlGvUpgCNebeumlqXrA7swmEsGE";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    DistanceSensor sensorRange;
    DistanceSensor backStop;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    HardwareMap hwMap = null;


    @Override
    public void runOpMode() {

        telemetry.addData("Ready", "");

        telemetry.update();
        robot.init(hardwareMap);

        frontLeft = robot.StdFrontLeft;
        frontRight = robot.StdFrontRight;
        backLeft = robot.StdBackLeft;
        backRight = robot.StdBackRight;
        intakeMotor = robot.StdIntakeMotor;
        outtakeMotor = robot.StdOuttakeMotor;
        carouselMotor = robot.StdCarouselMotor;
        capperMotor = robot.StdCapperMotor;
        outtakeMotor = robot.StdOuttakeMotor;
        outtakeServo = robot.StdOuttakeServo;
        capperServo = robot.StdCapperServo;

        sensorRange = robot.StdDistanceSensor;
        backStop = robot.StdBackStop;

        //outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        double allianceHubLevel = 0;
        double angleToTeamElement = 0;

        telemetry.addData("ready", "");

        frontLeft.setVelocity(0, AngleUnit.DEGREES);
        frontRight.setVelocity(0, AngleUnit.DEGREES);
        backLeft.setVelocity(0, AngleUnit.DEGREES);
        backRight.setVelocity(0, AngleUnit.DEGREES);
        intakeMotor.setPower(0);
        outtakeMotor.setPower(0);
        capperMotor.setPower(0);
        carouselMotor.setPower(0);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }

        waitForStart();

        while (opModeIsActive()) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    object = recognition.getLabel();
                    currentLeft = recognition.getLeft();
                    currentRight = recognition.getRight();
                    currentTop = recognition.getTop();
                    currentBottom = recognition.getBottom();

                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    telemetry.update();
                    sleep(250);

                    //if (currentTop != targetTop) {
                    //    drive(targetTop - currentTop, targetTop - currentTop, 360);
                    //    sleep(250);
                    //}
                }
            }
        }
    }

    public void drive(double right, double left, double anglrt) {

        int rightTarget;
        int leftTarget;

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftTarget = (int) (left * (33) + frontLeft.getCurrentPosition());
        rightTarget = (int) (right * (33) + frontRight.getCurrentPosition());

        frontLeft.setTargetPosition(leftTarget);
        backLeft.setTargetPosition(leftTarget);
        frontRight.setTargetPosition(rightTarget);
        backRight.setTargetPosition(rightTarget);

        frontLeft.setVelocity(90, AngleUnit.DEGREES);
        backLeft.setVelocity(90, AngleUnit.DEGREES);
        frontRight.setVelocity(90, AngleUnit.DEGREES);
        backRight.setVelocity(90, AngleUnit.DEGREES);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setVelocity(anglrt, AngleUnit.DEGREES);
        backLeft.setVelocity(anglrt, AngleUnit.DEGREES);
        frontRight.setVelocity(anglrt, AngleUnit.DEGREES);
        backRight.setVelocity(anglrt, AngleUnit.DEGREES);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            if (backStop.getDistance(DistanceUnit.INCH) < 6 ){
                frontLeft.setVelocity(0);
                frontRight.setVelocity(0);
                backLeft.setVelocity(0);
                backRight.setVelocity(0);
                break;
            }
            idle();
        }

    /*  frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); */

        frontLeft.setVelocity(0, AngleUnit.DEGREES);
        frontRight.setVelocity(0, AngleUnit.DEGREES);
        backLeft.setVelocity(0, AngleUnit.DEGREES);
        backRight.setVelocity(0, AngleUnit.DEGREES);

    }

    public double getAngle() {
        imu.getPosition();
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return lastAngles.firstAngle;
    }

    private void rotate(double degrees, double anglrt) {
        double temp = rotate(degrees, anglrt,0);
        return;
    }

    private double rotate(double degrees, double anglrt, int dummy) {
        double leftPower, rightPower;
        double currentAngle = 0, currentDistance = 0, minAngle = 0, minDistance = 100;

        // restart imu movement tracking.

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = anglrt;
            rightPower = -anglrt;
        } else if (degrees > 0) {
            // turn left.
            leftPower = -anglrt;
            rightPower = anglrt;
        } else return 0;

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power to rotate.
        frontLeft.setVelocity(leftPower, AngleUnit.DEGREES);
        backLeft.setVelocity(leftPower, AngleUnit.DEGREES);
        frontRight.setVelocity(rightPower, AngleUnit.DEGREES);
        backRight.setVelocity(rightPower, AngleUnit.DEGREES);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && (currentAngle = getAngle()) > degrees) {
                currentDistance = sensorRange.getDistance(DistanceUnit.INCH);
                if (currentDistance < minDistance) {
                    minDistance = currentDistance;
                    minAngle = currentAngle;
                }

            }
        } else {    // left turn.
            while (opModeIsActive() && getAngle() == 0) {
            }
            while (opModeIsActive() && (currentAngle = getAngle()) < degrees) {
                currentDistance = sensorRange.getDistance(DistanceUnit.INCH);
                if (currentDistance < minDistance) {
                    minDistance = currentDistance;
                    minAngle = currentAngle;
                }
            }
        }
        // turn the motors off.
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // wait for rotation to stop.
        sleep(100);

        telemetry.addData("Min Dist: " + minDistance, "; Min Angle: " + minAngle);
        telemetry.update();
        // reset angle tracking on new heading.
        return minAngle;
    }

    public void outtakeArmDrive(double power, double armInches, int timeout) {
        int newTarget;

        if (opModeIsActive()) {

            newTarget = outtakeMotor.getCurrentPosition() + (int) (armInches);
            outtakeMotor.setTargetPosition(newTarget);
            outtakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            outtakeMotor.setPower(power);

            while (opModeIsActive() && (runtime.seconds() < timeout) && outtakeMotor.isBusy()) {
                telemetry.addData("PathIA", "Running to %7d :%7d", newTarget, outtakeMotor.getCurrentPosition());
                telemetry.update();
                idle();
                outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void threeDump() {
        outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeServo.setPosition(0.15);
        sleep(250);
        outtakeMotor.setPower(-0.5);
        sleep(500);
        outtakeServo.setPosition(0);
        sleep(250);
        outtakeMotor.setPower(-0.5);
        sleep(500);
        outtakeMotor.setPower(0.5);//down
        sleep(500);
        outtakeServo.setPosition(0.15);
        sleep(100);
        outtakeMotor.setPower(0.5);
        sleep(400);
        outtakeMotor.setPower(0);
        outtakeServo.setPosition(0);
    }

    public void twoDump() {
        outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeServo.setPosition(0.15);
        sleep(250);
        outtakeMotor.setPower(0.5);
        sleep(500);
        outtakeMotor.setPower(0);
        outtakeServo.setPosition(0.5);
        sleep(500);
        outtakeServo.setPosition(0.15);//down
        sleep(250);
        outtakeMotor.setPower(-0.5);
        sleep(500);
        outtakeMotor.setPower(0);
        outtakeServo.setPosition(0);
        sleep(250);
    }

    public void oneDump() {
        outtakeServo.setPosition(0.55);
        sleep(500);
        outtakeServo.setPosition(0);
        sleep(250);
        outtakeServo.setPosition(0);
        sleep(250);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

}