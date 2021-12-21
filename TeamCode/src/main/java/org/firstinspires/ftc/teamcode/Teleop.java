package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsytems.Standard_Bot;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsytems.TankDrive;

@TeleOp (name="Teleop", group="Teleop")
public class Teleop extends LinearOpMode {

    TankDrive tankDrive = new TankDrive();
    Standard_Bot robot = new Standard_Bot();

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor intakeMotor = null;
    private DcMotor outtakeMotor = null;
    private DcMotor carouselMotor = null;
    private DcMotor capperMotor = null;

    private Servo outtakeServo = null;
    private Servo capperServo = null;

    private GyroSensor boxGyro = null;

    HardwareMap hwMap = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
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

        capperServo = robot.StdCapperServo;
        outtakeServo = robot.StdOuttakeServo;

        boxGyro = robot.StdBoxGyro;


        //GyroSensor sensorGyro;
        //ModernRoboticsI2cGyro mrGyro;

        //sensorGyro = hardwareMap.gyroSensor.get("boxgyro");
        //boxGyro = (ModernRoboticsI2cGyro) sensorGyro;
        //boxGyro.calibrate();

        double currentHeading = 0;
        double servoPosition = 100;
        double encoderCount = 0;

        waitForStart();
        runtime.reset();

        //while (boxGyro.isCalibrating()) {
        //}

        while (opModeIsActive()) {

            //outtakeServo.setPosition(90);

            currentHeading = boxGyro.getHeading();

            // drivePower is the power for forward/backward movement
            // rotatePower is the power for rotating the robot
            float drivePower = -gamepad1.left_stick_y;
            float rotatePower = gamepad1.right_stick_x;

            telemetry.addData("drivePower", String.valueOf(drivePower));
            telemetry.addData("rotatePower", String.valueOf(rotatePower));

            //drivePower = (float) (drivePower*(0.1));
            //rotatePower = (float) (rotatePower*(0.1));

            // Flip these signs if the robot rotates the wrong way
            frontLeft.setPower(drivePower + rotatePower);
            frontRight.setPower(drivePower - rotatePower);
            backLeft.setPower(drivePower + rotatePower);
            backRight.setPower(drivePower - rotatePower);

            encoderCount = outtakeMotor.getCurrentPosition();

            while (gamepad1.dpad_up){
                frontLeft.setPower(0.25);
                frontRight.setPower(0.25);
                backLeft.setPower(0.25);
                backRight.setPower(0.25);
            }
            while (gamepad1.dpad_down){
                frontLeft.setPower(-0.25);
                frontRight.setPower(-0.25);
                backLeft.setPower(-0.25);
                backRight.setPower(-0.25);
            }
            while (gamepad1.dpad_right){
                frontLeft.setPower(0.25);
                frontRight.setPower(0.25);
                backLeft.setPower(-0.25);
                backRight.setPower(-0.25);
            }
            while (gamepad1.dpad_left){
                frontLeft.setPower(-0.25);
                frontRight.setPower(-0.25);
                backLeft.setPower(0.25);
                backRight.setPower(0.25);
            }
            if (gamepad1.a) {
                intakeMotor.setPower(1);
            }
                else if (gamepad1.b){
                    intakeMotor.setPower(-1);
            }
                    else {
                        intakeMotor.setPower(0);
            }

            if (gamepad2.y) {
                outtakeMotor.setPower(0.25);
            }
                else if (gamepad2.x){
                    outtakeMotor.setPower(-0.25);
            }
                    else {
                        outtakeMotor.setPower(0);
            }

            if (gamepad2.left_bumper) {
                capperMotor.setPower(0.4);
            }
                else if (gamepad2.right_bumper) {
                    capperMotor.setPower(-0.4);
            }
                    else {
                        capperMotor.setPower(0);
            }

            if (gamepad2.a){
                carouselMotor.setPower(0.75);
            }
                else if (gamepad2.b){
                    carouselMotor.setPower(-0.75);
            }
                    else{
                        carouselMotor.setPower(0);
            }

            if (gamepad2.dpad_up) {
                outtakeServo.setPosition(1);
            }

            if (gamepad2.dpad_down) {
                outtakeServo.setPosition(0.6);
            }

            if (gamepad2.dpad_left) {
                capperServo.setPosition(0);
            }

            if (gamepad2.dpad_right) {
                capperServo.setPosition(0.5);
            }

            if (gamepad1.left_bumper) {
                //servoPosition = servoPosition + 10;
                //sleep(500);
                //outtakeServo.setPosition(servoPosition);
            }
            if (gamepad1.dpad_right){
                //servoPosition = servoPosition - 10;
                //sleep(500);
                //outtakeServo.setPosition(servoPosition);
                //sleep(250);
                //outtakeMotor.setTargetPosition(130);
                //outtakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //sleep(250);
                //outtakeServo.setPosition(80);
                //sleep(250);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("BoxAngle", currentHeading);
            telemetry.update();

            //outtakeMotor.setPower(0);
            //intakeMotor.setPower(0);
            //carouselMotor.setPower(0);
            //capperMotor.setPower(0);
            telemetry.addData("servoPosition", String.valueOf(servoPosition));

            idle();
        }
    }
}