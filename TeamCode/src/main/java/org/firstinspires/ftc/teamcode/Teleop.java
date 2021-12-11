package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.GyroSensor;

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
        double servoPosition = 0;
        double encoderCount = 0;

        waitForStart();
        runtime.reset();

        //while (boxGyro.isCalibrating()) {
        //}

        while (opModeIsActive()) {

            outtakeMotor.setTargetPosition(150);

            currentHeading = boxGyro.getHeading();

            // drivePower is the power for forward/backward movement
            // rotatePower is the power for rotating the robot
            float drivePower = -gamepad1.left_stick_y;
            float rotatePower = gamepad1.right_stick_x;

            telemetry.addData("drivePower", String.valueOf(drivePower));
            telemetry.addData("rotatePower", String.valueOf(rotatePower));

            // Flip these signs if the robot rotates the wrong way
            frontLeft.setPower(drivePower + rotatePower);
            frontRight.setPower(drivePower - rotatePower);
            backLeft.setPower(drivePower + rotatePower);
            backRight.setPower(drivePower - rotatePower);

            if (gamepad2.a) {
                intakeMotor.setPower(0.75);
            }
                else if (gamepad2.b){
                    intakeMotor.setPower(-0.75);
            }
                    else {
                        intakeMotor.setPower(0);
            }

            if (gamepad2.y) {
                outtakeMotor.setPower(0.5);
            }
                else if (gamepad2.x){
                    outtakeMotor.setPower(-0.5);
            }
                    else {
                        outtakeMotor.setPower(0);
            }

            if (gamepad2.left_bumper) {
                capperMotor.setPower(0.1);
            }
                else if (gamepad2.right_bumper) {
                    capperMotor.setPower(-0.1);
            }
                    else {
                        capperMotor.setPower(0);
            }
            if (gamepad1.a){
                carouselMotor.setPower(0.75);
            }
                else if (gamepad1.b){
                    carouselMotor.setPower(-0.75);
            }
                    else{
                        carouselMotor.setPower(0);
            }

            if (gamepad2.dpad_up) {
                outtakeServo.setPosition(95);
            }

            if (gamepad2.dpad_down) {
                outtakeServo.setPosition(0);
            }

            if (gamepad2.dpad_left) {
                capperServo.setPosition(180);
            }

            if (gamepad2.dpad_right) {
                capperServo.setPosition(0);
            }

            if (gamepad2.left_bumper) {
                servoPosition = servoPosition + 45;
            }

            else if (gamepad2.right_bumper) {
                servoPosition = servoPosition - 45;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("BoxAngle", currentHeading);
            telemetry.update();

            //outtakeMotor.setPower(0);
            //intakeMotor.setPower(0);
            //carouselMotor.setPower(0);
            //capperMotor.setPower(0);
            idle();
        }
    }
}