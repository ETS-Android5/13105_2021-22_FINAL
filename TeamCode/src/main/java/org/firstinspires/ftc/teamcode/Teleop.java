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

    HardwareMap hwMap =  null;

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
        boxGyro.calibrate();


        double xValue = 0;
        double yValue = 0;
        double zValue = 0;
        int motorTicks = 0;
        double currentHeading = 0;
        double servoPosition = 0;
        outtakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        runtime.reset();

        while (boxGyro.isCalibrating()) {
        }

        while (opModeIsActive()) {

            motorTicks = outtakeMotor.getCurrentPosition();
            telemetry.addData("motorTicks", String.valueOf(motorTicks));
            outtakeMotor.setTargetPosition(28);

            currentHeading = boxGyro.getHeading();
            xValue = boxGyro.rawX();
            yValue = boxGyro.rawY();
            zValue = boxGyro.rawZ();

            outtakeMotor.setPower(0.5);

            intakeMotor.setPower(0);
            outtakeMotor.setPower(0);
            carouselMotor.setPower(0);
            capperMotor.setPower(0);

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

            if (currentHeading != 0){
                servoPosition = currentHeading;
                telemetry.addData("servoPosition", String.valueOf(servoPosition));
                outtakeServo.setPosition(10);
            }
            if (gamepad2.a){
                carouselMotor.setPower(0.5);
            }
                else if (gamepad2.a = false){
                    carouselMotor.setPower(0);
            }
            if (gamepad2.b){
                carouselMotor.setPower(-0.5);
            }
                else if (gamepad2.b = false){
                    carouselMotor.setPower(0);
            }
            if (gamepad2.y){
                outtakeMotor.setPower(0.5);
            }
                else if (gamepad2.y = false){
                    outtakeMotor.setPower(0);
            }
            if (gamepad2.x){
                outtakeMotor.setPower(-0.5);
            }
                else if (gamepad2.x = false){
                    outtakeMotor.setPower(0);
            }
            if (gamepad2.left_bumper){
                capperMotor.setPower(0.25);
            }
                else if (gamepad2.left_bumper = false){
                    capperMotor.setPower(0);
            }
            if (gamepad2.right_bumper){
                capperMotor.setPower(-0.25);
            }
                else if (gamepad2.right_bumper = false){
                    capperMotor.setPower(0);
            }
            if (gamepad1.a){
                intakeMotor.setPower(0.5);
            }
                else if (gamepad1.a = false){
                    intakeMotor.setPower(0);
            }
            if (gamepad1.b) {
                intakeMotor.setPower(-0.5);
            }
                else if (gamepad1.b = false){
                    intakeMotor.setPower(0);
                }
            if (gamepad2.dpad_up){
                outtakeServo.setPosition(95);
            }
            if (gamepad2.dpad_down){
                outtakeServo.setPosition(0);
            }
            if (gamepad2.dpad_left){
                capperServo.setPosition(180);
            }
            if (gamepad2.dpad_right) {
                capperServo.setPosition(0);
            }
            if (gamepad2.left_bumper){
                servoPosition = servoPosition + 45;
            }
                else if (gamepad2.right_bumper){
                    servoPosition = servoPosition - 45;
            }
            //outtakeServo.setPosition(servoPosition);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("BoxAngle", currentHeading);
            //telemetry.addData("4. X", String.format("%03d", zValue));
            //telemetry.addData("Gyro Reading", String.valueOf(xValue));//, "x", String.valueOf(yValue), "y", String.valueOf(zValue), "z");
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
            //servoPosition = 0;
        }
    }
}