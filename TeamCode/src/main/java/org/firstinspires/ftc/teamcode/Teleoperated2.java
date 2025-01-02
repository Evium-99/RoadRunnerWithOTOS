package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="EviumAtom", group="TeleOp")
public class Teleoperated2 extends OpMode {

    // Motors & Sensors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotorEx cap;
    private DcMotorEx cap2;
    private PIDController capPID = new PIDController(0.02, 0, 0);
    private DcMotor spindle;
    private SparkFunOTOS odometry;
    private RevTouchSensor Lswitch;
    private double capSetpoint = 0;

    // Servo Button Toggles
    private double vindoViper = 0;
    private boolean switchViperDirection = false;
    private double capPower;
    private Rev2mDistanceSensor frontDistance;
    private Rev2mDistanceSensor backDistance;
    private boolean emergencyDisableLimit = false;
    private boolean emergencyDisableLimitDown = false;
    private PIDController distancePID = new PIDController(0.1, 0, 0);

    // Servos
    private ServoImplEx LSLower;
    private ServoImplEx LSTop;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

        spindle = hardwareMap.get(DcMotor.class, "spindle");
        odometry = hardwareMap.get(SparkFunOTOS.class, "odometry");
        cap = hardwareMap.get(DcMotorEx.class, "cap");
        cap2 = hardwareMap.get(DcMotorEx.class, "cap2");
        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cap.setDirection(DcMotorSimple.Direction.FORWARD);
        cap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cap2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cap2.setDirection(DcMotorSimple.Direction.REVERSE);
        capPID.setIntegrationBounds(0, 0.01);
        capPID.setTolerance(8);
        LSLower = hardwareMap.get(ServoImplEx.class, "LSLower");
        LSTop = hardwareMap.get(ServoImplEx.class, "LSTop");
        Lswitch = hardwareMap.get(RevTouchSensor.class, "Lswitch");
        frontDistance = hardwareMap.get(Rev2mDistanceSensor.class, "frontDistance");
        backDistance = hardwareMap.get(Rev2mDistanceSensor.class, "backDistance");


        // Define Servo range
        LSLower.setPwmEnable();
        LSTop.setPwmEnable();
        LSLower.scaleRange(0, 1);
        LSTop.scaleRange(0, 1);

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Breaking mode
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Make sure odo is ready
        odometry.begin();
        odometry.setPosition(new SparkFunOTOS.Pose2D(0,0,0));
    }

    @Override
    public void start() {
        telemetry.speak("Evium Start");
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    }

    @Override
    public void loop() {
        // Get joystick values
        double y;
        double x;
        double rx;

        // Toggle Emergency Limit Disable
        if (gamepad2.left_bumper && !emergencyDisableLimitDown) {
            emergencyDisableLimit = !emergencyDisableLimit;
        }
        emergencyDisableLimitDown = gamepad2.left_bumper;

        if (gamepad1.right_trigger > 0.33) {
            y = -gamepad1.left_stick_y * 0.2;
            x = gamepad1.left_stick_x * 0.5;
            rx = gamepad1.right_stick_x * 0.25;
        } else if (gamepad1.a) {
            distancePID.setSetPoint(5.8);
            y = - distancePID.calculate(frontDistance.getDistance(DistanceUnit.INCH));
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
        } else {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
        }

        // Set power to motors
        frontLeftMotor.setPower(y + x + rx);
        backLeftMotor.setPower(y - x + rx);
        frontRightMotor.setPower(y - x - rx);
        backRightMotor.setPower(y + x - rx);
        if (gamepad2.x) {
            LSLower.setPosition(0.07);
            LSTop.setPosition(0.93);
        }
        if (gamepad2.a) {
            LSLower.setPosition(0.73);
            LSTop.setPosition(0.27);
        }
        if (gamepad2.y) {
            LSLower.setPosition(0.4);
            LSTop.setPosition(0.6);
        }
        if ((gamepad2.b) || (gamepad1.b)) {
            if (vindoViper > 0.94) {
                switchViperDirection = true;
            }
            if (vindoViper < 0.06) {
                switchViperDirection = false;
            }
            if (switchViperDirection) {
                vindoViper = vindoViper - 0.05;
            } else {
                vindoViper = vindoViper + 0.05;
            }
            LSLower.setPosition(1 - vindoViper);
            LSTop.setPosition(1 - vindoViper);
        }
        if (gamepad2.left_trigger > 0.05) {
            capSetpoint = (capSetpoint - (gamepad2.left_trigger * 10));
        }
        if (gamepad2.right_trigger > 0.05) {
            capSetpoint = (capSetpoint + (gamepad2.right_trigger * 10));
        }
        if (gamepad2.dpad_up) {
            capSetpoint = 375;
        }
        if (gamepad2.dpad_down) {
            capSetpoint = 40;
        }
        if (!emergencyDisableLimit) {
            if (capSetpoint > 800) {
                capSetpoint = 800;
            }
            if (capSetpoint < 30) {
                capSetpoint = 30;
            }
        }
        double capPower = capPID.calculate(cap.getCurrentPosition(), capSetpoint);
        cap.setPower(capPower);
        cap2.setPower(capPower);

        if (Lswitch.isPressed()) {
            telemetry.speak("Warning Capstan. Pull Up");
            cap.setPower(capPower * 0.1);
            cap.setPower(capPower * 0.1);
        }
        spindle.setPower(-gamepad2.left_stick_y * 0.75);

        // Telemetry for debugging
        telemetry.addData("EmergencyDisableLimit", emergencyDisableLimit);
        telemetry.addData("DistanceFront", frontDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("DistanceBack", backDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("CapstanValue", cap.getCurrentPosition());
        telemetry.update();
    }
}
