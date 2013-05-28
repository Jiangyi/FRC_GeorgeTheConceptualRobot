/*----------------------------------------------------------------------------*/
/*(c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.RGBImage;
import edu.wpi.first.wpilibj.Timer;

public class RobotTemplate extends IterativeRobot {

    //Button Constants for Robot Functionality
    static final int BUTTON_ELEVATOR = 2;
    static final int BUTTON_EXTENDARM = 3;
    static final int BUTTON_CLAMP = 1;
    static final int BUTTON_CAPTURE_IMAGE = 11;
    static final int BUTTON_RELEASE_AIR = 10;
    static final int BUTTON_AUTOSCORE = 7;
    //Motor Values for Digital Sidecar
    static final int FRONT_LEFT_PWM = 2;
    static final int REAR_LEFT_PWM = 1;
    static final int FRONT_RIGHT_PWM = 4;
    static final int REAR_RIGHT_PWM = 3;
    //Colour Filter Settings
    static final int HUE_LOW = 10;
    static final int HUE_HIGH = 30;
    static final int SAT_LOW = 128;
    static final int SAT_HIGH = 204;
    static final int VAL_LOW = 204;
    static final int VAL_HIGH = 242;
    static boolean BUTTON_EXTEND_HELD;
    static boolean BUTTON_CLAMP_HELD;
    float fConvSpeed = 0.0f;
    //Debugging Console Instance
    DriverStationLCD debugConsole = DriverStationLCD.getInstance();
    //Tank Drive System - Two Joysticks for left and right motor control
    //Arcade Drive only needs one stick declaration
    //other stick is used for control scheme, if necessary
    Joystick stickDrive = new Joystick(1);
    //Robot Drive Object
    RobotDrive mainDrive = new RobotDrive(FRONT_LEFT_PWM, REAR_LEFT_PWM, FRONT_RIGHT_PWM, REAR_RIGHT_PWM);
    //Camera Declaration
    AxisCamera camera = AxisCamera.getInstance("10.12.46.11");
    //Images
    ColorImage currentImage;
    BinaryImage filteredImage;
    ColorImage referenceImage;
    //Motor Controls:
    Victor conveyorPickup = new Victor(5);
    //Air Compressor Object
    //constructor: pressureSwitchChannel, compressorRelayChannel
    Compressor mainCompressor = new Compressor(1, 5);
    //Solenoids - used for pistons
    DoubleSolenoid pistonExtend = new DoubleSolenoid(1, 2);
    DoubleSolenoid pistonClamp = new DoubleSolenoid(3, 4);
    DoubleSolenoid pistonRaise = new DoubleSolenoid(5, 6);
    boolean bAutoRun = false;
    int iAirLoopCounter = 0;

    public void robotInit() {
        //starts air compressor
        mainCompressor.start();
        pistonExtend.set(DoubleSolenoid.Value.kOff);
        pistonClamp.set(DoubleSolenoid.Value.kOff);
        pistonRaise.set(DoubleSolenoid.Value.kOff);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

        for (int counter = 0; counter < 6; counter++) {
            mainDrive.drive(0.75, 0.0);
            conveyorPickup.set(0.7);
            Timer.delay(0.5);
        }

        conveyorPickup.set(0.0);
        scoreGoal();



    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        //arcade drive method: (Joystick id, bool squaredInputs)
        mainDrive.arcadeDrive(stickDrive, true);
        //allows the conveyor belt to be controlled by zaxis
        fConvSpeed = (float) stickDrive.getAxis(Joystick.AxisType.kZ);
        //raises or lowers conveyor based on z axis on control stick
        if (stickDrive.getRawButton(BUTTON_ELEVATOR)) {
            conveyorPickup.set(-fConvSpeed);
        } else {
            conveyorPickup.set(0.0);
        }

        //Extend pistons upon button press. If piston already extended, contract pistons instead.
        if (stickDrive.getRawButton(BUTTON_EXTENDARM)) {
            if (!BUTTON_EXTEND_HELD) {
                if (pistonRaise.get() == DoubleSolenoid.Value.kForward
                        && pistonExtend.get() == DoubleSolenoid.Value.kForward) {
                    pistonRaise.set(DoubleSolenoid.Value.kReverse);
                    pistonExtend.set(DoubleSolenoid.Value.kReverse);
                } else {
                    pistonRaise.set(DoubleSolenoid.Value.kForward);
                    pistonExtend.set(DoubleSolenoid.Value.kForward);
                }
                BUTTON_EXTEND_HELD = true;
            }
        } else {
            BUTTON_EXTEND_HELD = false;
        }

        if (stickDrive.getRawButton(BUTTON_CLAMP)) {
            if (!BUTTON_CLAMP_HELD) {
                if (pistonClamp.get() == DoubleSolenoid.Value.kForward) {
                    pistonClamp.set(DoubleSolenoid.Value.kReverse);
                } else {
                    pistonClamp.set(DoubleSolenoid.Value.kForward);
                }
                BUTTON_CLAMP_HELD = true;
            }
        } else {
            BUTTON_CLAMP_HELD = false;
        }

        if (stickDrive.getRawButton(BUTTON_AUTOSCORE)) {
            scoreGoal();
        }
        //release pressure
        if (stickDrive.getRawButton(BUTTON_RELEASE_AIR)) {
            pistonClamp.set(DoubleSolenoid.Value.kOff);
            pistonExtend.set(DoubleSolenoid.Value.kOff);
            pistonRaise.set(DoubleSolenoid.Value.kOff);
            mainCompressor.stop();
        }

        if (mainCompressor.getPressureSwitchValue()) {
            mainCompressor.stop();
        }
        //camera image capture construct
        if (stickDrive.getRawButton(BUTTON_CAPTURE_IMAGE) && camera.freshImage()) {
            try {
                currentImage = camera.getImage();
                currentImage.write("originalCapture.jpg");
                debugConsole.println(DriverStationLCD.Line.kMain6, 1, "Image captured!");
                debugConsole.updateLCD();

                ColorImage trypicture = new RGBImage("originalCapture.jpg");
                trypicture.colorEqualize();
                trypicture.write("equalized.jpg");
                filteredImage = trypicture.thresholdHSV(HUE_LOW, HUE_HIGH,
                        SAT_LOW, SAT_HIGH,
                        VAL_LOW, VAL_HIGH);
                BinaryImage convexImg = filteredImage.convexHull(true);
                convexImg.write("convexHull.bmp");
                //filteredImage = filteredImage.removeSmallObjects(true, 8);
                filteredImage.write("filteredImage.bmp");
                debugConsole.println(DriverStationLCD.Line.kMain6, 1, "Filtered image produced!");
                debugConsole.updateLCD();
                currentImage.free();
                filteredImage.free();
                trypicture.free();
                convexImg.free();
            } catch (NIVisionException e) {
                String msg = e.getMessage();
                debugConsole.println(DriverStationLCD.Line.kMain6, 1, msg + ";;");
                debugConsole.updateLCD();
            } catch (AxisCameraException e) {
                String msg = e.getMessage();
                debugConsole.println(DriverStationLCD.Line.kMain6, 1, msg);
                debugConsole.updateLCD();
            }
        }


    }

    //Autonomous method for scoring a goal
    private void scoreGoal() {

        //forward motion - clamps ball, extends and raises up
        pistonClamp.set(DoubleSolenoid.Value.kForward);
        Timer.delay(0.5);
        pistonRaise.set(DoubleSolenoid.Value.kForward);
        Timer.delay(1.0);
        pistonExtend.set(DoubleSolenoid.Value.kForward);

        Timer.delay(1.0);

        //reverse motion - drops ball into hole, and returns to base
        pistonClamp.set(DoubleSolenoid.Value.kReverse);
        Timer.delay(0.5);
        pistonExtend.set(DoubleSolenoid.Value.kReverse);
        Timer.delay(1.0);
        pistonRaise.set(DoubleSolenoid.Value.kReverse);

    }
}
