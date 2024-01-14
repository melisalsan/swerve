// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.autonomous.Autonomous;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.shooter.Shooter;

public class Robot extends TimedRobot {
    private final Autonomous autonomous = Autonomous.getInstance();

    private final Subsystem[] subsystems = new Subsystem[] {
            Infrastructure.getInstance(),
            Shooter.getInstance(),
            Intake.getInstance(),
            Superstructure.getInstance(),
            Pneumatics.getInstance(),
            Drivebase.getInstance()
    };

    private boolean enabled = false;

    /**
     * Code that runs once when the robot code starts
     */
    @Override
    public void robotInit() {
        autonomous.robotInit();

        for (Subsystem subsystem : subsystems) {
            subsystem.robotInit();
        }
    }

    /**
     * Code that runs once per robot tick
     */
    @Override
    public void robotPeriodic() {
        for (Subsystem subsystem : subsystems) {
            subsystem.robotPeriodic();
        }

        Infrastructure.getInstance().enabled = isEnabled();
    }

    /**
     * Code that runs once when the autonomous period starts
     */
    @Override
    public void autonomousInit() {
        autonomous.autonomousInit();

        if (!enabled) {
            for (Subsystem subsystem : subsystems) {
                subsystem.enabledInit();
            }
            enabled = true;
        }
    }

    /**
     * Code that runs once per autonomous tick
     */
    @Override
    public void autonomousPeriodic() {
      autonomous.autonomousPeriodic();
    }

    /**
     * Code that runs once when the teleop period starts
     */
    @Override
    public void teleopInit() {
      if (!enabled) {
          for (Subsystem subsystem : subsystems) {
              subsystem.enabledInit();
          }
          enabled = true;
      }

      for (Subsystem subsystem : subsystems) {
          subsystem.teleopInit();
      }
    }

    /**
     * Code that runs once per teleop tick
     */
    @Override
    public void teleopPeriodic() {
      for (Subsystem subsystem : subsystems) {
          subsystem.teleopPeriodic();
      }
    }

    /**
     * Code that runs once when the robot is disabled
     */
    @Override
    public void disabledInit() {
        enabled = false;
        for (Subsystem subsystem : subsystems) {
            subsystem.disabledInit();
        }
    }

    /**
     * Code that runs once per disabled tick
     */
    @Override
    public void disabledPeriodic() {
        for (Subsystem subsystem : subsystems) {
            subsystem.disabledPeriodic();
        }
    }

    /**
     * Code that runs once when the robot is put into test mode
     */
    @Override
    public void testInit() {
        for (Subsystem subsystem : subsystems) {
            subsystem.testInit();
        }
    }

    /**
     * Code that runs once per test tick
     */
    @Override
    public void testPeriodic() {
        for (Subsystem subsystem : subsystems) {
            subsystem.testPeriodic();
        }
    }
}
