package org.firstinspires.ftc.teamcode.Biobuzz.TeleopLogic.Commands;

import static org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter.storage;

import org.firstinspires.ftc.teamcode.Biobuzz.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.Biobuzz.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.ShooterCalculator;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Storage;

public class ShootAllSequence {

    public static Scheduler ShootTasks = new Scheduler()
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    storage.unlockPuta();
                    ShooterCalculator.shouldCompensateHood = true;

                }

                @Override
                protected boolean Conditions() {
                    return true;
                }
            })
            .addTask(new Task() {
                @Override
                protected void Actions()
                {

                    //if(Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), Turret.goalPosition) > 2500)
                    //    storage.takeinTransfer();
                    //else
                    storage.takeIn();
                }

                @Override
                protected boolean Conditions() {
                    return true;
                }
            })
            .waitSeconds(0.5)
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    storage.lockPuta();
                }

                @Override
                protected boolean Conditions() {
                    return true;
                }
            })
            .waitSeconds(0.13)
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    storage.stop();
                    ShooterCalculator.shouldCompensateHood = false;
                }

                @Override
                protected boolean Conditions() {
                    return true;
                }
            });

}
