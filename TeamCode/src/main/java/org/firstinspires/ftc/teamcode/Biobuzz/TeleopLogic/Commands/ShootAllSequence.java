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
                protected void Actions()
                {
                    storage.unlockPuta();

                }

                @Override
                protected boolean Conditions() {
                    return true;
                }
            })
            .waitSeconds(0.1)
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    storage.transfer();
                }

                @Override
                protected boolean Conditions() {
                    return true;
                }
            })

            .waitSeconds(0.7)
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    storage.stop();
                }

                @Override
                protected boolean Conditions() {
                    return true;
                }
            })
            .waitSeconds(0.2)
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    storage.lockPuta();
                }

                @Override
                protected boolean Conditions() {
                    return true;
                }
            });

}
