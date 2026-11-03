package org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.SwerveKinematics;
import org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.SwervePod;

import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Locale;

/**
 * Auto-tuning P si D pentru TOATE cele 4 poduri swerve, in paralel.
 *
 * Algoritm:
 *  1) Test cu releu (Astrom-Hagglund): fiecare pod oscileaza mic in jurul unghiului de start,
 *     din amplitudine + perioada se calculeaza Ku si Tu, iar din ele un P si D de pornire.
 *  2) Twiddle (coordinate descent) in spatiu logaritmic, pe hardware real: fiecare candidat
 *     e testat cu acelasi set de pasi de unghi (90, 60, 25, 3 grade) si primeste un cost
 *     (eroare ponderata in timp + overshoot + tremurat la tinta). Se pastreaza ce scade costul.
 *     Faza A: un singur set P/D pentru tot. Faza B: doar smallCoef (|err| <= 4).
 *     bigCoef = |err| > 4, smallCoef = |err| <= 4.
 *
 * Bucla ruleaza Localizer.Update() + clearCache() exact ca in TeleOp, ca dt-ul sa fie realist.
 * B pe gamepad1 = opreste si pastreaza cele mai bune valori gasite pana acum.
 */
@TeleOp(name = "AutoTuneSwervePD", group = "Tuning")
@Config
public class AutoTunePods extends LinearOpMode {

    // ---------------- Test directie ----------------
    public static double DIR_TEST_POWER = 0.3;
    public static double DIR_TEST_TIME_S = 0.25;
    public static double DIR_MIN_MOVE_DEG = 3;
    public static double SERVO_ENABLE_SETTLE_S = 0.5;

    // ---------------- Releu ----------------
    public static double RELAY_POWER = 0.25;          // putere servo in testul cu releu
    public static double RELAY_HYSTERESIS_DEG = 1.0;  // anti-zgomot encoder
    public static double RELAY_MAX_TIME_S = 6.0;
    public static int RELAY_CYCLES = 7;
    public static double RELAY_P_FACTOR = 0.5;        // Kp_start = factor * Ku
    public static double RELAY_TD_FACTOR = 0.125;     // Kd_start = Kp * factor * Tu

    // ---------------- Trial-uri ----------------
    public static double STEP_TIME_S = 0.5;           // cat sta pe fiecare unghi
    public static double TOLERANCE_DEG = 0.75;        // eroare sub asta = 0 in cost (zgomot encoder)
    public static double W_OVERSHOOT = 1.0;
    public static double W_CHATTER = 0.01;            // penalizare pentru tremurat dupa ce a ajuns
    public static int MAX_TRIALS_PER_PHASE = 30;
    public static double CONVERGED_LOG_STEP = 0.05;   // ~5% schimbare -> gata
    public static boolean TUNE_NEAR_GAINS = true;     // faza B: smallCoef (|err| <= 4)
    public static boolean KEEP_EXISTING_F = true;     // F nu se tuneaza; il pastreaza pe cel actual
    public static double EXTRA_LOOP_MS = 0;           // simuleaza alte subsisteme in loop

    private static final double[] STEP_PATTERN = {90, 0, -60, 0, 25, 0, -3, 0, 3, 0};
    private static final double NEAR_BAND_DEG = 4;
    private static final double UNSTABLE_PENALTY = 50;
    private static final String TAG = "AutoTuneSwerve";

    private final ElapsedTime clock = new ElapsedTime();
    private double now, dt, lastTime, loopMsAvg;
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private boolean aborted = false;
    private String phaseName = "init";
    private int trialNo = 0;
    private PodTuner[] tuners;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);

        tuners = new PodTuner[]{
                new PodTuner("FL", SwerveKinematics.frontLeftModule),
                new PodTuner("FR", SwerveKinematics.frontRightModule),
                new PodTuner("BL", SwerveKinematics.backLeftModule),
                new PodTuner("BR", SwerveKinematics.backRightModule)
        };

        telemetry.addLine("Pune robotul pe podea, nimic langa roti.");
        telemetry.addLine("START = incepe (~5-8 min). B = stop si pastreaza ce e mai bun.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        clock.reset();
        lastTime = 0;
        RobotInitializers.clearCache();

        // ---------- Faza -1: directia fiecarui servo (pe rand, ca sa prinda si fire inversate) ----------
        // Intai trimitem 0 pe toate si asteptam: un CR servo poate zvacni cand primeste primul PWM,
        // iar zvacnitul ar pica in fereastra de test a primului pod (FL).
        phaseName = "pornire servo-uri";
        double settleStart = now;
        while (tick() && now - settleStart < SERVO_ENABLE_SETTLE_S) {
            for (PodTuner t : tuners) t.setServo(0);
        }

        phaseName = "test directie";
        for (int k = 0; k < tuners.length && !aborted && opModeIsActive(); k++) directionTest(k);

        for (PodTuner t : tuners) t.captureStart();

        // ---------- Faza 0: releu ----------
        phaseName = "releu";
        for (PodTuner t : tuners) t.relayBegin();
        double relayStart = now;
        while (tick()) {
            boolean allDone = true;
            for (PodTuner t : tuners) {
                t.relayUpdate();
                if (!t.relayDone()) allDone = false;
            }
            if (allDone || now - relayStart > RELAY_MAX_TIME_S) break;
        }
        for (PodTuner t : tuners) t.relayFinish();

        // ---------- Faza A: un singur set P/D ----------
        if (!aborted && opModeIsActive()) runPhase(PodTuner.PHASE_FAR);
        // ---------- Faza B: doar setul pentru |err| <= 4 ----------
        if (TUNE_NEAR_GAINS && !aborted && opModeIsActive()) runPhase(PodTuner.PHASE_NEAR);

        // ---------- Gata ----------
        phaseName = aborted ? "oprit (B) - valori cele mai bune" : "GATA";
        StringBuilder report = new StringBuilder();
        for (PodTuner t : tuners) {
            t.applyBest();
            t.holdTarget(t.a0);
            report.append(t.report()).append('\n');
        }
        RobotLog.ii(TAG, report.toString());
        saveReport(report.toString());

        while (opModeIsActive()) {
            tickNoAbort();
            for (PodTuner t : tuners) t.hold();
            if (telemetryTimer.milliseconds() > 250) {
                telemetryTimer.reset();
                telemetry.addLine(phaseName);
                telemetry.addLine(report.toString());
                telemetry.addLine("Salvat si in /sdcard/FIRST/swerve_pd_autotune.txt");
                telemetry.update();
            }
        }
        for (PodTuner t : tuners) t.pod.stop();
    }

    private void runPhase(int phase) {
        phaseName = phase == PodTuner.PHASE_FAR ? "A: P/D comun" : "B: smallCoef (|err|<=4)";
        for (PodTuner t : tuners) t.startPhase(phase);

        trialNo = 0;
        while (trialNo < MAX_TRIALS_PER_PHASE && !allOptimizersDone()) {
            for (PodTuner t : tuners) t.trialBegin();

            for (int s = 0; s < STEP_PATTERN.length; s++) {
                for (PodTuner t : tuners) t.stepBegin(STEP_PATTERN[s]);
                double stepStart = now;
                while (tick()) {
                    double ts = now - stepStart;
                    for (PodTuner t : tuners) t.stepSample(ts, dt);
                    if (ts >= STEP_TIME_S) break;
                }
                if (aborted || !opModeIsActive()) break;
                for (PodTuner t : tuners) t.stepEnd();
            }
            if (aborted || !opModeIsActive()) break;   // trial incomplet -> nu se raporteaza

            for (PodTuner t : tuners) t.trialEnd();
            trialNo++;
        }
        for (PodTuner t : tuners) t.finishPhase();
    }

    /**
     * Misca DOAR podul k cu putere pozitiva si vede care encoder se misca si in ce sens.
     * Conventia controllerului (out = -K * err) cere ca putere + sa SCADA unghiul.
     * Daca la voi putere + CRESTE unghiul, podul primeste semn = -1 si gain-urile ies negative.
     */
    private void directionTest(int k) {
        PodTuner me = tuners[k];
        double[] powers = {DIR_TEST_POWER, Math.min(1, DIR_TEST_POWER * 2)};
        double[] delta = new double[tuners.length];
        int jMax = k;

        for (double power : powers) {
            double[] start = new double[tuners.length];
            for (int j = 0; j < tuners.length; j++) {
                tuners[j].pod.CalculategetAngle();
                start[j] = tuners[j].pod.getAngle();
            }
            double t0 = now;
            while (tick() && now - t0 < DIR_TEST_TIME_S) {
                for (int j = 0; j < tuners.length; j++) tuners[j].setServo(j == k ? power : 0);
            }
            me.setServo(0);
            t0 = now;
            while (tick() && now - t0 < 0.2) { /* lasa-l sa se opreasca */ }
            if (aborted || !opModeIsActive()) return;

            jMax = k;
            for (int j = 0; j < tuners.length; j++) {
                tuners[j].pod.CalculategetAngle();
                delta[j] = tuners[j].pod.error(tuners[j].pod.getAngle(), start[j]);   // acum - start
                if (Math.abs(delta[j]) > Math.abs(delta[jMax])) jMax = j;
            }
            // Daca encoderul PROPRIU s-a miscat destul, e bine, chiar daca alt pod a zvacnit si el.
            if (Math.abs(delta[k]) >= DIR_MIN_MOVE_DEG) break;
        }

        if (Math.abs(delta[k]) >= DIR_MIN_MOVE_DEG) {
            me.sign = delta[k] < 0 ? 1 : -1;
            me.dirNote = String.format(Locale.US, "putere+ -> %+.0f grade, semn %s", delta[k], me.sign > 0 ? "+" : "-");
        } else if (Math.abs(delta[jMax]) >= DIR_MIN_MOVE_DEG) {
            // propriul encoder n-a miscat, dar altul da -> chiar pare fir/port inversat
            me.disabled = true;
            me.note = "servo-ul lui " + me.name + " misca encoderul lui " + tuners[jMax].name + " - fire/porturi inversate?";
        } else {
            me.disabled = true;
            me.note = String.format(Locale.US, "NU SE MISCA (%.1f grade) - servo/encoder/alimentare?", delta[k]);
        }
    }

    private boolean allOptimizersDone() {
        for (PodTuner t : tuners) if (!t.disabled && !t.opt.done) return false;
        return true;
    }

    /** O iteratie de loop, la fel de "grea" ca in TeleOp. */
    private boolean tick() {
        tickNoAbort();
        if (gamepad1.b) aborted = true;

        if (telemetryTimer.milliseconds() > 150) {
            telemetryTimer.reset();
            telemetry.addData("Faza", phaseName);
            telemetry.addData("Trial", "%d / %d", trialNo, MAX_TRIALS_PER_PHASE);
            telemetry.addData("Loop", "%.1f ms (%.0f Hz)", loopMsAvg, 1000.0 / Math.max(loopMsAvg, 0.1));
            for (PodTuner t : tuners) telemetry.addLine(t.statusLine());
            telemetry.addLine("B = stop si pastreaza cele mai bune valori");
            telemetry.update();
        }
        return opModeIsActive() && !aborted;
    }

    private void tickNoAbort() {
        Localizer.Update();
        if (EXTRA_LOOP_MS > 0) sleep((long) EXTRA_LOOP_MS);
        RobotInitializers.clearCache();

        now = clock.seconds();
        dt = now - lastTime;
        lastTime = now;
        loopMsAvg = loopMsAvg == 0 ? dt * 1000 : loopMsAvg * 0.95 + dt * 1000 * 0.05;
    }

    private void saveReport(String text) {
        try {
            FileWriter w = new FileWriter("/sdcard/FIRST/swerve_pd_autotune.txt");
            w.write(text);
            w.close();
        } catch (Exception e) {
            RobotLog.ww(TAG, "Nu am putut salva fisierul: " + e.getMessage());
        }
    }

    // =====================================================================================
    /** Starea de tuning pentru un pod. Toate cele 4 ruleaza in acelasi loop, in paralel. */
    private class PodTuner {
        static final int PHASE_FAR = 0, PHASE_NEAR = 1;

        final String name;
        final SwervePod pod;
        double a0;
        double fFar, fNear;
        double farP, farD, nearP, nearD;
        double startFarP, startFarD, startNearP, startNearD;
        double relayP = -1, relayD = -1;
        boolean disabled = false;
        String note = "", dirNote = "";
        int sign = 1;   // -1 daca la pod putere+ creste unghiul -> gain-urile se scriu negative

        // releu
        double relayOut, lastUpSwitch = -1, cycleMax, cycleMin, absErrSum;
        int absErrN;
        final ArrayList<Double> periods = new ArrayList<>(), amps = new ArrayList<>();

        // optimizator
        Twiddle opt;
        int phase;

        // cost
        double cost, lastCost = Double.NaN, target, stepSize, stepDir, maxOvershoot, lastPower;
        boolean reached, cut;

        PodTuner(String name, SwervePod pod) {
            this.name = name;
            this.pod = pod;
        }

        void captureStart() {
            pod.CalculategetAngle();
            a0 = pod.getAngle();
            target = a0;
            // bigCoef = |err| > 4 (far), smallCoef = |err| <= 4 (near)
            // valorile existente pot fi negative (servo inversat) -> lucram cu modulul
            startFarP = Math.abs(pod.bigCoef.p);     startFarD = Math.abs(pod.bigCoef.i);
            startNearP = Math.abs(pod.smallCoef.p);  startNearD = Math.abs(pod.smallCoef.i);
            fFar = KEEP_EXISTING_F ? Math.abs(pod.bigCoef.d) : 0;
            fNear = KEEP_EXISTING_F ? Math.abs(pod.smallCoef.d) : 0;
            farP = startFarP; farD = startFarD; nearP = startNearP; nearD = startNearD;
            pod.motorPower = 0;
        }

        // ---------------- releu ----------------
        void relayBegin() {
            relayOut = RELAY_POWER;
            cycleMax = -1e9;
            cycleMin = 1e9;
        }

        void relayUpdate() {
            pod.CalculategetAngle();
            if (disabled) { setServo(0); return; }
            double e = pod.error(a0, pod.getAngle());
            absErrSum += Math.abs(e);
            absErrN++;
            cycleMax = Math.max(cycleMax, e);
            cycleMin = Math.min(cycleMin, e);

            // Conventia din EfficientPDFController: putere = -K * eroare
            if (relayOut > 0 && e > RELAY_HYSTERESIS_DEG) {
                relayOut = -RELAY_POWER;
            } else if (relayOut < 0 && e < -RELAY_HYSTERESIS_DEG) {
                relayOut = RELAY_POWER;
                if (lastUpSwitch >= 0) {
                    periods.add(now - lastUpSwitch);
                    amps.add((cycleMax - cycleMin) / 2.0);
                }
                lastUpSwitch = now;
                cycleMax = -1e9;
                cycleMin = 1e9;
            }
            setServo(sign * relayOut);
            Dash(name + " err", e);
        }

        boolean relayDone() {
            return disabled || periods.size() >= RELAY_CYCLES;
        }

        void relayFinish() {
            setServo(0);
            if (disabled) return;   // deja oprit la testul de directie, pastram mesajul
            double meanAbs = absErrN > 0 ? absErrSum / absErrN : 0;

            if (meanAbs > 45) {
                // Servo-ul a fugit de tinta: sensul e invers fata de conventia controllerului.
                disabled = true;
                note = "a fugit de tinta in testul cu releu (encoder zgomotos / sare?)";
                return;
            }
            if (periods.size() < 3) {
                note = "releu fara oscilatii (RELAY_POWER prea mic?)";
                seedFallback();
                return;
            }
            // primul ciclu e tranzitoriu -> il sarim
            double pSum = 0, aSum = 0;
            int n = 0;
            for (int k = 1; k < periods.size(); k++) {
                pSum += periods.get(k);
                aSum += amps.get(k);
                n++;
            }
            double tu = pSum / n, a = aSum / n;
            double h = RELAY_HYSTERESIS_DEG;
            double ku = 4 * RELAY_POWER / (Math.PI * Math.sqrt(Math.max(a * a - h * h, 1e-4)));
            relayP = RELAY_P_FACTOR * ku;
            relayD = relayP * RELAY_TD_FACTOR * tu;
            note = String.format(Locale.US, "Ku=%.4f Tu=%.3fs", ku, tu);
        }

        void seedFallback() {
            if (startFarP > 0) { relayP = startFarP; relayD = Math.max(startFarD, startFarP * 0.002); }
            else { relayP = 0.01; relayD = 0.0003; }
        }

        // ---------------- faze ----------------
        void startPhase(int phase) {
            this.phase = phase;
            if (disabled) return;
            ArrayList<double[]> seeds = new ArrayList<>();
            if (phase == PHASE_FAR) {
                seeds.add(log(relayP, relayD));
                if (startFarP > 0) seeds.add(log(startFarP, startFarD));
                opt = new Twiddle(seeds, 0.5);
            } else {
                seeds.add(log(farP, farD));        // rezultatul din faza A
                seeds.add(log(farP * 2, farD));    // aproape de tinta ajuta de obicei un P mai mare
                if (startNearP > 0) seeds.add(log(startNearP, startNearD));
                opt = new Twiddle(seeds, 0.4);
            }
        }

        void finishPhase() {
            if (disabled || opt == null) return;
            double[] best = opt.best();
            setFromX(best);
        }

        void setFromX(double[] x) {
            double p = Math.exp(x[0]), d = Math.exp(x[1]);
            if (phase == PHASE_FAR) { farP = nearP = p; farD = nearD = d; }
            else { nearP = p; nearD = d; }
        }

        // ---------------- trial ----------------
        void trialBegin() {
            cost = 0;
            cut = false;
            if (disabled) return;
            setFromX(opt.done ? opt.best() : opt.candidate());
            applyGains();
            pod.controller.reset();
        }

        void stepBegin(double offset) {
            pod.CalculategetAngle();
            target = norm(a0 + offset);
            double e0 = pod.error(target, pod.getAngle());
            stepSize = Math.max(Math.abs(e0), 1);
            stepDir = Math.signum(e0);
            pod.targetAngle = target;   // direct, fara flip-ul de 180 din set()
            pod.motorPower = 0;
            maxOvershoot = 0;
            reached = false;
            lastPower = pod.lastServoPower;
        }

        void stepSample(double ts, double dt) {
            if (disabled || cut) {
                pod.CalculategetAngle();
                setServo(0);
                return;
            }
            pod.update(1);
            double e = pod.getError();

            double eAbs = Math.max(0, Math.abs(e) - TOLERANCE_DEG);
            cost += eAbs / stepSize * (0.2 + ts) * dt;            // ITAE normalizat

            double os = -stepDir * e;
            if (os > maxOvershoot) maxOvershoot = os;

            if (Math.abs(e) < 2) reached = true;
            if (reached) cost += W_CHATTER * Math.abs(pod.lastServoPower - lastPower);
            lastPower = pod.lastServoPower;

            if (Math.abs(e) > stepSize + 30) {                   // instabil -> taie
                cut = true;
                cost += UNSTABLE_PENALTY;
                pod.smallCoef = new PIDCoefficients(0, 0, 0);
                pod.bigCoef = new PIDCoefficients(0, 0, 0);
                setServo(0);
            }
            Dash(name + " err", e);
        }

        void stepEnd() {
            if (disabled || cut) return;
            cost += W_OVERSHOOT * Math.max(0, maxOvershoot - TOLERANCE_DEG) / stepSize;
        }

        void trialEnd() {
            if (disabled) return;
            lastCost = cost;
            if (!opt.done) opt.report(cost);
        }

        // ---------------- final ----------------
        void applyBest() {
            if (disabled) {
                farP = startFarP; farD = startFarD; nearP = startNearP; nearD = startNearD;
            }
            applyGains();
        }

        void holdTarget(double angle) {
            pod.targetAngle = angle;
            pod.motorPower = 0;
            pod.controller.reset();
        }

        void hold() {
            if (disabled) { pod.CalculategetAngle(); setServo(0); return; }
            pod.update(1);
        }

        void applyGains() {
            pod.bigCoef = new PIDCoefficients(sign * farP, sign * farD, sign * fFar);       // |err| > 4
            pod.smallCoef = new PIDCoefficients(sign * nearP, sign * nearD, sign * fNear);  // |err| <= 4
        }

        void setServo(double p) {
            pod.servo.setPower(p);
            pod.lastServoPower = p;
        }

        String statusLine() {
            if (disabled) return name + ": " + note;
            String o = opt == null ? "" : String.format(Locale.US, " best=%.4f pas=%.2f%s",
                    opt.bestCost, opt.maxStep(), opt.done ? " OK" : "");
            return String.format(Locale.US, "%s(%s) big P=%.5f D=%.6f | small P=%.5f D=%.6f | cost=%.4f%s",
                    name, sign > 0 ? "+" : "-", farP, farD, nearP, nearD, lastCost, o);
        }

        String report() {
            String field = name.equals("FL") ? "frontLeftModule" : name.equals("FR") ? "frontRightModule"
                    : name.equals("BL") ? "backLeftModule" : "backRightModule";
            if (disabled) return name + ": NETUNAT - " + note + (dirNote.isEmpty() ? "" : " (" + dirNote + ")");
            return String.format(Locale.US,
                    "%s (%s)\n%s.bigCoef   = new PIDCoefficients(%.6f, %.7f, %.4f); // |err| > 4\n"
                            + "%s.smallCoef = new PIDCoefficients(%.6f, %.7f, %.4f); // |err| <= 4",
                    name, note + "; " + dirNote, field, sign * farP, sign * farD, sign * fFar + 0.0,
                    field, sign * nearP, sign * nearD, sign * fNear + 0.0);
        }

        private void Dash(String k, double v) {
            RobotInitializers.Dashtelemetry.addData(k, v);
        }
    }

    // =====================================================================================
    /**
     * Twiddle (coordinate descent) in spatiu log: x = [ln P, ln D].
     * Pasii sunt multiplicativi (0.5 in log = x1.65), deci merge si pentru D ~ 0.0001 si pentru P ~ 0.01.
     * Re-evalueaza periodic cel mai bun punct ca un trial "norocos" sa nu blocheze cautarea.
     */
    static class Twiddle {
        static final int SEED = 0, PLUS = 1, MINUS = 2, REEVAL = 3;
        static final double GROW = 1.3, SHRINK = 0.6;

        final ArrayList<double[]> seeds;
        double[] x;
        final double[] dx;
        double bestCost = Double.POSITIVE_INFINITY;
        int state = SEED, i = 0, seedIdx = 0;
        boolean done = false;

        Twiddle(ArrayList<double[]> seeds, double initialStep) {
            this.seeds = seeds;
            x = seeds.get(0).clone();
            dx = new double[x.length];
            for (int k = 0; k < dx.length; k++) dx[k] = initialStep;
        }

        double[] candidate() {
            double[] c = x.clone();
            if (state == SEED) return seeds.get(seedIdx).clone();
            if (state == PLUS) c[i] += dx[i];
            if (state == MINUS) c[i] -= dx[i];
            return c;
        }

        void report(double cost) {
            switch (state) {
                case SEED:
                    if (cost < bestCost) { bestCost = cost; x = seeds.get(seedIdx).clone(); }
                    seedIdx++;
                    if (seedIdx >= seeds.size()) state = PLUS;
                    break;
                case PLUS:
                    if (cost < bestCost) {
                        bestCost = cost; x[i] += dx[i]; dx[i] *= GROW; advance();
                    } else state = MINUS;
                    break;
                case MINUS:
                    if (cost < bestCost) {
                        bestCost = cost; x[i] -= dx[i]; dx[i] *= GROW;
                    } else dx[i] *= SHRINK;
                    advance();
                    break;
                case REEVAL:
                    bestCost = 0.5 * (bestCost + cost);
                    state = PLUS;
                    break;
            }
        }

        private void advance() {
            i++;
            if (i >= x.length) {
                i = 0;
                if (maxStep() < CONVERGED_LOG_STEP) { done = true; return; }
                state = REEVAL;
            } else state = PLUS;
        }

        double maxStep() {
            double m = 0;
            for (double d : dx) m = Math.max(m, d);
            return m;
        }

        double[] best() { return x.clone(); }
    }

    private static double[] log(double p, double d) {
        return new double[]{Math.log(Math.max(p, 1e-6)), Math.log(Math.max(d, 1e-7))};
    }

    private static double norm(double a) {
        a %= 360;
        if (a < 0) a += 360;
        return a;
    }
}