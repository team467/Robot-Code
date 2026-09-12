package frc.robot.sim;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Hub;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.RobotState;
import frc.robot.RobotState.IntakePosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.extend.IntakeExtend;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.magicCarpet.MagicCarpet;
import frc.robot.subsystems.shooter.Shooter;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.geom.*;
import java.util.List;
import javax.swing.*;
import javax.swing.border.EmptyBorder;
import javax.swing.border.LineBorder;

/**
 * Custom Java Swing UI Dashboard window displaying real-time robot state, subsystem diagnostics,
 * ball inventory, top-down field view, and dynamic 2D shooting trajectory arcs.
 */
public class SimDashboardWindow extends JFrame {
  private static SimDashboardWindow instance;

  private Drive drive;
  private Shooter shooter;
  private Indexer indexer;
  private MagicCarpet magicCarpet;
  private IntakeRollers intakeRollers;
  private IntakeExtend intakeExtend;

  private final BallSimulator ballSimulator = BallSimulator.getInstance();

  // Autonomous countdown tracking
  public static final double AUTO_DURATION_SECONDS = 20.0;
  private double autoStartTime = 0.0;
  private boolean autoRunning = false;
  private JButton autoCardBtn;

  // UI Panels
  private FieldVisualizerPanel fieldPanel;
  private TrajectoryVisualizerPanel trajectoryPanel;
  private SubsystemsPanel telemetryPanel;
  private RobotAnimationPanel robotAnimPanel;

  public double getAutoRemainingSeconds() {
    if (!DriverStation.isEnabled() || !DriverStation.isAutonomous()) {
      return AUTO_DURATION_SECONDS;
    }
    double elapsed = Timer.getFPGATimestamp() - autoStartTime;
    return Math.max(0.0, AUTO_DURATION_SECONDS - elapsed);
  }

  public boolean isAutoRunning() {
    return DriverStation.isEnabled() && DriverStation.isAutonomous();
  }

  public static synchronized void launch(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      MagicCarpet magicCarpet,
      IntakeRollers intakeRollers,
      IntakeExtend intakeExtend) {
    if (GraphicsEnvironment.isHeadless()) {
      System.out.println("[SimDashboard] Headless environment detected, skipping GUI launch.");
      return;
    }
    if (instance == null) {
      SwingUtilities.invokeLater(
          () -> {
            instance =
                new SimDashboardWindow(
                    drive, shooter, indexer, magicCarpet, intakeRollers, intakeExtend);
            instance.setVisible(true);
          });
    } else {
      instance.updateReferences(drive, shooter, indexer, magicCarpet, intakeRollers, intakeExtend);
    }
  }

  public static SimDashboardWindow getInstance() {
    return instance;
  }

  private SimDashboardWindow(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      MagicCarpet magicCarpet,
      IntakeRollers intakeRollers,
      IntakeExtend intakeExtend) {
    super("FRC 467 - Robot Simulation & Learning Dashboard (2026)");
    this.drive = drive;
    this.shooter = shooter;
    this.indexer = indexer;
    this.magicCarpet = magicCarpet;
    this.intakeRollers = intakeRollers;
    this.intakeExtend = intakeExtend;

    initUI();
  }

  public void updateReferences(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      MagicCarpet magicCarpet,
      IntakeRollers intakeRollers,
      IntakeExtend intakeExtend) {
    this.drive = drive;
    this.shooter = shooter;
    this.indexer = indexer;
    this.magicCarpet = magicCarpet;
    this.intakeRollers = intakeRollers;
    this.intakeExtend = intakeExtend;
  }

  private void initUI() {
    setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE); // Keep running with simulator
    setSize(1180, 780);
    setMinimumSize(new Dimension(980, 640));
    setLocationRelativeTo(null);
    getContentPane().setBackground(new Color(24, 24, 28));

    setLayout(new BorderLayout(10, 10));

    // Top Header Banner
    JPanel headerPanel = createHeaderPanel();
    add(headerPanel, BorderLayout.NORTH);

    // Left Subsystems & Telemetry Panel
    telemetryPanel = new SubsystemsPanel();
    JScrollPane telemetryScroll = new JScrollPane(telemetryPanel);
    telemetryScroll.setBorder(null);
    telemetryScroll.setPreferredSize(new Dimension(340, 600));
    telemetryScroll.getVerticalScrollBar().setUnitIncrement(12);
    add(telemetryScroll, BorderLayout.WEST);

    // Center Graphical Area: Field View on top, Trajectory Arc + Robot Animation on bottom
    JPanel centerPanel = new JPanel(new BorderLayout(8, 8));
    centerPanel.setBackground(new Color(24, 24, 28));
    centerPanel.setBorder(new EmptyBorder(0, 0, 10, 10));

    fieldPanel = new FieldVisualizerPanel();
    trajectoryPanel = new TrajectoryVisualizerPanel();
    robotAnimPanel = new RobotAnimationPanel();

    centerPanel.add(fieldPanel, BorderLayout.CENTER);

    // Bottom row: trajectory on the left, robot animation on the right
    JPanel bottomRow = new JPanel(new GridLayout(1, 2, 8, 0));
    bottomRow.setBackground(new Color(24, 24, 28));
    bottomRow.setPreferredSize(new Dimension(0, 280));
    bottomRow.add(trajectoryPanel);
    bottomRow.add(robotAnimPanel);
    centerPanel.add(bottomRow, BorderLayout.SOUTH);

    add(centerPanel, BorderLayout.CENTER);

    // Repaint timer @ ~30 FPS
    javax.swing.Timer timer = new javax.swing.Timer(33, (ActionEvent e) -> repaint());
    timer.start();
  }

  private JPanel createHeaderPanel() {
    JPanel panel = new JPanel(new BorderLayout(15, 0));
    panel.setBackground(new Color(32, 33, 38));
    panel.setBorder(new EmptyBorder(10, 15, 10, 15));

    JLabel titleLabel = new JLabel("TEAM 467 ROBOT SIMULATION");
    titleLabel.setFont(new Font("SansSerif", Font.BOLD, 18));
    titleLabel.setForeground(new Color(240, 240, 245));

    JPanel titleBox = new JPanel(new GridLayout(1, 1));
    titleBox.setOpaque(false);
    titleBox.add(titleLabel);

    panel.add(titleBox, BorderLayout.WEST);

    // Right Status Badges & Controls (Auto Toggle, Match State, Mode)
    JPanel statusBox = new JPanel(new FlowLayout(FlowLayout.RIGHT, 10, 0));
    statusBox.setOpaque(false);

    // Alliance & Position Station Dropdown (Blue 1, 2, 3 / Red 1, 2, 3) - Default to Blue 3
    JLabel stationLabel = new JLabel("Station:");
    stationLabel.setFont(new Font("SansSerif", Font.PLAIN, 11));
    stationLabel.setForeground(new Color(170, 175, 185));

    String[] stationOptions = {"Blue 1", "Blue 2", "Blue 3", "Red 1", "Red 2", "Red 3"};
    JComboBox<String> stationCombo = new JComboBox<>(stationOptions);
    stationCombo.setFont(new Font("SansSerif", Font.BOLD, 11));
    stationCombo.setBackground(new Color(40, 44, 54));
    stationCombo.setForeground(Color.WHITE);
    stationCombo.setFocusable(false);
    stationCombo.setCursor(new Cursor(Cursor.HAND_CURSOR));

    // Default to Blue 3
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue3);
    DriverStationSim.notifyNewData();
    stationCombo.setSelectedItem("Blue 3");
    if (drive != null && !DriverStation.isEnabled()) {
      drive.setPose(AllianceFlipUtil.apply(new Pose2d(3.645, 2.0, new Rotation2d(0))));
    }

    stationCombo.addActionListener(
        e -> {
          String selected = (String) stationCombo.getSelectedItem();
          if (selected == null) return;
          AllianceStationID id =
              switch (selected) {
                case "Blue 1" -> AllianceStationID.Blue1;
                case "Blue 2" -> AllianceStationID.Blue2;
                case "Blue 3" -> AllianceStationID.Blue3;
                case "Red 1" -> AllianceStationID.Red1;
                case "Red 2" -> AllianceStationID.Red2;
                case "Red 3" -> AllianceStationID.Red3;
                default -> AllianceStationID.Blue3;
              };
          DriverStationSim.setAllianceStationId(id);
          DriverStationSim.notifyNewData();

          // If robot is disabled, immediately update starting position on field
          if (drive != null && !DriverStation.isEnabled()) {
            double y = 2.0;
            double x = 3.645;
            if (id == AllianceStationID.Red2 || id == AllianceStationID.Blue2) {
              y = FieldConstants.fieldWidth / 2.0;
              x = 2.0;
            } else if (id == AllianceStationID.Red1 || id == AllianceStationID.Blue1) {
              y = FieldConstants.fieldWidth - y;
            }
            drive.setPose(AllianceFlipUtil.apply(new Pose2d(x, y, new Rotation2d(0))));
          }
        });

    JButton autoBtn = new JButton("Start Auto");
    autoBtn.setFont(new Font("SansSerif", Font.BOLD, 12));
    autoBtn.setForeground(Color.WHITE);
    autoBtn.setBackground(new Color(35, 140, 60));
    autoBtn.setFocusPainted(false);
    autoBtn.setCursor(new Cursor(Cursor.HAND_CURSOR));
    autoBtn.setBorder(new EmptyBorder(5, 12, 5, 12));

    autoBtn.addActionListener(
        e -> {
          boolean isAuto = DriverStation.isEnabled() && DriverStation.isAutonomous();
          if (isAuto) {
            DriverStationSim.setAutonomous(false);
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            stopAllSubsystems();
          } else {
            ballSimulator.setBallsInRobot(8);
            ballSimulator.resetBallsScored();
            ballSimulator.resetFieldBalls();
            if (drive != null) {
              drive.stop();
            }
            autoStartTime = Timer.getFPGATimestamp();
            autoRunning = true;
            DriverStationSim.setAutonomous(true);
            DriverStationSim.setEnabled(true);
            DriverStationSim.notifyNewData();
          }
        });

    JLabel autoTimerBadge =
        new JLabel("AUTO: 20.0s") {
          @Override
          protected void paintComponent(Graphics g) {
            Graphics2D g2 = (Graphics2D) g.create();
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            boolean isAuto = isAutoRunning();
            double rem = getAutoRemainingSeconds();
            Color bg;
            if (isAuto) {
              bg = (rem <= 3.0) ? new Color(200, 45, 45) : new Color(210, 130, 20);
            } else {
              bg = new Color(50, 53, 62);
            }
            g2.setColor(bg);
            g2.fillRoundRect(0, 0, getWidth(), getHeight(), 10, 10);
            if (isAuto) {
              g2.setColor(Color.WHITE);
              g2.setStroke(new BasicStroke(1.2f));
              g2.drawRoundRect(0, 0, getWidth() - 1, getHeight() - 1, 10, 10);
            }
            g2.dispose();
            super.paintComponent(g);
          }
        };
    autoTimerBadge.setFont(new Font("Monospaced", Font.BOLD, 12));
    autoTimerBadge.setForeground(Color.WHITE);
    autoTimerBadge.setBorder(new EmptyBorder(4, 10, 4, 10));

    JLabel modeBadge =
        new JLabel() {
          @Override
          protected void paintComponent(Graphics g) {
            Graphics2D g2 = (Graphics2D) g.create();
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            boolean enabled = DriverStation.isEnabled();
            boolean auto = DriverStation.isAutonomous();
            Color bg =
                !enabled
                    ? new Color(80, 80, 85)
                    : (auto ? new Color(180, 130, 20) : new Color(30, 140, 60));
            g2.setColor(bg);
            g2.fillRoundRect(0, 0, getWidth(), getHeight(), 10, 10);
            g2.dispose();
            super.paintComponent(g);
          }
        };
    modeBadge.setFont(new Font("SansSerif", Font.BOLD, 12));
    modeBadge.setForeground(Color.WHITE);
    modeBadge.setBorder(new EmptyBorder(4, 10, 4, 10));

    // Dynamic timer updater for header & auto button
    boolean[] wasEnabledState = new boolean[] {false};
    new javax.swing.Timer(
            40,
            e -> {
              boolean enabled = DriverStation.isEnabled();
              boolean auto = DriverStation.isAutonomous();
              if (wasEnabledState[0] && !enabled) {
                stopAllSubsystems();
              }
              wasEnabledState[0] = enabled;

              boolean isAuto = enabled && auto;
              if (isAuto && !autoRunning) {
                autoRunning = true;
                autoStartTime = Timer.getFPGATimestamp();
              } else if (!isAuto && autoRunning) {
                autoRunning = false;
              }

              double remaining = getAutoRemainingSeconds();

              if (isAuto) {
                if (remaining <= 0.0) {
                  DriverStationSim.setAutonomous(false);
                  DriverStationSim.setEnabled(false);
                  DriverStationSim.notifyNewData();
                  stopAllSubsystems();
                  autoRunning = false;
                  autoTimerBadge.setText("AUTO: 0.0s");
                } else {
                  autoTimerBadge.setText(String.format("AUTO: %4.1fs", remaining));
                }
                autoBtn.setText("Stop Auto");
                autoBtn.setBackground(new Color(180, 45, 45));
                if (autoCardBtn != null) {
                  autoCardBtn.setText("Stop Auto");
                  autoCardBtn.setBackground(new Color(180, 45, 45));
                }
              } else {
                autoTimerBadge.setText("AUTO: 20.0s");
                autoBtn.setText("Start Auto");
                autoBtn.setBackground(new Color(35, 140, 60));
                if (autoCardBtn != null) {
                  autoCardBtn.setText("Start Auto");
                  autoCardBtn.setBackground(new Color(35, 140, 60));
                }
              }

              String modeText = !enabled ? "DISABLED" : (auto ? "AUTO" : "TELEOP");
              Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
              modeBadge.setText(alliance.toString().toUpperCase() + " | " + modeText);
            })
        .start();

    statusBox.add(stationLabel);
    statusBox.add(stationCombo);
    statusBox.add(autoBtn);
    statusBox.add(autoTimerBadge);
    statusBox.add(modeBadge);
    panel.add(statusBox, BorderLayout.EAST);

    return panel;
  }

  /** Immediately stops all commands, mechanisms, and subsystem motors when disabled */
  public void stopAllSubsystems() {
    autoRunning = false;
    CommandScheduler.getInstance().cancelAll();
    if (drive != null) {
      drive.stop();
    }
    if (shooter != null) {
      shooter.stopMotor();
    }
    if (intakeRollers != null) {
      intakeRollers.stopIntake();
    }
    if (intakeExtend != null) {
      intakeExtend.stopExtend();
    }
    if (indexer != null) {
      indexer.stopIndexer();
    }
    if (magicCarpet != null) {
      magicCarpet.manualRun = false;
    }
    RobotState state = RobotState.getInstance();
    state.intaking = false;
    state.indexerRunning = false;
    state.shooterAtSpeed = false;
    state.shooterSetpoint = 0.0;
  }

  /** Subsystems Telemetry & Ball Storage Side Panel */
  private class SubsystemsPanel extends JPanel {
    public SubsystemsPanel() {
      setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
      setBackground(new Color(28, 29, 34));
      setBorder(new EmptyBorder(12, 12, 12, 12));

      // 1. Ball Inventory Card
      add(createBallInventoryCard());
      add(Box.createVerticalStrut(10));

      // 2. Mechanism & Feeder Status Card (includes Drive, Shooter, Intake, Carpet, Indexer)
      add(createSubsystemsCard());
      add(Box.createVerticalStrut(10));

      // 3. Practice & Override Controls Card
      add(createStudentControlsCard());
    }

    private JPanel createBallInventoryCard() {
      JPanel card = new JPanel(new BorderLayout(8, 8));
      card.setBackground(new Color(38, 40, 48));
      card.setBorder(
          BorderFactory.createCompoundBorder(
              new LineBorder(new Color(55, 58, 68), 1, true), new EmptyBorder(10, 12, 10, 12)));

      JLabel title = new JLabel("BALL INVENTORY & SCORE");
      title.setFont(new Font("SansSerif", Font.BOLD, 13));
      title.setForeground(new Color(255, 180, 50));
      card.add(title, BorderLayout.NORTH);

      JPanel content =
          new JPanel() {
            @Override
            protected void paintComponent(Graphics g) {
              super.paintComponent(g);
              Graphics2D g2 = (Graphics2D) g.create();
              g2.setRenderingHint(
                  RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

              int balls = ballSimulator.getBallsInRobot();
              int scored = ballSimulator.getBallsScoredInHub();

              // Draw Scored Big Number
              g2.setColor(new Color(50, 205, 100));
              g2.setFont(new Font("SansSerif", Font.BOLD, 28));
              g2.drawString(String.valueOf(scored), 15, 38);

              g2.setColor(new Color(180, 185, 195));
              g2.setFont(new Font("SansSerif", Font.PLAIN, 11));
              g2.drawString("HUB GOALS SCORED", 15, 52);

              // Draw Magazine Hopper (2 rows of 10 for 20 balls)
              int startX = 140;
              int startY = 16;
              int radius = 10;
              int spacingX = 14;
              int spacingY = 14;

              g2.drawString(
                  "Hopper: " + balls + " / " + BallSimulator.MAX_CAPACITY, startX, startY - 2);
              for (int i = 0; i < BallSimulator.MAX_CAPACITY; i++) {
                int cx = startX + (i % 10) * spacingX;
                int cy = startY + 5 + (i / 10) * spacingY;
                if (i < balls) {
                  // Glowing orange ball
                  g2.setColor(new Color(255, 140, 20));
                  g2.fillOval(cx, cy, radius, radius);
                  g2.setColor(new Color(255, 200, 80));
                  g2.drawOval(cx, cy, radius, radius);
                } else {
                  // Empty slot
                  g2.setColor(new Color(60, 63, 72));
                  g2.drawOval(cx, cy, radius, radius);
                }
              }
              g2.dispose();
            }
          };
      content.setPreferredSize(new Dimension(300, 75));
      content.setOpaque(false);
      card.add(content, BorderLayout.CENTER);

      return card;
    }

    private JPanel createSubsystemsCard() {
      JPanel card = new JPanel(new BorderLayout(8, 8));
      card.setBackground(new Color(38, 40, 48));
      card.setBorder(
          BorderFactory.createCompoundBorder(
              new LineBorder(new Color(55, 58, 68), 1, true), new EmptyBorder(10, 12, 10, 12)));

      JLabel title = new JLabel("MECHANISM & FEEDER STATUS");
      title.setFont(new Font("SansSerif", Font.BOLD, 13));
      title.setForeground(new Color(160, 220, 90));
      card.add(title, BorderLayout.NORTH);

      JPanel content =
          new JPanel() {
            @Override
            protected void paintComponent(Graphics g) {
              super.paintComponent(g);
              Graphics2D g2 = (Graphics2D) g.create();
              g2.setRenderingHint(
                  RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

              int y = 14;
              // Column Headers
              g2.setFont(new Font("SansSerif", Font.PLAIN, 10));
              g2.setColor(new Color(140, 145, 155));
              g2.drawString("SUBSYSTEM", 8, y);
              g2.drawString("ACTIVE COMMAND", 95, y);
              g2.drawString("STATUS", 232, y);

              // 1. Drive
              boolean isEnabled = DriverStation.isEnabled();

              // 1. Drive
              String driveCmd =
                  (isEnabled && drive != null && drive.getCurrentCommand() != null)
                      ? drive.getCurrentCommand().getName()
                      : "None";
              double speed =
                  (isEnabled && drive != null)
                      ? Math.hypot(
                          drive.getChassisSpeeds().vxMetersPerSecond,
                          drive.getChassisSpeeds().vyMetersPerSecond)
                      : 0.0;
              String driveStatus = speed > 0.05 ? String.format("%.1f m/s", speed) : "STOPPED";
              y += 24;
              drawSubsystemRow(g2, "Drive", driveCmd, driveStatus, isEnabled && speed > 0.05, y);

              // 2. Shooter
              String shooterCmd =
                  (isEnabled && shooter != null && shooter.getCurrentCommand() != null)
                      ? shooter.getCurrentCommand().getName()
                      : "None";
              double actualRpm = (isEnabled && shooter != null) ? shooter.getVelocityRPM() : 0.0;
              double targetRpm =
                  (isEnabled && shooter != null)
                      ? (shooter.getSetpoint() * 60.0 / (2 * Math.PI))
                      : 0.0;
              boolean atSpeed = RobotState.getInstance().shooterAtSpeed;
              String shooterStatus =
                  targetRpm > 10.0
                      ? (atSpeed
                          ? String.format("%.0f RPM", actualRpm)
                          : String.format("%.0f RPM...", actualRpm))
                      : "IDLE";
              y += 24;
              drawSubsystemRow(
                  g2, "Shooter", shooterCmd, shooterStatus, isEnabled && actualRpm > 10.0, y);

              // 3. IntakeRollers
              String rollersCmd =
                  (isEnabled && intakeRollers != null && intakeRollers.getCurrentCommand() != null)
                      ? intakeRollers.getCurrentCommand().getName()
                      : "None";
              boolean intaking = isEnabled && RobotState.getInstance().intaking;
              y += 24;
              drawSubsystemRow(
                  g2, "IntakeRollers", rollersCmd, intaking ? "SPINNING" : "OFF", intaking, y);

              // 4. IntakeExtend
              String extendCmd =
                  (isEnabled && intakeExtend != null && intakeExtend.getCurrentCommand() != null)
                      ? intakeExtend.getCurrentCommand().getName()
                      : "None";
              boolean deployed =
                  isEnabled && RobotState.getInstance().intakePosition == IntakePosition.DEPLOYED;
              y += 24;
              drawSubsystemRow(
                  g2, "IntakeExtend", extendCmd, deployed ? "DEPLOYED" : "STOWED", deployed, y);

              // 5. MagicCarpet
              String carpetCmd =
                  (isEnabled && magicCarpet != null && magicCarpet.getCurrentCommand() != null)
                      ? magicCarpet.getCurrentCommand().getName()
                      : "None";
              boolean carpetRunning =
                  isEnabled
                      && ((magicCarpet != null && magicCarpet.manualRun)
                          || RobotState.getInstance().indexerRunning);
              y += 24;
              drawSubsystemRow(
                  g2,
                  "MagicCarpet",
                  carpetCmd,
                  carpetRunning ? "RUNNING" : "STOPPED",
                  carpetRunning,
                  y);

              // 6. Indexer
              String indexerCmd =
                  (isEnabled && indexer != null && indexer.getCurrentCommand() != null)
                      ? indexer.getCurrentCommand().getName()
                      : "None";
              boolean indexerRunning = isEnabled && RobotState.getInstance().indexerRunning;
              y += 24;
              drawSubsystemRow(
                  g2,
                  "Indexer",
                  indexerCmd,
                  indexerRunning ? "RUNNING" : "STOPPED",
                  indexerRunning,
                  y);

              g2.dispose();
            }

            private void drawSubsystemRow(
                Graphics2D g2,
                String subName,
                String cmdName,
                String status,
                boolean active,
                int y) {
              // Subsystem code name
              g2.setColor(new Color(220, 225, 235));
              g2.setFont(new Font("SansSerif", Font.BOLD, 11));
              g2.drawString(subName, 8, y);

              // Active command name
              boolean hasCmd = cmdName != null && !cmdName.equals("None");
              g2.setColor(hasCmd ? new Color(90, 190, 255) : new Color(110, 115, 125));
              g2.setFont(new Font("Monospaced", Font.PLAIN, 10));
              String displayCmd = cmdName != null ? cmdName : "None";
              if (displayCmd.length() > 16) {
                displayCmd = displayCmd.substring(0, 14) + "..";
              }
              g2.drawString(displayCmd, 95, y);

              // Status badge pill
              Color badgeColor = active ? new Color(35, 140, 70) : new Color(60, 63, 72);
              g2.setColor(badgeColor);
              int pillX = 218;
              int pillW = 74;
              int pillH = 17;
              g2.fillRoundRect(pillX, y - 12, pillW, pillH, 6, 6);

              g2.setColor(Color.WHITE);
              g2.setFont(new Font("SansSerif", Font.BOLD, 9));
              FontMetrics fm = g2.getFontMetrics();
              int tx = pillX + (pillW - fm.stringWidth(status)) / 2;
              g2.drawString(status, tx, y);
            }
          };
      content.setPreferredSize(new Dimension(300, 170));
      content.setOpaque(false);
      card.add(content, BorderLayout.CENTER);

      return card;
    }

    private JPanel createStudentControlsCard() {
      JPanel card = new JPanel(new BorderLayout(8, 8));
      card.setBackground(new Color(38, 40, 48));
      card.setBorder(
          BorderFactory.createCompoundBorder(
              new LineBorder(new Color(55, 58, 68), 1, true), new EmptyBorder(10, 12, 10, 12)));

      JLabel title = new JLabel("PRACTICE & OVERRIDE CONTROLS");
      title.setFont(new Font("SansSerif", Font.BOLD, 13));
      title.setForeground(new Color(220, 140, 240));
      card.add(title, BorderLayout.NORTH);

      JPanel btnGrid = new JPanel(new GridLayout(3, 2, 6, 6));
      btnGrid.setOpaque(false);

      JButton addBallBtn = new JButton("+ Add Ball");
      styleButton(addBallBtn, new Color(50, 120, 190));
      addBallBtn.addActionListener(e -> ballSimulator.addBall());

      JButton removeBallBtn = new JButton("- Remove Ball");
      styleButton(removeBallBtn, new Color(150, 60, 60));
      removeBallBtn.addActionListener(e -> ballSimulator.removeBall());

      JButton fillBallsBtn = new JButton("Fill Hopper (" + BallSimulator.MAX_CAPACITY + ")");
      styleButton(fillBallsBtn, new Color(180, 120, 30));
      fillBallsBtn.addActionListener(
          e -> ballSimulator.setBallsInRobot(BallSimulator.MAX_CAPACITY));

      JButton resetScoreBtn = new JButton("Reset Balls/Score");
      styleButton(resetScoreBtn, new Color(80, 85, 95));
      resetScoreBtn.addActionListener(
          e -> {
            ballSimulator.resetBallsScored();
            ballSimulator.resetFieldBalls();
          });

      autoCardBtn = new JButton("Start Auto");
      styleButton(autoCardBtn, new Color(35, 140, 60));
      autoCardBtn.addActionListener(
          e -> {
            boolean isAuto = DriverStation.isEnabled() && DriverStation.isAutonomous();
            if (isAuto) {
              DriverStationSim.setAutonomous(false);
              DriverStationSim.setEnabled(false);
              DriverStationSim.notifyNewData();
              stopAllSubsystems();
            } else {
              ballSimulator.setBallsInRobot(8);
              ballSimulator.resetBallsScored();
              ballSimulator.resetFieldBalls();
              if (drive != null) {
                drive.stop();
              }
              autoStartTime = Timer.getFPGATimestamp();
              autoRunning = true;
              DriverStationSim.setAutonomous(true);
              DriverStationSim.setEnabled(true);
              DriverStationSim.notifyNewData();
            }
          });

      JButton disableCardBtn = new JButton("Disable Robot");
      styleButton(disableCardBtn, new Color(150, 50, 50));
      disableCardBtn.addActionListener(
          e -> {
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            stopAllSubsystems();
          });

      btnGrid.add(addBallBtn);
      btnGrid.add(removeBallBtn);
      btnGrid.add(fillBallsBtn);
      btnGrid.add(resetScoreBtn);
      btnGrid.add(autoCardBtn);
      btnGrid.add(disableCardBtn);

      card.add(btnGrid, BorderLayout.CENTER);

      return card;
    }

    private void styleButton(JButton btn, Color bg) {
      btn.setBackground(bg);
      btn.setForeground(Color.WHITE);
      btn.setFocusPainted(false);
      btn.setFont(new Font("SansSerif", Font.BOLD, 11));
      btn.setBorder(new EmptyBorder(6, 8, 6, 8));
    }
  }

  /** 2D Field Top-Down Visualizer Panel */
  private class FieldVisualizerPanel extends JPanel {
    // Mouse position in screen pixels (null when outside panel)
    private Point mousePos = null;

    public FieldVisualizerPanel() {
      setBackground(new Color(20, 21, 25));
      setBorder(new LineBorder(new Color(45, 48, 56), 1, true));

      addMouseMotionListener(
          new java.awt.event.MouseMotionAdapter() {
            @Override
            public void mouseMoved(java.awt.event.MouseEvent e) {
              mousePos = e.getPoint();
            }

            @Override
            public void mouseDragged(java.awt.event.MouseEvent e) {
              mousePos = e.getPoint();
            }
          });

      addMouseListener(
          new java.awt.event.MouseAdapter() {
            @Override
            public void mouseExited(java.awt.event.MouseEvent e) {
              mousePos = null;
            }
          });
    }

    @Override
    protected void paintComponent(Graphics g) {
      super.paintComponent(g);
      Graphics2D g2 = (Graphics2D) g.create();
      g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

      int width = getWidth();
      int height = getHeight();

      // Field dimension in meters: 16.54m x 8.02m
      double fieldL = FieldConstants.fieldLength;
      double fieldW = FieldConstants.fieldWidth;

      double pad = 25.0;
      double scaleX = (width - 2 * pad) / fieldL;
      double scaleY = (height - 2 * pad) / fieldW;
      double scale = Math.min(scaleX, scaleY);

      double originX = pad + ((width - 2 * pad) - fieldL * scale) / 2.0;
      double originY = pad + ((height - 2 * pad) - fieldW * scale) / 2.0;

      // Coordinate converter helper
      // WPILib Field: (0,0) is Blue station right, X extends along length, Y extends along width
      // Screen: (originX + x*scale, originY + (fieldW - y)*scale)

      // 1. Draw Field Carpet & Zones
      g2.setColor(new Color(30, 32, 38));
      g2.fillRect((int) originX, (int) originY, (int) (fieldL * scale), (int) (fieldW * scale));

      // Neutral Zone highlight
      double nzNearX = originX + LinesVertical.neutralZoneNear * scale;
      double nzFarX = originX + LinesVertical.neutralZoneFar * scale;
      g2.setColor(new Color(42, 45, 54));
      g2.fillRect((int) nzNearX, (int) originY, (int) (nzFarX - nzNearX), (int) (fieldW * scale));

      // Field Border
      g2.setColor(new Color(80, 85, 95));
      g2.setStroke(new BasicStroke(2.0f));
      g2.drawRect((int) originX, (int) originY, (int) (fieldL * scale), (int) (fieldW * scale));

      // Zone Labels
      g2.setFont(new Font("SansSerif", Font.PLAIN, 10));
      g2.setColor(new Color(70, 130, 220, 140));
      g2.drawString("BLUE ALLIANCE ZONE", (int) originX + 15, (int) originY + 18);

      g2.setColor(new Color(220, 180, 60, 140));
      g2.drawString("NEUTRAL ZONE", (int) nzNearX + 15, (int) originY + 18);

      g2.setColor(new Color(220, 80, 80, 140));
      g2.drawString("RED ALLIANCE ZONE", (int) nzFarX + 15, (int) originY + 18);

      // 2. Draw Hubs
      drawHub(
          g2, Hub.blueCenter, new Color(40, 120, 240), originX, originY, fieldW, scale, "BLUE HUB");
      drawHub(
          g2, Hub.redCenter, new Color(230, 60, 60), originX, originY, fieldW, scale, "RED HUB");

      // 2.5 Draw Field Depot Corrals & Physical Field Balls
      g2.setStroke(new BasicStroke(1.5f));
      // Red Upper Depot (R1/R2)
      g2.setColor(new Color(220, 60, 60, 180));
      g2.drawRect(
          (int) originX,
          (int) (originY + (fieldW - 6.45) * scale),
          (int) (0.75 * scale),
          (int) (1.30 * scale));
      // Red Lower Depot (R3)
      g2.drawRect(
          (int) originX,
          (int) (originY + (fieldW - 1.40) * scale),
          (int) (0.55 * scale),
          (int) (1.10 * scale));

      // Blue Lower Depot (B1/B2)
      g2.setColor(new Color(50, 120, 230, 180));
      g2.drawRect(
          (int) (originX + (fieldL - 0.75) * scale),
          (int) (originY + (fieldW - 3.15) * scale),
          (int) (0.75 * scale),
          (int) (1.30 * scale));
      // Blue Upper Depot (B3)
      g2.drawRect(
          (int) (originX + (fieldL - 0.55) * scale),
          (int) (originY + (fieldW - 7.95) * scale),
          (int) (0.55 * scale),
          (int) (1.10 * scale));

      // Draw all active physical field balls on the carpet
      List<BallSimulator.FieldBall> fieldBalls = ballSimulator.getFieldBalls();
      synchronized (fieldBalls) {
        double rScreen = BallSimulator.FieldBall.RADIUS * scale;
        int dScreen = (int) Math.max(5, rScreen * 2.0);
        for (BallSimulator.FieldBall ball : fieldBalls) {
          if (!ball.inPlay) continue;
          double bx = originX + ball.x * scale;
          double by = originY + (fieldW - ball.y) * scale;

          // Yellow sphere with gold border
          g2.setColor(new Color(248, 220, 32));
          g2.fillOval((int) (bx - dScreen / 2), (int) (by - dScreen / 2), dScreen, dScreen);
          g2.setColor(new Color(180, 145, 12));
          g2.setStroke(new BasicStroke(1.0f));
          g2.drawOval((int) (bx - dScreen / 2), (int) (by - dScreen / 2), dScreen, dScreen);
        }
      }

      // 3. Draw Robot Pose & Aim
      if (drive != null) {
        Pose2d pose = drive.getPose();
        double rx = originX + pose.getX() * scale;
        double ry = originY + (fieldW - pose.getY()) * scale;

        // Aim Ray to Hub
        Translation2d shooterPos = ballSimulator.getShooterPosition(pose);
        Translation2d targetHub = ballSimulator.getTargetHubCenter();
        double currentRPM =
            (shooter != null && shooter.getSetpoint() > 0)
                ? (shooter.getSetpoint() * 60 / (2 * Math.PI))
                : 1000.0;

        BallSimulator.TrajectoryPrediction prediction =
            ballSimulator.predictTrajectory(pose, currentRPM);

        double sx = originX + shooterPos.getX() * scale;
        double sy = originY + (fieldW - shooterPos.getY()) * scale;
        double tx = originX + targetHub.getX() * scale;
        double ty = originY + (fieldW - targetHub.getY()) * scale;

        // Line to Hub
        g2.setColor(
            prediction.willHit() ? new Color(50, 220, 100, 180) : new Color(255, 100, 100, 140));
        g2.setStroke(
            new BasicStroke(
                1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0, new float[] {4, 4}, 0));
        g2.draw(new Line2D.Double(sx, sy, tx, ty));

        // Landing prediction spot
        double lx = originX + prediction.landingPos().getX() * scale;
        double ly = originY + (fieldW - prediction.landingPos().getY()) * scale;
        g2.setColor(prediction.willHit() ? new Color(50, 255, 120) : new Color(255, 70, 70));
        g2.fill(new Ellipse2D.Double(lx - 5, ly - 5, 10, 10));

        // Draw Robot Bumper Square (0.8m x 0.8m)
        double rSize = 0.85 * scale;
        AffineTransform old = g2.getTransform();
        g2.translate(rx, ry);
        g2.rotate(-pose.getRotation().getRadians());

        // Robot Body — color by alliance
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        boolean isRed = alliance == Alliance.Red;
        g2.setColor(isRed ? new Color(80, 30, 30) : new Color(30, 40, 80));
        g2.fill(new Rectangle2D.Double(-rSize / 2, -rSize / 2, rSize, rSize));
        g2.setColor(isRed ? new Color(220, 60, 60) : new Color(120, 180, 255));
        g2.setStroke(new BasicStroke(2.0f));
        g2.draw(new Rectangle2D.Double(-rSize / 2, -rSize / 2, rSize, rSize));

        // Intake visual indicator on front bumper (+X)
        boolean isDeployed = RobotState.getInstance().intakePosition == IntakePosition.DEPLOYED;
        if (isDeployed) {
          boolean isIntaking = DriverStation.isEnabled() && RobotState.getInstance().intaking;
          g2.setColor(isIntaking ? new Color(50, 240, 100) : new Color(220, 200, 60));
          double intakeExt = 0.22 * scale;
          double rollerW = 0.70 * scale;
          g2.fill(new RoundRectangle2D.Double(rSize / 2, -rollerW / 2, intakeExt, rollerW, 4, 4));
          g2.setColor(Color.WHITE);
          g2.setStroke(new BasicStroke(1.2f));
          g2.draw(new RoundRectangle2D.Double(rSize / 2, -rollerW / 2, intakeExt, rollerW, 4, 4));
        }

        // Heading Indicator Arrow (Pointing in forward X)
        g2.setColor(new Color(255, 200, 50));
        g2.drawLine(0, 0, (int) (rSize * 0.65), 0);
        g2.fillPolygon(
            new int[] {(int) (rSize * 0.65), (int) (rSize * 0.45), (int) (rSize * 0.45)},
            new int[] {0, -5, 5},
            3);

        g2.setTransform(old);
      }

      // 4. Draw Active In-Flight Balls
      double now = Timer.getFPGATimestamp();
      List<BallSimulator.FlyingBall> flyingBalls = ballSimulator.getActiveFlyingBalls();
      synchronized (flyingBalls) {
        for (BallSimulator.FlyingBall ball : flyingBalls) {
          double elapsed = now - ball.startTime;
          if (ball.isHit && elapsed >= ball.flightDuration) {
            continue; // Disappear when landing in the hub
          }
          double[] pos = ball.getPositionAtTime(now);
          double bx = originX + pos[0] * scale;
          double by = originY + (fieldW - pos[1]) * scale;
          double bz = pos[2];

          // Size scales with height z
          int bSize = (int) Math.max(6, 6 + bz * 4.0);
          g2.setColor(new Color(255, 160, 30));
          g2.fillOval((int) bx - bSize / 2, (int) by - bSize / 2, bSize, bSize);
          g2.setColor(Color.WHITE);
          g2.drawOval((int) bx - bSize / 2, (int) by - bSize / 2, bSize, bSize);
        }
      }

      // 5. Overlay: Robot Position & Mouse Field Position
      g2.setStroke(new BasicStroke(1.0f));
      g2.setFont(new Font("Monospaced", Font.PLAIN, 11));
      int overlayX = (int) originX + 5;
      int overlayY = (int) (originY + fieldW * scale) - 8;

      // Robot position label (bottom-left corner of field)
      if (drive != null) {
        Pose2d rPose = drive.getPose();
        String robotText =
            String.format(
                "Robot: (%.2f, %.2f) m  %.1f\u00b0",
                rPose.getX(), rPose.getY(), rPose.getRotation().getDegrees());
        g2.setColor(new Color(0, 0, 0, 160));
        g2.fillRoundRect(overlayX - 3, overlayY - 14, 265, 19, 5, 5);
        g2.setColor(new Color(120, 180, 255));
        g2.drawString(robotText, overlayX, overlayY);
        overlayY -= 22;
      }

      // Mouse field position label (above robot label) + dashed crosshair
      if (mousePos != null) {
        double mFieldX = (mousePos.x - originX) / scale;
        double mFieldY = fieldW - (mousePos.y - originY) / scale;
        mFieldX = Math.max(0, Math.min(fieldL, mFieldX));
        mFieldY = Math.max(0, Math.min(fieldW, mFieldY));
        String mouseText = String.format("Cursor: (%.2f, %.2f) m", mFieldX, mFieldY);
        g2.setColor(new Color(0, 0, 0, 160));
        g2.fillRoundRect(overlayX - 3, overlayY - 14, 200, 19, 5, 5);
        g2.setColor(new Color(220, 220, 100));
        g2.drawString(mouseText, overlayX, overlayY);

        // Dashed crosshair
        g2.setColor(new Color(220, 220, 100, 140));
        g2.setStroke(
            new BasicStroke(
                1.0f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0, new float[] {4, 4}, 0));
        int fieldRight = (int) (originX + fieldL * scale);
        int fieldBottom = (int) (originY + fieldW * scale);
        g2.drawLine(mousePos.x, (int) originY, mousePos.x, fieldBottom);
        g2.drawLine((int) originX, mousePos.y, fieldRight, mousePos.y);
      }

      g2.dispose();
    }

    private void drawHub(
        Graphics2D g2,
        Translation2d hubPos,
        Color color,
        double originX,
        double originY,
        double fieldW,
        double scale,
        String name) {
      double hx = originX + hubPos.getX() * scale;
      double hy = originY + (fieldW - hubPos.getY()) * scale;
      double r = (Hub.width / 2.0) * scale;

      // Outer rim
      g2.setColor(new Color(color.getRed(), color.getGreen(), color.getBlue(), 60));
      g2.fill(new Ellipse2D.Double(hx - r, hy - r, r * 2, r * 2));
      g2.setColor(color);
      g2.setStroke(new BasicStroke(2.0f));
      g2.draw(new Ellipse2D.Double(hx - r, hy - r, r * 2, r * 2));

      // Center mark
      g2.fill(new Ellipse2D.Double(hx - 3, hy - 3, 6, 6));

      // Backboard (visual wall facing the neutral zone to show shots cannot enter from neutral
      // zone)
      boolean isBlueHub = hubPos.getX() < FieldConstants.fieldLength / 2.0;
      double bbX = isBlueHub ? (hx + r) : (hx - r);
      double bbLen = r * 1.8;

      // Thick backboard barrier
      g2.setColor(new Color(250, 250, 255));
      g2.setStroke(new BasicStroke(5.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g2.drawLine((int) bbX, (int) (hy - bbLen / 2), (int) bbX, (int) (hy + bbLen / 2));

      // Backboard hazard/block outline
      g2.setColor(new Color(230, 60, 60));
      g2.setStroke(new BasicStroke(1.5f));
      int barX = isBlueHub ? (int) bbX - 1 : (int) bbX - 3;
      g2.drawRect(barX, (int) (hy - bbLen / 2), 4, (int) bbLen);

      g2.setFont(new Font("SansSerif", Font.BOLD, 10));
      g2.setColor(color);
      g2.drawString(name, (int) (hx - 25), (int) (hy - r - 4));
    }
  }

  /** Dynamic 2D Trajectory Arc Elevation Panel */
  private class TrajectoryVisualizerPanel extends JPanel {
    public TrajectoryVisualizerPanel() {
      setBackground(new Color(20, 21, 25));
      setBorder(new LineBorder(new Color(45, 48, 56), 1, true));
    }

    @Override
    protected void paintComponent(Graphics g) {
      super.paintComponent(g);
      Graphics2D g2 = (Graphics2D) g.create();
      g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

      int width = getWidth();
      int height = getHeight();

      double padLeft = 44.0;
      double padRight = 16.0;
      double padTop = 66.0;
      double padBottom = 26.0;

      double graphW = width - padLeft - padRight;
      double graphH = height - padTop - padBottom;

      // Distance range: 0 to 6.5 meters
      // Height range: 0 to 3.2 meters
      double maxDist = 6.5;
      double maxHeight = 3.2;

      double scaleX = graphW / maxDist;
      double scaleY = graphH / maxHeight;

      double originX = padLeft;
      double originY = height - padBottom;

      // 1. Draw Grid Lines & Axes
      g2.setColor(new Color(40, 43, 52));
      g2.setStroke(new BasicStroke(1.0f));

      for (double d = 1.0; d <= maxDist; d += 1.0) {
        int gx = (int) (originX + d * scaleX);
        g2.drawLine(gx, (int) (originY - graphH), gx, (int) originY);
        g2.setColor(new Color(140, 145, 155));
        g2.setFont(new Font("SansSerif", Font.PLAIN, 10));
        g2.drawString(String.format("%.0fm", d), gx - 8, (int) originY + 15);
        g2.setColor(new Color(40, 43, 52));
      }

      for (double h = 1.0; h <= maxHeight; h += 1.0) {
        int gy = (int) (originY - h * scaleY);
        g2.drawLine((int) originX, gy, (int) (originX + graphW), gy);
        g2.setColor(new Color(140, 145, 155));
        g2.setFont(new Font("SansSerif", Font.PLAIN, 10));
        g2.drawString(String.format("%.1fm", h), (int) originX - 32, gy + 4);
        g2.setColor(new Color(40, 43, 52));
      }

      // Axes lines
      g2.setColor(new Color(85, 90, 102));
      g2.setStroke(new BasicStroke(2.0f));
      g2.drawLine((int) originX, (int) originY, (int) (originX + graphW), (int) originY);
      g2.drawLine((int) originX, (int) originY, (int) originX, (int) (originY - graphH));

      // 2. Trajectory Calculation & Arc
      if (drive != null) {
        Pose2d pose = drive.getPose();
        double currentRPM =
            (shooter != null && shooter.getSetpoint() > 0)
                ? (shooter.getSetpoint() * 60 / (2 * Math.PI))
                : 1085.0;
        BallSimulator.TrajectoryPrediction pred = ballSimulator.predictTrajectory(pose, currentRPM);

        double distToHub = pred.distanceToHub();
        double v0h = pred.v0h();
        double v0z = pred.v0z();
        boolean willHit = pred.willHit();
        boolean inAllianceZone = ballSimulator.isInAllianceZone(pose);

        // Draw Hub Target Basket at Distance D and Height H (if within graph range)
        int hubScreenX = (int) (originX + distToHub * scaleX);
        int hubScreenY = (int) (originY - BallSimulator.HUB_HEIGHT_M * scaleY);
        int hubW = (int) (Hub.width * scaleX);

        if (distToHub <= maxDist + 0.2) {
          // Hub Basket Funnel Graphic
          g2.setColor(new Color(60, 140, 255, 90));
          g2.fillRect(hubScreenX - hubW / 2, hubScreenY, hubW, (int) (0.6 * scaleY));
          g2.setColor(new Color(80, 160, 255));
          g2.setStroke(new BasicStroke(2.0f));
          g2.drawRect(hubScreenX - hubW / 2, hubScreenY, hubW, (int) (0.6 * scaleY));

          // Hub Backboard Graphic on the far rim of the target basket
          int bbW = 5;
          int bbH = (int) (1.2 * scaleY);
          int bbX = hubScreenX + hubW / 2 - 2;
          int bbY = hubScreenY - bbH + (int) (0.5 * scaleY);
          g2.setColor(new Color(245, 245, 250));
          g2.fillRect(bbX, bbY, bbW, bbH);
          g2.setColor(new Color(230, 60, 60));
          g2.setStroke(new BasicStroke(1.5f));
          g2.drawRect(bbX, bbY, bbW, bbH);

          g2.setFont(new Font("SansSerif", Font.BOLD, 10));
          g2.setColor(new Color(80, 160, 255));
          g2.drawString("HUB OPENING", hubScreenX - 35, hubScreenY - 6);
        } else {
          // Hub is beyond the current graph distance range
          g2.setColor(new Color(80, 160, 255, 180));
          g2.setFont(new Font("SansSerif", Font.ITALIC, 10));
          g2.drawString(
              String.format("Hub Target: %.1fm \u2192", distToHub),
              (int) (originX + graphW - 95),
              hubScreenY - 6);
        }

        // Draw Parabolic Arc
        Path2D.Double arcPath = new Path2D.Double();
        arcPath.moveTo(originX, originY - BallSimulator.SHOOTER_HEIGHT_M * scaleY);

        double dt = 0.01;
        boolean arcValid = v0h > 0.5;
        if (arcValid) {
          for (double t = 0.0; t <= 1.8; t += dt) {
            double d = v0h * t;
            double z =
                BallSimulator.SHOOTER_HEIGHT_M + v0z * t - 0.5 * BallSimulator.GRAVITY * t * t;
            if (z < 0.0 || d > maxDist) break;

            double sx = originX + d * scaleX;
            double sy = originY - z * scaleY;
            arcPath.lineTo(sx, sy);
          }

          // Arc Color
          Color arcColor;
          if (!inAllianceZone) {
            arcColor = new Color(255, 75, 75); // Blocked by backboard
          } else if (willHit) {
            arcColor = new Color(50, 230, 110); // Will score
          } else {
            arcColor = new Color(255, 130, 60); // Miss
          }
          g2.setColor(arcColor);
          g2.setStroke(new BasicStroke(3.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
          g2.draw(arcPath);
        }

        // 3. Top Diagnostic HUD Card (Structured across 3 distinct rows so nothing overlaps)
        int hudX = 8;
        int hudY = 6;
        int hudW = width - 16;
        int hudH = 54;

        g2.setColor(new Color(26, 28, 35, 235));
        g2.fillRoundRect(hudX, hudY, hudW, hudH, 6, 6);
        g2.setColor(new Color(48, 52, 62));
        g2.setStroke(new BasicStroke(1.0f));
        g2.drawRoundRect(hudX, hudY, hudW, hudH, 6, 6);

        // Row 1: Trajectory Status
        g2.setFont(new Font("SansSerif", Font.BOLD, 11));
        String trajStatus;
        Color statusColor;
        if (!inAllianceZone) {
          trajStatus = "TRAJECTORY STATUS: BLOCKED";
          statusColor = new Color(255, 80, 80);
        } else if (willHit) {
          trajStatus = "TRAJECTORY STATUS: ON TARGET";
          statusColor = new Color(50, 230, 110);
        } else {
          trajStatus = "TRAJECTORY STATUS: OFF TARGET";
          statusColor = new Color(255, 140, 60);
        }
        g2.setColor(statusColor);
        g2.drawString(trajStatus, hudX + 8, hudY + 15);

        // Row 2: Target & Trajectory Metrics
        g2.setFont(new Font("SansSerif", Font.PLAIN, 10));
        g2.setColor(new Color(190, 195, 205));
        String metricsText =
            String.format(
                "Hub Dist: %.2fm   |   Angle Err: %.1f\u00b0   |   Flight Time: %.2fs",
                distToHub, Math.toDegrees(pred.angleErrorRad()), pred.timeToHub());
        g2.drawString(metricsText, hudX + 8, hudY + 30);

        // Row 3: Shooter Subsystem Indicators
        double actualRadPerSec = shooter != null ? shooter.getVelocityRadPerSec() : 0.0;
        double actualRPM = actualRadPerSec * 60.0 / (2 * Math.PI);
        double setpointRadPerSec = shooter != null ? shooter.getSetpoint() : 0.0;
        double setpointRPM = setpointRadPerSec * 60.0 / (2 * Math.PI);
        boolean atSpeed = RobotState.getInstance().shooterAtSpeed;

        int row3Y = hudY + 45;

        // Status indicator dot
        Color dotColor =
            (setpointRPM <= 10.0)
                ? new Color(110, 115, 125)
                : (atSpeed ? new Color(50, 220, 100) : new Color(255, 170, 30));
        g2.setColor(dotColor);
        g2.fillOval(hudX + 8, row3Y - 8, 8, 8);

        // Shooter state label
        g2.setColor(Color.WHITE);
        g2.setFont(new Font("SansSerif", Font.BOLD, 10));
        String shooterStateText =
            (setpointRPM <= 10.0)
                ? "SHOOTER: IDLE"
                : (atSpeed ? "SHOOTER: READY" : "SHOOTER: SPINNING UP...");
        g2.drawString(shooterStateText, hudX + 20, row3Y);

        // Setpoint readout: show actual RPM / target RPM
        g2.setFont(new Font("SansSerif", Font.PLAIN, 10));
        g2.setColor(new Color(170, 175, 185));
        int shooterTextWidth = g2.getFontMetrics().stringWidth(shooterStateText);
        int setpointX = hudX + 20 + shooterTextWidth + 10;
        String setpointText =
            (setpointRPM > 10.0)
                ? String.format("%.0f / %.0f RPM", actualRPM, setpointRPM)
                : String.format("Setpoint: %.0f RPM", setpointRPM);
        g2.drawString(setpointText, setpointX, row3Y);

        // Mini Speed Progress Bar
        int setpointTextWidth = g2.getFontMetrics().stringWidth(setpointText);
        int barX = setpointX + setpointTextWidth + 8;
        int barW = Math.max(30, hudX + hudW - barX - 8);
        int barH = 5;
        g2.setColor(new Color(42, 45, 54));
        g2.fillRoundRect(barX, row3Y - 6, barW, barH, 3, 3);

        double targetMax = Math.max(setpointRPM, 3500.0);
        double progress = Math.min(1.0, Math.max(0.0, actualRPM / targetMax));
        g2.setColor(atSpeed ? new Color(50, 205, 100) : new Color(255, 170, 30));
        g2.fillRoundRect(barX, row3Y - 6, (int) (barW * progress), barH, 3, 3);
      }

      g2.dispose();
    }
  }

  /** Animated 2D side-view cutaway of the robot showing mechanisms and ball flow */
  private class RobotAnimationPanel extends JPanel {
    private final java.util.ArrayList<double[]> shotAnims = new java.util.ArrayList<>();
    private int lastShotsAttempted = 0;

    public RobotAnimationPanel() {
      setBackground(new Color(20, 21, 25));
      setBorder(new LineBorder(new Color(45, 48, 56), 1, true));
    }

    @Override
    protected void paintComponent(Graphics g) {
      super.paintComponent(g);
      Graphics2D g2 = (Graphics2D) g.create();
      g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

      int w = getWidth();
      int h = getHeight();
      double now = Timer.getFPGATimestamp();

      // --- Read robot state ---
      boolean isEnabled = DriverStation.isEnabled();
      RobotState state = RobotState.getInstance();
      boolean intakeDeployed = isEnabled && state.intakePosition == IntakePosition.DEPLOYED;
      boolean intakeSpinning = isEnabled && state.intaking;
      boolean indexerRunning = isEnabled && state.indexerRunning;
      boolean carpetRunning =
          isEnabled && (indexerRunning || (magicCarpet != null && magicCarpet.manualRun));
      double shooterRPM = (isEnabled && shooter != null) ? shooter.getVelocityRPM() : 0.0;
      boolean shooterSpinning = isEnabled && shooterRPM > 10.0;
      int ballCount = ballSimulator.getBallsInRobot();

      Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
      boolean isRed = alliance == Alliance.Red;

      // --- Key geometry (all proportional to panel size) ---
      double margin = 12;
      double groundY = h * 0.92;

      // Chassis rectangle (the main robot body)
      double chassisL = w * 0.22;
      double chassisR = w * 0.82;
      double chassisT = h * 0.34;
      double chassisB = h * 0.68;
      double chassisW = chassisR - chassisL;
      double chassisH = chassisB - chassisT;

      // When intake deploys, the whole robot expands (front extends outward to the left)
      double frontExtension = intakeDeployed ? chassisW * 0.12 : 0;
      double effectiveChassisL = chassisL - frontExtension;

      // Bumper (colored band below chassis)
      double bumperH = h * 0.065;
      double bumperT = chassisB;
      double bumperB = bumperT + bumperH;

      // --- Ground line ---
      g2.setColor(new Color(55, 58, 68));
      g2.setStroke(new BasicStroke(2.0f));
      g2.drawLine((int) margin, (int) groundY, w - (int) margin, (int) groundY);

      // --- Drive wheels ---
      double wheelR = h * 0.042;
      double wheel1X = effectiveChassisL + chassisW * 0.20;
      double wheel2X = chassisR - chassisW * 0.15;
      double wheelY = bumperB + wheelR * 0.4;
      g2.setColor(new Color(42, 45, 52));
      g2.fill(new Ellipse2D.Double(wheel1X - wheelR, wheelY - wheelR, wheelR * 2, wheelR * 2));
      g2.fill(new Ellipse2D.Double(wheel2X - wheelR, wheelY - wheelR, wheelR * 2, wheelR * 2));
      g2.setColor(new Color(75, 80, 90));
      g2.setStroke(new BasicStroke(1.5f));
      g2.draw(new Ellipse2D.Double(wheel1X - wheelR, wheelY - wheelR, wheelR * 2, wheelR * 2));
      g2.draw(new Ellipse2D.Double(wheel2X - wheelR, wheelY - wheelR, wheelR * 2, wheelR * 2));

      // --- Chassis body background (cutaway view) ---
      g2.setColor(new Color(30, 33, 42));
      g2.fill(
          new Rectangle2D.Double(
              effectiveChassisL, chassisT, chassisR - effectiveChassisL, chassisH));

      // --- Bumper ---
      g2.setColor(isRed ? new Color(150, 25, 25) : new Color(25, 55, 150));
      g2.fill(
          new Rectangle2D.Double(
              effectiveChassisL - 3, bumperT, chassisR - effectiveChassisL + 6, bumperH));
      g2.setColor(isRed ? new Color(210, 55, 55) : new Color(65, 125, 230));
      g2.setStroke(new BasicStroke(2.0f));
      g2.draw(
          new Rectangle2D.Double(
              effectiveChassisL - 3, bumperT, chassisR - effectiveChassisL + 6, bumperH));

      // "467" label on bumper
      int fontSize = Math.max(9, (int) (bumperH * 0.72));
      g2.setFont(new Font("SansSerif", Font.BOLD, fontSize));
      g2.setColor(Color.WHITE);
      FontMetrics fm = g2.getFontMetrics();
      String teamLabel = "467";
      int labelW = fm.stringWidth(teamLabel);
      double bumperCenterX = effectiveChassisL + (chassisR - effectiveChassisL) / 2.0;
      g2.drawString(
          teamLabel, (int) (bumperCenterX - labelW / 2.0), (int) (bumperT + bumperH * 0.8));

      // --- Conveyor belt (magic carpet) inside chassis ---
      double conveyorY = chassisB - chassisH * 0.2;
      double conveyorL = effectiveChassisL + (intakeDeployed ? 12 : chassisW * 0.18);
      double conveyorR = chassisR - chassisW * 0.22;

      g2.setColor(new Color(52, 55, 65));
      g2.setStroke(new BasicStroke(3.0f));
      g2.draw(new Line2D.Double(conveyorL, conveyorY, conveyorR, conveyorY));

      // Animated magic carpet moving from LEFT TO RIGHT
      if (carpetRunning) {
        double speed = 55.0; // px/sec
        double spacing = 16.0;
        double offset = (now * speed) % spacing;
        g2.setColor(new Color(255, 170, 40));
        g2.setStroke(new BasicStroke(1.8f));
        for (double cx = conveyorL + offset; cx < conveyorR - 3; cx += spacing) {
          int x = (int) cx;
          int y = (int) (conveyorY + 3);
          g2.drawLine(x - 3, y - 3, x + 1, y);
          g2.drawLine(x - 3, y + 3, x + 1, y);
        }
      }

      // --- Indexer channel (vertical feed section) ---
      double indexerX = conveyorR + 3;
      double indexerW = chassisW * 0.1;
      double indexerTop = chassisT + chassisH * 0.06;
      double indexerBot = conveyorY;

      g2.setColor(new Color(42, 46, 56));
      g2.fill(new Rectangle2D.Double(indexerX, indexerTop, indexerW, indexerBot - indexerTop));
      g2.setColor(new Color(65, 70, 80));
      g2.setStroke(new BasicStroke(1.5f));
      g2.draw(new Rectangle2D.Double(indexerX, indexerTop, indexerW, indexerBot - indexerTop));

      if (indexerRunning) {
        // Upward-moving arrow indicators
        g2.setColor(new Color(85, 170, 255));
        g2.setStroke(new BasicStroke(1.5f));
        double arrowOff = (now * 45) % 18;
        double arrowCX = indexerX + indexerW / 2;
        for (double ay = indexerBot - arrowOff; ay > indexerTop + 6; ay -= 18) {
          int ax = (int) arrowCX;
          int ayI = (int) ay;
          g2.drawLine(ax, ayI, ax, ayI - 7);
          g2.drawLine(ax - 3, ayI - 4, ax, ayI - 7);
          g2.drawLine(ax + 3, ayI - 4, ax, ayI - 7);
        }
      }

      // --- Shooter wheels (at top of indexer exit) ---
      double shooterWheelR = h * 0.038;
      double exitX = indexerX + indexerW / 2;
      double exitY = indexerTop;

      // Two wheels flanking the exit channel
      double sw1cx = exitX - shooterWheelR * 0.9;
      double sw1cy = exitY - shooterWheelR * 0.5;
      double sw2cx = exitX + shooterWheelR * 0.9;
      double sw2cy = exitY - shooterWheelR * 0.5;

      // Shooter housing
      g2.setColor(new Color(45, 50, 60));
      double housingW = shooterWheelR * 3.8;
      double housingH = shooterWheelR * 2.2;
      g2.fill(
          new RoundRectangle2D.Double(
              exitX - housingW / 2, exitY - housingH - 3, housingW, housingH, 6, 6));
      g2.setColor(new Color(65, 70, 82));
      g2.setStroke(new BasicStroke(1.5f));
      g2.draw(
          new RoundRectangle2D.Double(
              exitX - housingW / 2, exitY - housingH - 3, housingW, housingH, 6, 6));

      // Draw each shooter wheel
      double[] swcxs = {sw1cx, sw2cx};
      double[] swcys = {sw1cy, sw2cy};
      for (int wi = 0; wi < 2; wi++) {
        double wcx = swcxs[wi];
        double wcy = swcys[wi];
        g2.setColor(new Color(55, 58, 68));
        g2.fill(
            new Ellipse2D.Double(
                wcx - shooterWheelR, wcy - shooterWheelR, shooterWheelR * 2, shooterWheelR * 2));

        if (shooterSpinning) {
          g2.setColor(new Color(255, 200, 70));
          g2.setStroke(new BasicStroke(2.0f));
          double spinRate = Math.min(shooterRPM / 60.0 * 2 * Math.PI * 0.08, 18);
          double spin = (now * spinRate) % (2 * Math.PI);
          for (int si = 0; si < 3; si++) {
            double a = spin + si * 2 * Math.PI / 3;
            g2.draw(
                new Line2D.Double(
                    wcx + shooterWheelR * 0.2 * Math.cos(a),
                    wcy + shooterWheelR * 0.2 * Math.sin(a),
                    wcx + shooterWheelR * 0.8 * Math.cos(a),
                    wcy + shooterWheelR * 0.8 * Math.sin(a)));
          }
        }

        g2.setColor(new Color(95, 100, 112));
        g2.setStroke(new BasicStroke(1.5f));
        g2.draw(
            new Ellipse2D.Double(
                wcx - shooterWheelR, wcy - shooterWheelR, shooterWheelR * 2, shooterWheelR * 2));
      }

      // --- Intake arm & rollers ---
      // When stowed, arm is completely inside the robot rectangle; when deployed, it lowers forward
      double intakeRollerR = h * 0.032;
      double intakePivotX;
      double intakePivotY;
      double armEndX;
      double armEndY;

      if (intakeDeployed) {
        // Deployed: arm swings out forward (left) and lowers to the carpet
        intakePivotX = effectiveChassisL + 8;
        intakePivotY = chassisB - chassisH * 0.15;
        armEndX = effectiveChassisL - chassisW * 0.12;
        armEndY = groundY - intakeRollerR - 2;
      } else {
        // Stowed: arm is folded completely INSIDE the robot rectangle
        intakePivotX = chassisL + chassisW * 0.06;
        intakePivotY = chassisB - chassisH * 0.15;
        armEndX = chassisL + chassisW * 0.11;
        armEndY = chassisT + chassisH * 0.38;
      }

      // Arm strut
      g2.setColor(new Color(95, 100, 110));
      g2.setStroke(new BasicStroke(3.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g2.draw(new Line2D.Double(intakePivotX, intakePivotY, armEndX, armEndY));

      // Roller circle at end of arm
      g2.setColor(new Color(55, 58, 65));
      g2.fill(
          new Ellipse2D.Double(
              armEndX - intakeRollerR,
              armEndY - intakeRollerR,
              intakeRollerR * 2,
              intakeRollerR * 2));

      if (intakeSpinning) {
        // Spinning indicator lines on roller
        g2.setColor(new Color(50, 220, 100));
        g2.setStroke(new BasicStroke(2.0f));
        double spin = (now * 10) % (2 * Math.PI);
        for (int i = 0; i < 4; i++) {
          double a = spin + i * Math.PI / 2;
          g2.draw(
              new Line2D.Double(
                  armEndX + intakeRollerR * 0.25 * Math.cos(a),
                  armEndY + intakeRollerR * 0.25 * Math.sin(a),
                  armEndX + intakeRollerR * 0.85 * Math.cos(a),
                  armEndY + intakeRollerR * 0.85 * Math.sin(a)));
        }
      }

      // Roller outline (alliance-colored)
      g2.setColor(isRed ? new Color(200, 55, 55) : new Color(100, 160, 240));
      g2.setStroke(new BasicStroke(2.0f));
      g2.draw(
          new Ellipse2D.Double(
              armEndX - intakeRollerR,
              armEndY - intakeRollerR,
              intakeRollerR * 2,
              intakeRollerR * 2));

      // --- Chassis body border (drawn over interior for clean cutaway outline) ---
      g2.setColor(isRed ? new Color(170, 45, 45) : new Color(70, 130, 210));
      g2.setStroke(new BasicStroke(2.5f));
      g2.draw(
          new Rectangle2D.Double(
              effectiveChassisL, chassisT, chassisR - effectiveChassisL, chassisH));

      // --- Balls inside the robot (supports up to 20 balls in 2 rows) ---
      double ballR = Math.min(chassisH * 0.13, (conveyorR - conveyorL) / 22.0);
      double ballBaseY = conveyorY - ballR - 2;
      double availableW = conveyorR - conveyorL - ballR * 2 - 4;
      int ballsPerRow = 10;

      for (int bi = 0; bi < ballCount && bi < BallSimulator.MAX_CAPACITY; bi++) {
        int col = bi % ballsPerRow;
        int row = bi / ballsPerRow; // 0 = bottom row, 1 = stacked row
        double spacing = availableW / (ballsPerRow - 1);
        double bx = conveyorL + ballR + 2 + col * spacing;
        double by = ballBaseY - row * (ballR * 1.8);

        // Bounce when magic carpet is active
        if (carpetRunning) {
          by -= Math.abs(Math.sin(now * 5.5 + bi * 1.3)) * ballR * 0.6;
        }

        // Yellow ball with highlight
        g2.setColor(new Color(215, 195, 35));
        g2.fill(new Ellipse2D.Double(bx - ballR, by - ballR, ballR * 2, ballR * 2));
        g2.setColor(new Color(255, 240, 70));
        g2.setStroke(new BasicStroke(1.5f));
        g2.draw(new Ellipse2D.Double(bx - ballR, by - ballR, ballR * 2, ballR * 2));
        // Small highlight dot
        g2.setColor(new Color(255, 255, 200, 160));
        double hlR = ballR * 0.25;
        g2.fill(
            new Ellipse2D.Double(
                bx - ballR * 0.3 - hlR, by - ballR * 0.35 - hlR, hlR * 2, hlR * 2));
      }

      // --- Shot ball animations (arc upward and to the LEFT) ---
      if (!isEnabled) {
        shotAnims.clear();
        lastShotsAttempted = ballSimulator.getTotalShotsAttempted();
      } else {
        int currentShots = ballSimulator.getTotalShotsAttempted();
        if (currentShots > lastShotsAttempted && lastShotsAttempted >= 0) {
          int newShots = Math.min(currentShots - lastShotsAttempted, 3);
          for (int ns = 0; ns < newShots; ns++) {
            // {startTime, exitScreenX, exitScreenY}
            shotAnims.add(new double[] {now + ns * 0.12, exitX, exitY - shooterWheelR * 1.5});
          }
        }
        lastShotsAttempted = currentShots;
      }

      // Draw shot balls flying upward and to the left
      java.util.Iterator<double[]> it = shotAnims.iterator();
      double launchAngle = Math.toRadians(68);
      double shotSpeed = h * 0.55;
      double shotGravity = h * 0.20;
      while (it.hasNext()) {
        double[] shot = it.next();
        double elapsed = now - shot[0];
        if (elapsed < 0) continue;
        if (elapsed > 1.4) {
          it.remove();
          continue;
        }
        // Clearly animates to the left (negative X)
        double sx = shot[1] - shotSpeed * Math.cos(launchAngle) * elapsed * 0.9;
        double sy =
            shot[2]
                - shotSpeed * Math.sin(launchAngle) * elapsed
                + 0.5 * shotGravity * elapsed * elapsed;

        int alpha = (int) Math.max(0, 255 * (1.0 - elapsed / 1.4));
        g2.setColor(new Color(215, 195, 35, alpha));
        g2.fill(new Ellipse2D.Double(sx - ballR, sy - ballR, ballR * 2, ballR * 2));
        g2.setColor(new Color(255, 240, 70, alpha));
        g2.setStroke(new BasicStroke(1.0f));
        g2.draw(new Ellipse2D.Double(sx - ballR, sy - ballR, ballR * 2, ballR * 2));
      }

      // --- Panel title ---
      g2.setFont(new Font("SansSerif", Font.BOLD, 11));
      g2.setColor(new Color(170, 175, 185));
      g2.drawString("ROBOT SIDE VIEW", (int) margin + 2, 16);

      // --- Status readout at bottom ---
      g2.setFont(new Font("Monospaced", Font.PLAIN, 10));
      g2.setColor(new Color(130, 135, 145));
      String statusLine =
          String.format(
              "Balls: %d/%d  Shooter: %.0f RPM  Scored: %d",
              ballCount,
              BallSimulator.MAX_CAPACITY,
              shooterRPM,
              ballSimulator.getBallsScoredInHub());
      g2.drawString(statusLine, (int) margin + 2, h - (int) margin);

      g2.dispose();
    }
  }
}
