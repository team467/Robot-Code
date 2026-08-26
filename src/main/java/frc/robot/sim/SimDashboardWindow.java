package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
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

  // UI Panels
  private FieldVisualizerPanel fieldPanel;
  private TrajectoryVisualizerPanel trajectoryPanel;
  private SubsystemsPanel telemetryPanel;

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

    // Center Graphical Area: Split Field View & Trajectory Arc
    JPanel centerPanel = new JPanel(new GridLayout(2, 1, 8, 8));
    centerPanel.setBackground(new Color(24, 24, 28));
    centerPanel.setBorder(new EmptyBorder(0, 0, 10, 10));

    fieldPanel = new FieldVisualizerPanel();
    trajectoryPanel = new TrajectoryVisualizerPanel();

    centerPanel.add(fieldPanel);
    centerPanel.add(trajectoryPanel);

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

    JLabel subLabel = new JLabel("Interactive Learning & Subsystem Telemetry");
    subLabel.setFont(new Font("SansSerif", Font.PLAIN, 12));
    subLabel.setForeground(new Color(160, 165, 175));

    JPanel titleBox = new JPanel(new GridLayout(2, 1));
    titleBox.setOpaque(false);
    titleBox.add(titleLabel);
    titleBox.add(subLabel);

    panel.add(titleBox, BorderLayout.WEST);

    // Right Status Badges (Match State, Mode, Time)
    JPanel statusBox = new JPanel(new FlowLayout(FlowLayout.RIGHT, 10, 0));
    statusBox.setOpaque(false);

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

    // Dynamic timer updater for header
    new javax.swing.Timer(
            100,
            e -> {
              boolean enabled = DriverStation.isEnabled();
              boolean auto = DriverStation.isAutonomous();
              String modeText = !enabled ? "DISABLED" : (auto ? "AUTO" : "TELEOP");
              Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
              modeBadge.setText(alliance.toString().toUpperCase() + " | " + modeText);
            })
        .start();

    statusBox.add(modeBadge);
    panel.add(statusBox, BorderLayout.EAST);

    return panel;
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

      // 2. Shooter Diagnostics Card
      add(createShooterCard());
      add(Box.createVerticalStrut(10));

      // 3. Feeder & Subsystems Status Card
      add(createSubsystemsCard());
      add(Box.createVerticalStrut(10));

      // 4. Interactive Learning Controls Card
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

              // Draw Magazine Hopper
              int startX = 160;
              int startY = 18;
              int radius = 18;
              int spacing = 22;

              g2.drawString(
                  "Hopper: " + balls + " / " + BallSimulator.MAX_CAPACITY, startX, startY - 4);
              for (int i = 0; i < BallSimulator.MAX_CAPACITY; i++) {
                int cx = startX + (i % 4) * spacing;
                int cy = startY + 6 + (i / 4) * spacing;
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

    private JPanel createShooterCard() {
      JPanel card = new JPanel(new BorderLayout(8, 8));
      card.setBackground(new Color(38, 40, 48));
      card.setBorder(
          BorderFactory.createCompoundBorder(
              new LineBorder(new Color(55, 58, 68), 1, true), new EmptyBorder(10, 12, 10, 12)));

      JLabel title = new JLabel("SHOOTER SUBSYSTEM");
      title.setFont(new Font("SansSerif", Font.BOLD, 13));
      title.setForeground(new Color(90, 175, 255));
      card.add(title, BorderLayout.NORTH);

      JPanel content =
          new JPanel() {
            @Override
            protected void paintComponent(Graphics g) {
              super.paintComponent(g);
              Graphics2D g2 = (Graphics2D) g.create();
              g2.setRenderingHint(
                  RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

              double setpointRadPerSec = shooter != null ? shooter.getSetpoint() : 0.0;
              double setpointRPM = setpointRadPerSec * 60.0 / (2 * Math.PI);
              boolean atSpeed = RobotState.getInstance().shooterAtSpeed;

              // Status indicator dot
              Color dotColor =
                  (setpointRPM <= 10.0)
                      ? new Color(90, 95, 105)
                      : (atSpeed ? new Color(50, 220, 100) : new Color(255, 170, 30));
              g2.setColor(dotColor);
              g2.fillOval(10, 10, 14, 14);

              g2.setColor(Color.WHITE);
              g2.setFont(new Font("SansSerif", Font.BOLD, 13));
              String statusText =
                  (setpointRPM <= 10.0)
                      ? "IDLE"
                      : (atSpeed ? "READY / AT SPEED" : "SPINNING UP...");
              g2.drawString(statusText, 32, 22);

              // RPM Readout
              g2.setColor(new Color(200, 205, 215));
              g2.setFont(new Font("SansSerif", Font.PLAIN, 12));
              g2.drawString(String.format("Target Setpoint:  %.0f RPM", setpointRPM), 10, 48);
              g2.drawString(String.format("Velocity:  %.1f rad/s", setpointRadPerSec), 10, 68);

              // Mini Speed Progress Bar
              int barX = 10;
              int barY = 78;
              int barW = 270;
              int barH = 10;
              g2.setColor(new Color(50, 53, 62));
              g2.fillRoundRect(barX, barY, barW, barH, 6, 6);

              double progress = Math.min(1.0, setpointRPM / 4500.0);
              g2.setColor(atSpeed ? new Color(50, 205, 100) : new Color(90, 175, 255));
              g2.fillRoundRect(barX, barY, (int) (barW * progress), barH, 6, 6);

              g2.dispose();
            }
          };
      content.setPreferredSize(new Dimension(300, 95));
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

              boolean carpetOn =
                  RobotState.getInstance().indexerRunning; // Magic carpet follows indexer
              boolean indexerOn = RobotState.getInstance().indexerRunning;
              boolean intaking = RobotState.getInstance().intaking;
              boolean intakeDeployed =
                  RobotState.getInstance().intakePosition == IntakePosition.DEPLOYED;

              int y = 18;
              drawStatusRow(
                  g2, "Magic Carpet Conveyor:", carpetOn ? "RUNNING" : "STOPPED", carpetOn, y);
              y += 24;
              drawStatusRow(
                  g2, "Indexer Feed-Up:", indexerOn ? "RUNNING" : "STOPPED", indexerOn, y);
              y += 24;
              drawStatusRow(g2, "Intake Rollers:", intaking ? "SPINNING" : "OFF", intaking, y);
              y += 24;
              drawStatusRow(
                  g2,
                  "Intake Position:",
                  intakeDeployed ? "DEPLOYED" : "STOWED",
                  intakeDeployed,
                  y);

              g2.dispose();
            }

            private void drawStatusRow(
                Graphics2D g2, String label, String state, boolean active, int y) {
              g2.setColor(new Color(180, 185, 195));
              g2.setFont(new Font("SansSerif", Font.PLAIN, 12));
              g2.drawString(label, 10, y);

              Color badgeColor = active ? new Color(40, 160, 80) : new Color(75, 78, 88);
              g2.setColor(badgeColor);
              g2.fillRoundRect(190, y - 13, 90, 18, 8, 8);

              g2.setColor(Color.WHITE);
              g2.setFont(new Font("SansSerif", Font.BOLD, 10));
              FontMetrics fm = g2.getFontMetrics();
              int tx = 190 + (90 - fm.stringWidth(state)) / 2;
              g2.drawString(state, tx, y);
            }
          };
      content.setPreferredSize(new Dimension(300, 105));
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

      JPanel btnGrid = new JPanel(new GridLayout(2, 2, 6, 6));
      btnGrid.setOpaque(false);

      JButton addBallBtn = new JButton("+ Add Ball");
      styleButton(addBallBtn, new Color(50, 120, 190));
      addBallBtn.addActionListener(e -> ballSimulator.addBall());

      JButton removeBallBtn = new JButton("- Remove Ball");
      styleButton(removeBallBtn, new Color(150, 60, 60));
      removeBallBtn.addActionListener(e -> ballSimulator.removeBall());

      JButton fillBallsBtn = new JButton("Fill Hopper (8)");
      styleButton(fillBallsBtn, new Color(180, 120, 30));
      fillBallsBtn.addActionListener(
          e -> ballSimulator.setBallsInRobot(BallSimulator.MAX_CAPACITY));

      JButton resetScoreBtn = new JButton("Reset Score");
      styleButton(resetScoreBtn, new Color(80, 85, 95));
      resetScoreBtn.addActionListener(e -> ballSimulator.resetBallsScored());

      btnGrid.add(addBallBtn);
      btnGrid.add(removeBallBtn);
      btnGrid.add(fillBallsBtn);
      btnGrid.add(resetScoreBtn);

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
    public FieldVisualizerPanel() {
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
      g2.drawString("BLUE ALLIANCE ZONE (Low Intake %)", (int) originX + 15, (int) originY + 18);

      g2.setColor(new Color(220, 180, 60, 140));
      g2.drawString("NEUTRAL ZONE (High Intake %)", (int) nzNearX + 15, (int) originY + 18);

      g2.setColor(new Color(220, 80, 80, 140));
      g2.drawString("RED ALLIANCE ZONE (Low Intake %)", (int) nzFarX + 15, (int) originY + 18);

      // 2. Draw Hubs
      drawHub(
          g2, Hub.blueCenter, new Color(40, 120, 240), originX, originY, fieldW, scale, "BLUE HUB");
      drawHub(
          g2, Hub.redCenter, new Color(230, 60, 60), originX, originY, fieldW, scale, "RED HUB");

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

        // Robot Body
        g2.setColor(new Color(50, 55, 68));
        g2.fill(new Rectangle2D.Double(-rSize / 2, -rSize / 2, rSize, rSize));
        g2.setColor(new Color(120, 180, 255));
        g2.setStroke(new BasicStroke(2.0f));
        g2.draw(new Rectangle2D.Double(-rSize / 2, -rSize / 2, rSize, rSize));

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

      double padX = 60.0;
      double padY = 40.0;
      double graphW = width - 2 * padX;
      double graphH = height - 2 * padY;

      // Distance range: 0 to 7 meters
      // Height range: 0 to 3.5 meters
      double maxDist = 6.5;
      double maxHeight = 3.2;

      double scaleX = graphW / maxDist;
      double scaleY = graphH / maxHeight;

      double originX = padX;
      double originY = height - padY;

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

        // Draw Hub Target Basket at Distance D and Height H
        int hubScreenX = (int) (originX + distToHub * scaleX);
        int hubScreenY = (int) (originY - BallSimulator.HUB_HEIGHT_M * scaleY);

        // Hub Basket Funnel Graphic
        int hubW = (int) (Hub.width * scaleX);
        g2.setColor(new Color(60, 140, 255, 90));
        g2.fillRect(hubScreenX - hubW / 2, hubScreenY, hubW, (int) (0.6 * scaleY));
        g2.setColor(new Color(80, 160, 255));
        g2.setStroke(new BasicStroke(2.0f));
        g2.drawRect(hubScreenX - hubW / 2, hubScreenY, hubW, (int) (0.6 * scaleY));

        g2.setFont(new Font("SansSerif", Font.BOLD, 10));
        g2.drawString("HUB OPENING", hubScreenX - 35, hubScreenY - 6);

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

          // Arc Color (Green if scores in Hub, Orange/Red if misses)
          Color arcColor = willHit ? new Color(50, 230, 110) : new Color(255, 90, 70);
          g2.setColor(arcColor);
          g2.setStroke(new BasicStroke(3.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
          g2.draw(arcPath);
        }

        // Diagnostic HUD Overlays
        g2.setFont(new Font("SansSerif", Font.BOLD, 13));
        String trajStatus =
            willHit
                ? "TRAJECTORY STATUS: ON TARGET (WILL SCORE)"
                : "TRAJECTORY STATUS: OFF TARGET (TOO WEAK / OVER / MISALIGNED)";
        g2.setColor(willHit ? new Color(50, 230, 110) : new Color(255, 90, 70));
        g2.drawString(trajStatus, (int) originX + 20, (int) (originY - graphH) + 20);

        g2.setFont(new Font("SansSerif", Font.PLAIN, 11));
        g2.setColor(new Color(200, 205, 215));
        g2.drawString(
            String.format(
                "Hub Distance: %.2f m   |   Shooter Speed: %.0f RPM   |   Angle Error: %.1f deg",
                distToHub, currentRPM, Math.toDegrees(pred.angleErrorRad())),
            (int) originX + 20,
            (int) (originY - graphH) + 38);
      }

      g2.dispose();
    }
  }
}
