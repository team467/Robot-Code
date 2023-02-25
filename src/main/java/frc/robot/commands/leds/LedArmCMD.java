// package frc.robot.commands.leds;

// import frc.robot.subsystems.arm.Arm;
// import frc.robot.subsystems.intakerelease.IntakeRelease;
// import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;
// import frc.robot.subsystems.led.Led2023;
// import frc.robot.subsystems.led.Led2023.COLORS_467;

// public class LedArmCMD extends LedBaseCMD {
//   private Arm arm;
//   private IntakeRelease intakerelease;

//   public LedArmCMD(Led2023 ledStrip, Arm arm) {
//     super(ledStrip);
//     this.arm = arm;
//     addRequirements(arm);
//   }

//   @Override
//   public void execute() {
//     if (extend||up) {
//       if (intakerelease.getWants()==Wants.CUBE||intakerelease.haveCube()) {
//       ledStrip.setAlternateColorsUp(COLORS_467.Purple, COLORS_467.White,
// COLORS_467.Black.getColor());
//       } else {
//       ledStrip.setAlternateColorsUp(COLORS_467.Gold, COLORS_467.White,
// COLORS_467.Black.getColor());
//       }
//     } else if (retract||down) {
//       if (intakerelease.getWants()==Wants.CUBE||intakerelease.haveCube()) {
//       ledStrip.setAlternateColorsDown(COLORS_467.Purple, COLORS_467.White,
// COLORS_467.Black.getColor());
//       } else {
//       ledStrip.setAlternateColorsDown(COLORS_467.Gold, COLORS_467.White,
// COLORS_467.Black.getColor());
//       }
//     }
//   }
// }
