# FRC Robot Code Guidelines

## Project Structure

### Source Code Organization
- **src/main/java/frc/robot/** - Robot-specific code
  - **commands/** - Command classes for robot actions
  - **subsystems/** - Subsystem classes for robot components
  - **Robot.java** - Main robot class
  - **RobotContainer.java** - Subsystems and command bindings
  - **Constants.java** - Robot-wide constants
- **src/main/java/frc/lib/** - Reusable utilities and libraries
- **src/main/deploy/** - Files to be deployed to the robot

### Build System
- Gradle-based build system with GradleRIO plugin
- Java 17 is used for development

## Tech Stack

- **WPILib** - Core FRC library
- **AdvantageKit** - Data logging and replay framework
- **PathPlanner** - Path planning for autonomous navigation
- **ChoreoLib** - Choreographed robot movements
- **Phoenix6** - CTRE motor controller library
- **REVLib** - REV Robotics device library
- **PhotonLib** - Computer vision library

## Development Workflow

### Building and Deploying
- Build the project: `./gradlew build`
- Deploy to robot: `./gradlew deploy`
- Simulate the robot: `./gradlew simulateJava`, however this requires active user input and is likely not usable when running Junie

### Testing
- Run tests: `./gradlew test`
- Tests are not currently used in the code base
- SysId commands can be made for system identification (feedforward)

### Code Quality
- Code formatting is enforced using Spotless with Google Java Format
- Run formatter: `./gradlew spotlessApply`

## Best Practices

### Code Organization
- Follow the command-based programming paradigm
- Separate reusable code (frc.lib) from robot-specific code (frc.robot)
- Use constants for magic numbers and configuration values

### Subsystem Design
- Each subsystem should have a clear interface
- Use IO interfaces for hardware abstraction
- Implement simulation support where possible

### Command Design
- Commands should be small and focused
- Use command composition for complex behaviors
- Ensure proper requirement declaration to avoid conflicts

### Version Control
- Commits on event branches are automatically created during deployment
- Follow standard Git workflow for feature development

### Simulation
- Test code in simulation before deploying to the robot, if possible
- Use AdvantageKit for data logging and replay
