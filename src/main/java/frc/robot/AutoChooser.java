package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.SwitchableChooser;
import frc.lib.utils.VirtualSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoChooser extends VirtualSubsystem {
  private static final int maxQuestions = 3;
  private static final AutoRoutine defaultRoutine =
      new AutoRoutine("Do Nothing", List.of(), Commands.none());

  private final LoggedDashboardChooser<AutoRoutine> routineChooser;
  private final List<StringPublisher> questionPublishers;
  private final List<SwitchableChooser> questionChoosers;

  private AutoRoutine lastRoutine;
  private List<AutoQuestionResponse> lastResponses;

  public AutoChooser(String key) {
    routineChooser = new LoggedDashboardChooser<>(key);
    routineChooser.addDefaultOption(defaultRoutine.name(), defaultRoutine);
    lastRoutine = defaultRoutine;
    lastResponses = List.of();

    // Publish questions and choosers
    questionPublishers = new ArrayList<>();
    questionChoosers = new ArrayList<>();
    for (int i = 0; i < maxQuestions; i++) {
      var publisher =
          NetworkTableInstance.getDefault()
              .getStringTopic("/SmartDashboard/" + key + "/Question #" + Integer.toString(i + 1))
              .publish();
      publisher.set("NA");
      questionPublishers.add(publisher);
      questionChoosers.add(
          new SwitchableChooser(key + "/Question #" + Integer.toString(i + 1) + " Chooser"));
    }
    System.out.println("AutoChooser Initialized");
  }

  /** Registers a new auto routine that can be selected. */
  public void addOption(String name, Command command) {
    addOption(name, List.of(), command);
  }

  public void addDefaultOption(String name, List<AutoQuestion> questions, Command command) {
    routineChooser.addDefaultOption(name, new AutoRoutine(name, questions, command));
  }

  public void addDefaultOption(String name, Command command) {
    routineChooser.addDefaultOption(name, new AutoRoutine(name, List.of(), command));
  }

  /** Registers a new auto routine that can be selected. */
  public void addOption(String name, List<AutoQuestion> questions, Command command) {
    if (questions.size() > maxQuestions) {
      throw new RuntimeException(
          "Auto routine contained more than "
              + Integer.toString(maxQuestions)
              + " questions: "
              + name);
    }
    routineChooser.addOption(name, new AutoRoutine(name, questions, command));
  }

  /** Returns the selected auto command. */
  public Command get() {
    System.out.println("Chosen Auto: " + getName());
    System.out.println("Auto Options: " + getResponses());
    return lastRoutine.command();
  }

  /** Returns the name of the selected routine. */
  public String getName() {
    return lastRoutine.name();
  }

  /** Returns the selected question responses. */
  public List<AutoQuestionResponse> getResponses() {
    return lastResponses;
  }

  @Override
  public void periodic() {
    // Skip updates when actively running in auto
    if (DriverStation.isAutonomousEnabled() && lastRoutine != null && lastResponses == null) {
      return;
    }

    // Update the list of questions
    var selectedRoutine = routineChooser.get();
    if (selectedRoutine == null) {
      return;
    }
    if (!selectedRoutine.equals(lastRoutine)) {
      var questions = selectedRoutine.questions();
      for (int i = 0; i < maxQuestions; i++) {
        if (i < questions.size()) {
          if (questions.get(i).conditionMet()) {
            questionPublishers.get(i).set(questions.get(i).question);
            questionChoosers
                .get(i)
                .setOptions(
                    questions.get(i).responses.stream().map(Enum::toString).toArray(String[]::new),
                    String.valueOf(questions.get(i).defaultOption));
          }
        } else {
          questionPublishers.get(i).set("");
          questionChoosers.get(i).setOptions(new String[] {});
        }
      }
    }

    // Update the routine and responses
    lastRoutine = selectedRoutine;
    lastResponses = new ArrayList<>();
    for (int i = 0; i < lastRoutine.questions().size(); i++) {
      String responseString = questionChoosers.get(i).get();
      lastResponses.add(
          responseString == null
              ? lastRoutine.questions().get(i).responses.get(0)
              : AutoQuestionResponse.valueOf(responseString));
    }
  }

  /** A customizable auto routine associated with a single command. */
  private static final record AutoRoutine(
      String name, List<AutoQuestion> questions, Command command) {}

  /** A question to ask for customizing an auto routine. */
  public static class AutoQuestion {
    private static class ConditionWrapper {
      private BooleanSupplier condition;
      private boolean conditionSet = false;
      private boolean defaultCondition = true;

      public void setCondition(BooleanSupplier condition) {
        this.condition = condition;
        conditionSet = true;
      }

      public boolean isConditionMet() {
        return conditionSet ? condition.getAsBoolean() : defaultCondition;
      }
    }

    private final String question;
    private final List<AutoQuestionResponse> responses;
    private final AutoQuestionResponse defaultOption;
    private final ConditionWrapper conditionWrapper = new ConditionWrapper();

    public AutoQuestion(
        String question, List<AutoQuestionResponse> responses, AutoQuestionResponse defaultOption) {
      this.question = question;
      this.responses = responses;
      this.defaultOption = defaultOption;
    }

    public AutoQuestion conditional(BooleanSupplier condition) {
      conditionWrapper.setCondition(condition);
      return this;
    }

    public boolean conditionMet() {
      System.out.println(conditionWrapper.isConditionMet());
      return conditionWrapper.isConditionMet();
    }
  }

  /** Responses to auto routine questions. */
  public static enum AutoQuestionResponse {
    LEFT,
    CENTER,
    RIGHT,
    YES,
    NO,
    AMP,
    STAGE
  }
}
