package frc.robot.logging;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Arrays;

import org.apache.logging.log4j.Level;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.core.LoggerContext;
import org.apache.logging.log4j.core.appender.RollingRandomAccessFileAppender;
import org.apache.logging.log4j.core.appender.RollingRandomAccessFileAppender.Builder;
import org.apache.logging.log4j.core.appender.rolling.CompositeTriggeringPolicy;
import org.apache.logging.log4j.core.appender.rolling.DefaultRolloverStrategy;
import org.apache.logging.log4j.core.appender.rolling.OnStartupTriggeringPolicy;
import org.apache.logging.log4j.core.appender.rolling.SizeBasedTriggeringPolicy;
import org.apache.logging.log4j.core.appender.rolling.TriggeringPolicy;
import org.apache.logging.log4j.core.config.Configuration;
import org.apache.logging.log4j.core.config.ConfigurationFactory;
import org.apache.logging.log4j.core.config.ConfigurationSource;
import org.apache.logging.log4j.core.config.Configurator;
import org.apache.logging.log4j.core.config.yaml.YamlConfigurationFactory;
import org.apache.logging.log4j.core.layout.PatternLayout;

public class RobotLogManager {

  private static boolean initialized = false;

  private static File directory;

  private static String[] filepaths = { //File paths go in this array
      "/media/sda1/logging/log4j2.yaml",
      "/media/sda2/logging/log4j2.yaml",
      "/media/sdb1/logging/log4j2.yaml",
      "/media/sdb2/logging/log4j2.yaml",
      //"/home/lvuser/deploy/log4j2.yaml",
      //"./src/main/deploy/log4j2.yaml",
  };
  
  public static File getDirectory() {
    return directory;
  }

  private static boolean doesFileExist(String filepath) {
    File file = new File(filepath);
    if (file.exists()) {
      directory = new File(file.getParent(), "logs");
      if (!directory.exists()) {
        directory.mkdirs();
      }
      return true;
    } else {
      return false;
    }
  }

  private static void init(String pathToConfig) {
    if (!initialized) {
      try {
        File configSourceFile = new File(pathToConfig);
        ConfigurationSource source = new ConfigurationSource(
              new FileInputStream(configSourceFile), configSourceFile);
        ConfigurationFactory configurationFactory = YamlConfigurationFactory.getInstance();
        ConfigurationFactory.setConfigurationFactory(configurationFactory);
        Configuration configuration = configurationFactory.getConfiguration(null, source);
        Builder<?> appenderBuilder = RollingRandomAccessFileAppender.newBuilder();
        appenderBuilder.setName("Async_File_Appender");
        appenderBuilder.withFileName(pathToConfig.replace("log4j2.yaml", "logs") + "/app.log");
        appenderBuilder.withFilePattern(pathToConfig.replace("log4j2.yaml", "logs") + "/app-%d{yyyy-MM-dd-HH-mm-ss-SSS}.log");
        appenderBuilder.withImmediateFlush(true);
        appenderBuilder.withAppend(true);
        TriggeringPolicy onStartupTriggeringPolicy = OnStartupTriggeringPolicy.createPolicy(1);
        TriggeringPolicy sizeBasedTriggeringPolicy = SizeBasedTriggeringPolicy.createPolicy("10 MB");
        appenderBuilder.withPolicy(CompositeTriggeringPolicy.createPolicy(onStartupTriggeringPolicy, sizeBasedTriggeringPolicy));
        org.apache.logging.log4j.core.appender.rolling.DefaultRolloverStrategy.Builder strategyBuilder = DefaultRolloverStrategy.newBuilder();
        strategyBuilder.withMax("50");
        appenderBuilder.withStrategy(strategyBuilder.build());
        org.apache.logging.log4j.core.layout.PatternLayout.Builder patternBuilder = PatternLayout.newBuilder();
        patternBuilder.withPattern("[%-5level] %d{yyyy-MM-dd HH:mm:ss.SSS} [%t] %c{1} - %msg%n");
        appenderBuilder.setLayout(patternBuilder.build());
        RollingRandomAccessFileAppender rollingRandomAccessFileAppender = appenderBuilder.build();
        rollingRandomAccessFileAppender.start();
        configuration.addAppender(rollingRandomAccessFileAppender);
        Configurator.initialize(configuration);
        configuration.getRootLogger().addAppender(rollingRandomAccessFileAppender, Level.INFO, null);
        configuration.getLoggerConfig("frc.robot.Robot").addAppender(rollingRandomAccessFileAppender, Level.INFO, null);
        System.out.println(Arrays.toString(configuration.getAppenders().keySet().toArray()));
      } catch (IOException e) {
        e.printStackTrace();
      }
      initialized = true;
    }
  }

  private static boolean init() {
    if (!initialized) {
      for (String path : filepaths) {
        if (doesFileExist(path)) {
          System.out.println("Found path: " + path);
          init(path);
          break;
        }
      }
      System.out.println("No valid path found.");
    }
    return initialized;
  }

  /**
   * Initializes the log system if required, then returns the telemetry logger.
   * 
   * @return the logger
   */
  public static Logger getTelemetryLogger() {
    init();
    return LogManager.getLogger("TELEMETRY");
  }

  /**
   * Initializes the log system if required, then returns the appropriate class logger.
   * 
   * @return the logger
   */
  public static Logger getPerfLogger() {
    init();
    return LogManager.getLogger("PERF_TIMERS");
  }

  /**
   * Initializes the log system if required, then returns the appropriate class logger.
   * 
   * @param className the class for subsetting the logger
   * @return the logger
   */
  public static Logger getMainLogger(String className) {
    init();
    return LogManager.getLogger(className);
  }

  /**
   * Initializes the log system if required, then returns the appropriate class logger.
   * The custom config file would normally be used for tests.
   * 
   * @param customLogConfig the full path to a customized log configuration file.
   * @param className the class for subsetting the logger
   * @return the logger
   */
  public static Logger getMainLogger(String customLogConfig, String className) {
    init(customLogConfig);
    return LogManager.getLogger(className);
  }

}
