package frc.lib.utils;

// NOTE: This file is available at
// https://algs4.cs.princeton.edu/code/edu/princeton/cs/algs4/LinearRegression.java.html

/**
 * The {@code LinearRegression} class performs a simple linear regression on an set of <em>n</em>
 * data points (<em>y<sub>i</sub></em>, <em>x<sub>i</sub></em>). That is, it fits a straight line
 * <em>y</em> = &alpha; + &beta; <em>x</em>, (where <em>y</em> is the response variable, <em>x</em>
 * is the predictor variable, &alpha; is the <em>y-intercept</em>, and &beta; is the <em>slope</em>)
 * that minimizes the sum of squared residuals of the linear regression model. It also computes
 * associated statistics, including the coefficient of determination <em>R</em><sup>2</sup> and the
 * standard deviation of the estimates for the slope and <em>y</em>-intercept.
 *
 * @author Robert Sedgewick
 * @author Kevin Wayne
 */
public class LinearRegression {
  private final double intercept, slope;
  private final double r2;
  private final double svar0, svar1;

  /**
   * Construct a LinearRegression object with the given matched data points.
   *
   * @param matchedXY a map containing the matched data points, where the keys represent the x
   *     values and the values represent the y values
   */
  //  public LinearRegression(Map<Double, Double> matchedXY) {
  //    double[] x = new double[matchedXY.size()];
  //    double[] y = new double[matchedXY.size()];
  //    int i = 0;
  //    for (Map.Entry<Double, Double> entry : matchedXY.entrySet()) {
  //      x[i] = entry.getKey();
  //      y[i] = entry.getValue();
  //      i++;
  //    }
  //  }

  /**
   * Performs a linear regression on the data points {@code (y[i], x[i])}.
   *
   * @param x the values of the predictor variable
   * @param y the corresponding values of the response variable
   * @throws IllegalArgumentException if the lengths of the two arrays are not equal
   */
  public LinearRegression(double[] x, double[] y) {
    if (x.length != y.length) {
      throw new IllegalArgumentException("array lengths are not equal");
    }
    int n = x.length;

    // first pass
    double sumx = 0.0, sumy = 0.0, sumx2 = 0.0;
    for (int i = 0; i < n; i++) {
      sumx += x[i];
      sumx2 += x[i] * x[i];
      sumy += y[i];
    }
    double xbar = sumx / n;
    double ybar = sumy / n;

    // second pass: compute summary statistics
    double xxbar = 0.0, yybar = 0.0, xybar = 0.0;
    for (int i = 0; i < n; i++) {
      xxbar += (x[i] - xbar) * (x[i] - xbar);
      yybar += (y[i] - ybar) * (y[i] - ybar);
      xybar += (x[i] - xbar) * (y[i] - ybar);
    }
    slope = xybar / xxbar;
    intercept = ybar - slope * xbar;

    // more statistical analysis
    double rss = 0.0; // residual sum of squares
    double ssr = 0.0; // regression sum of squares
    for (int i = 0; i < n; i++) {
      double fit = slope * x[i] + intercept;
      rss += (fit - y[i]) * (fit - y[i]);
      ssr += (fit - ybar) * (fit - ybar);
    }

    int degreesOfFreedom = n - 2;
    r2 = ssr / yybar;
    double svar = rss / degreesOfFreedom;
    svar1 = svar / xxbar;
    svar0 = svar / n + xbar * xbar * svar1;
  }

  /**
   * Returns the <em>y</em>-intercept &alpha; of the best of the best-fit line <em>y</em> = &alpha;
   * + &beta; <em>x</em>.
   *
   * @return the <em>y</em>-intercept &alpha; of the best-fit line <em>y = &alpha; + &beta; x</em>
   */
  public double intercept() {
    return intercept;
  }

  /**
   * Returns the slope &beta; of the best of the best-fit line <em>y</em> = &alpha; + &beta;
   * <em>x</em>.
   *
   * @return the slope &beta; of the best-fit line <em>y</em> = &alpha; + &beta; <em>x</em>
   */
  public double slope() {
    return slope;
  }

  /**
   * Returns the coefficient of determination <em>R</em><sup>2</sup>.
   *
   * @return the coefficient of determination <em>R</em><sup>2</sup>, which is a real number between
   *     0 and 1
   */
  public double R2() {
    return r2;
  }

  /**
   * Returns the standard error of the estimate for the intercept.
   *
   * @return the standard error of the estimate for the intercept
   */
  public double interceptStdErr() {
    return Math.sqrt(svar0);
  }

  /**
   * Returns the standard error of the estimate for the slope.
   *
   * @return the standard error of the estimate for the slope
   */
  public double slopeStdErr() {
    return Math.sqrt(svar1);
  }

  /**
   * Returns the expected response {@code y} given the value of the predictor variable {@code x}.
   *
   * @param x the value of the predictor variable
   * @return the expected response {@code y} given the value of the predictor variable {@code x}
   */
  public double predict(double x) {
    return slope * x + intercept;
  }

  /**
   * Returns a string representation of the simple linear regression model.
   *
   * @return a string representation of the simple linear regression model, including the best-fit
   *     line and the coefficient of determination <em>R</em><sup>2</sup>
   */
  public String toString() {
    StringBuilder s = new StringBuilder();
    s.append(String.format("%.2f n + %.2f", slope(), intercept()));
    s.append("  (R^2 = " + String.format("%.3f", R2()) + ")");
    return s.toString();
  }
}
