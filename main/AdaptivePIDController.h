#ifndef ADAPTIVE_PID_CONTROLLER_H
#define ADAPTIVE_PID_CONTROLLER_H

class AdaptivePIDController {
private:
    // Tuning parameters
    float kp;
    float ki;
    float kd;

    // State tracking
    float previousError;
    float integral;

    // Adaptation tracking
    const int MAX_ERROR_HISTORY = 10;
    float errorHistory[10];
    int errorHistoryIndex;

    // Oscillation and overshoot detection
    int oscillationCount;
    int overshootCount;

    // Helper method to add error to history
    void addErrorToHistory(float error) {
        errorHistory[errorHistoryIndex] = error;
        errorHistoryIndex = (errorHistoryIndex + 1) % MAX_ERROR_HISTORY;
    }

    // Adaptive tuning logic
    void adaptiveTuning(float error) {
        // Oscillation detection
        bool isOscillating = false;
        for (int i = 1; i < MAX_ERROR_HISTORY; i++) {
            int prevIndex = (errorHistoryIndex - i + MAX_ERROR_HISTORY) % MAX_ERROR_HISTORY;
            if (errorHistory[prevIndex] * error < 0) {
                isOscillating = true;
                break;
            }
        }

        if (isOscillating) {
            oscillationCount++;
            
            // If oscillating too much, reduce proportional and derivative gains
            if (oscillationCount > 3) {
                kp *= 0.8;  // Reduce proportional gain
                kd *= 0.8;  // Reduce derivative gain
                oscillationCount = 0;
            }
        }

        // Overshoot detection
        if (abs(error) > abs(previousError)) {
            overshootCount++;
            
            // If consistent overshoot, increase derivative gain
            if (overshootCount > 2) {
                kd *= 1.2;  // Increase derivative gain
                overshootCount = 0;
            }
        }

        // Anti-windup for integral term
        if (abs(integral) > 10) {
            integral *= 0.9;
            ki *= 0.9;  // Reduce integral gain
        }

        // Ensure gains stay within reasonable bounds
        kp = constrain(kp, 0.1, 5.0);
        ki = constrain(ki, 0.1, 2.0);
        kd = constrain(kd, 0.01, 3.0);
    }

public:
    // Constructor with initial gains
    AdaptivePIDController(float initial_kp = .1, float initial_ki = 0.2, float initial_kd = 0.1) 
        : kp(initial_kp), ki(initial_ki), kd(initial_kd), 
          previousError(0), integral(0), 
          errorHistoryIndex(0), 
          oscillationCount(0), overshootCount(0) {
        // Initialize error history to zero
        for (int i = 0; i < MAX_ERROR_HISTORY; i++) {
            errorHistory[i] = 0;
        }
    }

    // Compute PID output with adaptive tuning
    float compute(float setpoint, float currentValue, float dt) {
        // Calculate error
        float error = setpoint - currentValue;

        // Add error to history
        addErrorToHistory(error);

        // Compute PID terms
        float p_term = kp * error;
        
        // Integral term
        integral += error * dt;
        float i_term = ki * integral;
        
        // Derivative term
        float d_term = (dt > 0) ? (kd * (error - previousError) / dt) : 0;

        // Compute output
        float output = p_term + i_term + d_term;

        // Adaptive tuning
        adaptiveTuning(error);

        // Update previous error
        previousError = error;

        return output;
    }

    // Getter methods for current gains
    float getKp() const { return kp; }
    float getKi() const { return ki; }
    float getKd() const { return kd; }
};

#endif // ADAPTIVE_PID_CONTROLLER_H