#include <cmath>
#include <iostream>
#include <bits/stdc++.h>
/// Very basic, mostly educational PID controller with derivative filter.
/// Standard PID (proportional, integral, derivative) controller. Derivative
/// component is filtered using an exponential moving average filter.
template <typename T>
class PID
{
public:
    PID() = default;
    /// @param  kp
    ///         Proportional gain
    /// @param  ki
    ///         Integral gain
    /// @param  kd
    ///         Derivative gain
    /// @param  Ts
    ///         Sampling time (seconds)
    /// @param  fc
    ///         Cutoff frequency of derivative EMA filter (Hertz),
    ///         zero to disable the filter entirely
    PID(T kp, T ki, T kd, T Ts, T f_c = 0.0f,
        T maxOutput = 255.0f)
        : Ts(Ts), maxOutput(maxOutput)
    {
        setKp(kp);
        setKi(ki);
        setKd(kd);
        setEMACutoff(f_c);
    }

    /// Update the controller: given the current position, compute the control
    /// action.
    T update(T input)
    {
        // The error is the difference between the reference (setpoint) and the
        // actual position (input)
        T error = setpoint - input;
        // The integral or sum of current and previous errors
        T newIntegral = integral + error;
        // Compute the difference between the current and the previous input,
        // but compute a weighted average using a factor α ∊ (0,1]
        T diff = emaAlpha * (prevInput - input);
        // Update the average
        prevInput -= diff;

        // Check if we can turn off the motor
        if (activityCount >= activityThres && activityThres)
        {
            T filtError = setpoint - prevInput;
            if (filtError >= -errThres && filtError <= errThres)
            {
                errThres = 2; // hysteresis
                return 0;
            }
            else
            {
                errThres = 1;
            }
        }
        else
        {
            ++activityCount;
            errThres = 1;
        }

        bool backward = false;
        T calcIntegral = backward ? newIntegral : integral;

        // Standard PID rule
        T output = kp * error + ki_Ts * calcIntegral + kd_Ts * diff;

        // Clamp and anti-windup
        if (output > maxOutput)
            output = maxOutput;
        else if (output < -maxOutput)
            output = -maxOutput;
        else
            integral = newIntegral;

        return output;
    }

    void setKp(T kp) { this->kp = kp; }               ///< Proportional gain
    void setKi(T ki) { this->ki_Ts = ki * this->Ts; } ///< Integral gain
    void setKd(T kd) { this->kd_Ts = kd / this->Ts; } ///< Derivative gain

    T getKp() const { return kp; }         ///< Proportional gain
    T getKi() const { return ki_Ts / Ts; } ///< Integral gain
    T getKd() const { return kd_Ts * Ts; } ///< Derivative gain

    /// Set the cutoff frequency (-3 dB point) of the exponential moving average
    /// filter that is applied to the input before taking the difference for
    /// computing the derivative term.
    void setEMACutoff(T f_c)
    {
        T f_n = f_c * Ts; // normalized sampling frequency
        this->emaAlpha = f_c == 0 ? 1 : calcAlphaEMA(f_n);
    }

    /// Set the reference/target/setpoint of the controller.
    void setSetpoint(T setpoint)
    {
        if (this->setpoint != setpoint)
            this->activityCount = 0;
        this->setpoint = setpoint;
    }
    /// @see @ref setSetpoint(T)
    T getSetpoint() const { return setpoint; }

    /// Set the maximum control output magnitude. Default is 255, which clamps
    /// the control output in [-255, +255].
    void setMaxOutput(T maxOutput) { this->maxOutput = maxOutput; }
    /// @see @ref setMaxOutput(T)
    T getMaxOutput() const { return maxOutput; }

    /// Reset the activity counter to prevent the motor from turning off.
    void resetActivityCounter() { this->activityCount = 0; }
    /// Set the number of seconds after which the motor is turned off, zero to
    /// keep it on indefinitely.
    void setActivityTimeout(T s)
    {
        if (s == 0)
            activityThres = 0;
        else
            activityThres = uint16_t(s / Ts) == 0 ? 1 : s / Ts;
    }

    /// Reset the sum of the previous errors to zero.
    void resetIntegral() { integral = 0; }

private:
    T calcAlphaEMA(T fn)
    {
        if (fn <= 0)
            return 1;
        // α(fₙ) = cos(2πfₙ) - 1 + √( cos(2πfₙ)² - 4 cos(2πfₙ) + 3 )
        const T c = std::cos(2 * T(M_PI) * fn);
        return c - 1 + std::sqrt(c * c - 4 * c + 3);
    }
    T Ts = 1;                   ///< Sampling time (seconds)
    T maxOutput = 255;          ///< Maximum control output magnitude
    T kp = 1;                   ///< Proportional gain
    T ki_Ts = 0;                ///< Integral gain times Ts
    T kd_Ts = 0;                ///< Derivative gain divided by Ts
    T emaAlpha = 1;             ///< Weight factor of derivative EMA filter.
    T prevInput = 0;            ///< (Filtered) previous input for derivative.
    uint16_t activityCount = 0; ///< How many ticks since last setpoint change.
    uint16_t activityThres = 0; ///< Threshold for turning off the output.
    uint8_t errThres = 1;       ///< Threshold with hysteresis.
    T integral = 0;             ///< Sum of previous errors for integral.
    T setpoint = 0;             ///< Position reference.
};