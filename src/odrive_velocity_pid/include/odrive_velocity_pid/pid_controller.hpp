// pid_controller.hpp
//
// Generic, reusable PID controller — pure C++, no ROS dependencies.
//
// Design notes:
//   - Gains (kp, ki, kd) and limit fields are public so callers can update them at any time
//     without requiring a setter method, keeping the class simple and flat.
//   - `saturated` is also public so the caller can override it after combining the PID output
//     with a feedforward term and applying a final clamp.  The anti-windup logic inside
//     compute() reads `saturated` at the START of each call (reflecting the state set after
//     the PREVIOUS call), which correctly accounts for external clamping.
//   - All internal mutable state is private; only `saturated` is semi-shared.

#pragma once

#include <algorithm>
#include <cmath>

class PidController
{
public:
  // ── Gains ─────────────────────────────────────────────────────────────────────────────────────
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};

  // ── Limits ────────────────────────────────────────────────────────────────────────────────────
  double integral_limit{1.0};   // symmetric integral accumulator clamp: integral ∈ [-limit, limit]
  double deadband{0.0};         // error values within ±deadband are treated as zero
  double output_min{-1e9};      // output saturation lower bound
  double output_max{ 1e9};      // output saturation upper bound

  // ── Saturation flag ───────────────────────────────────────────────────────────────────────────
  // Set by compute() after clamping the PID output.  The caller may override this AFTER compute()
  // returns to reflect saturation from an external final clamp (e.g., after adding feedforward).
  // The updated value is then used by the NEXT compute() call for anti-windup.
  bool saturated{false};

  // ── reset — clear all stateful fields ─────────────────────────────────────────────────────────
  void reset()
  {
    integral_         = 0.0;
    prev_measurement_ = 0.0;
    saturated         = false;
    first_run_        = true;
  }

  // ── compute — main PID step ────────────────────────────────────────────────────────────────────
  //
  // Returns the PID output (P + I + D terms) clamped to [output_min, output_max].
  // Feedforward is NOT included here; the caller adds it and re-clamps the total if needed.
  //
  // Parameters:
  //   setpoint    – desired process value
  //   measurement – current measured process value
  //   dt          – elapsed time since last call [seconds]; must be > 0
  //
  // Anti-windup strategy (directional / conditional):
  //   Integration is frozen when `saturated` is true AND the current error would push the
  //   integral further in the saturation direction (i.e., error * integral > 0).  This is
  //   identical to the logic in the original velocity_pid_node.cpp.
  //
  // Derivative strategy: derivative-on-measurement.
  //   d_term = -(measurement − prev_measurement) / dt
  //   Using the measurement rate rather than the error rate avoids a "derivative kick" when
  //   the setpoint changes step-wise.
  double compute(double setpoint, double measurement, double dt)
  {
    // ── Error with deadband ──────────────────────────────────────────────────────────────────────
    double error = setpoint - measurement;
    if (std::abs(error) < deadband) {
      error = 0.0;
    }

    // ── Derivative-on-measurement ────────────────────────────────────────────────────────────────
    double derivative = 0.0;
    if (!first_run_ && dt > 0.0) {
      derivative = -(measurement - prev_measurement_) / dt;
    }
    first_run_        = false;
    prev_measurement_ = measurement;

    // ── Directional anti-windup ──────────────────────────────────────────────────────────────────
    // Allow integration only when it would help reduce windup rather than deepen it.
    if (!saturated || (error * integral_ <= 0.0)) {
      integral_ += error * dt;
      integral_  = std::clamp(integral_, -integral_limit, integral_limit);
    }

    // ── PID output ───────────────────────────────────────────────────────────────────────────────
    double output = kp * error + ki * integral_ + kd * derivative;

    // ── Output saturation ────────────────────────────────────────────────────────────────────────
    const bool was_saturated = saturated;
    output    = std::clamp(output, output_min, output_max);
    // Inclusive boundary: output == output_max or output == output_min is treated as saturated,
    // consistent with the original code's  (|output| >= limit)  check.
    saturated = (output >= output_max || output <= output_min);

    // On the first cycle after leaving saturation, re-clamp the integral to clear any
    // overshoot that built up while anti-windup was active.
    if (was_saturated && !saturated) {
      integral_ = std::clamp(integral_, -integral_limit, integral_limit);
    }

    return output;
  }

private:
  double integral_{0.0};
  double prev_measurement_{0.0};
  bool   first_run_{true};
};
