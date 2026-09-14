# ── Payload release configuration ─────────────────────────────────────────────
SERVO_CHANNEL: int = 2                  # PWM output channel for the release servo
SERVO_OPEN_PWM: int = 1400             # PWM value that opens the payload latch
SERVO_CLOSED_PWM: int = 1900          # PWM value that keeps the payload closed
DROP_MAX_RETRIES: int = 3             # Maximum retry attempts for release operation
RETRY_DELAY: float = 1.0                # Delay between retry attempts [s]

DROP_HEIGHT: float = 0.65             # Height at which the payload is released [m]

# ── PID gains for package alignment / positioning ────────────────────────────
X_KP: float = 0.123                   # Proportional gain for X axis
X_KI: float = 0.0                     # Integral gain for X axis
X_KD: float = 0.02                    # Derivative gain for X axis

Y_KP: float = 0.123                   # Proportional gain for Y axis
Y_KI: float = 0.0                     # Integral gain for Y axis
Y_KD: float = 0.02                    # Derivative gain for Y axis

PHOTO_FAIL_THRESHOLD: int = 5              # Number of consecutive photo failures before aborting
LOST_THRESHOLD: int = 5                     # Number of consecutive lost detections before aborting

# Package FSM outcomes
LOST_PERSON = "LOST_PERSON"
ALIGNMENT_FAILED = "ALIGNMENT_FAILED"
DROP_RETRY = "DROP_RETRY"
DROP_READY = "DROP_READY"
