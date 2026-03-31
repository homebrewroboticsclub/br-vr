"""Quest controller buttons on /quest/joints (JointState names L_X, L_Y, R_A, …)."""


def rising_edge(prev: float, cur: float, threshold: float = 0.5) -> bool:
    """True when signal crosses from not-pressed to pressed (typ. 0→1)."""
    return cur > threshold and prev <= threshold
