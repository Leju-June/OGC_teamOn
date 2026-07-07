def algorithm(prob_info: dict, timelimit: float = 60) -> dict:
    """
    OGC 2026 Baseline Simple Algorithm Wrapper.
    """
    import baseline_simple
    return baseline_simple.algorithm(prob_info, timelimit)
