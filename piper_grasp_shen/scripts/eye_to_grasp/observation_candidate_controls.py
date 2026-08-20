"""Terminal controls shared by the headless observation recorder."""


def build_single_sample_candidate(
    candidate_id: str,
    sample: dict,
    *,
    timestamp_s: float,
    global_image: str,
    local_image: str,
) -> dict:
    return {
        "candidate_id": candidate_id,
        "timestamp_s": timestamp_s,
        "global_image": global_image,
        "local_image": local_image,
        "samples": [sample],
    }


def terminal_action(line: str) -> str:
    if line == "":
        return "eof"
    command = line.strip()
    if not command:
        return "capture"
    if command.lower() == "q":
        return "save"
    return "invalid"
