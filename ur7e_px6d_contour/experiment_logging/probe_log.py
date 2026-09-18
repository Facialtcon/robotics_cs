"""Same probe-episode schema for simulation and real experiments."""
import csv
import json


def write_probe_logs(directory, episodes):
    records = [episode.to_record() for episode in episodes]
    with (directory / "probe_episodes.json").open("w", encoding="utf-8") as handle:
        json.dump(records, handle, indent=2, allow_nan=False)
    fields = ["probe_id", "anchor_pose", "return_target_pose", "probe_direction", "probe_start_time", "probe_end_time",
              "return_end_time", "max_probe_distance", "contact", "outcome", "contact_pose",
              "end_pose", "returned_pose", "max_force", "return_completed", "accepted_as_boundary",
              "policy_state", "purpose", "initialization_round", "recovery_id", "angle_deg", "rejection_reason"]
    with (directory / "probe_episodes.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for record in records:
            writer.writerow({key: json.dumps(record[key]) if isinstance(record[key], list) else record[key] for key in fields})
    with (directory / "probe_forces.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["probe_id", "timestamp", "fxy", "phase"])
        for record in records:
            writer.writerows((record["probe_id"], *sample) for sample in record["force_samples"])
