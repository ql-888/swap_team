#!/usr/bin/env python3
"""Convert grouped taught observation poses into planner-compatible recipes."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def transform_matrix(item):
    value = np.eye(4)
    value[:3, :3] = Rotation.from_quat(item["quaternion_xyzw"]).as_matrix()
    value[:3, 3] = np.asarray(item["translation_m"], dtype=float)
    return value


def inverse(matrix):
    value = np.eye(4)
    value[:3, :3] = matrix[:3, :3].T
    value[:3, 3] = -value[:3, :3] @ matrix[:3, 3]
    return value


def average_transforms(matrices):
    value = np.eye(4)
    value[:3, :3] = Rotation.from_matrix(
        np.asarray([matrix[:3, :3] for matrix in matrices])
    ).mean().as_matrix()
    value[:3, 3] = np.mean([matrix[:3, 3] for matrix in matrices], axis=0)
    return value


def flange_from_tcp():
    value = np.eye(4)
    yaw = math.radians(48.2)
    value[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    value[2, 3] = 0.198
    return value


def prepend_legacy_candidate(record, legacy_record, legacy_source):
    legacy_samples = legacy_record.get("samples")
    if not isinstance(legacy_samples, list) or not legacy_samples:
        raise ValueError("Legacy observation record contains no samples")
    new_candidates = record.get("candidates")
    if not isinstance(new_candidates, list) or not new_candidates:
        raise ValueError("Observation candidate record contains no candidates")

    candidates = [
        {
            "candidate_id": "candidate_01",
            "samples": [legacy_samples[0]],
            "source": str(legacy_source),
        }
    ]
    for index, candidate in enumerate(new_candidates, 2):
        item = dict(candidate)
        item["candidate_id"] = f"candidate_{index:02d}"
        item["source"] = str(record.get("source", "multi_candidate_record"))
        candidates.append(item)
    merged = dict(record)
    merged["candidates"] = candidates
    return merged


def build_candidate_recipes(record):
    if record.get("schema_version") != 2:
        raise ValueError("Observation candidate record must use schema_version 2")
    candidates = record.get("candidates")
    if not isinstance(candidates, list) or not candidates:
        raise ValueError("Observation candidate record contains no candidates")

    recipes = []
    for index, candidate in enumerate(candidates, 1):
        samples = candidate.get("samples")
        if not isinstance(samples, list) or not samples:
            raise ValueError(f"Candidate {index} contains no samples")
        tag_from_flange_samples = []
        for sample in samples:
            base_from_tag = transform_matrix(sample["base_from_global_tag"])
            base_from_flange = transform_matrix(sample["base_from_flange"])
            tag_from_flange_samples.append(inverse(base_from_tag) @ base_from_flange)
        tag_from_flange = average_transforms(tag_from_flange_samples)
        candidate_id = str(candidate.get("candidate_id", f"candidate_{index:02d}"))
        recipes.append(
            {
                "object_id": "apriltag_object",
                "object_from_tcp": (tag_from_flange @ flange_from_tcp()).tolist(),
                "axial_angles_deg": [0.0],
                "pregrasp_distance_m": 0.001,
                "retreat_distance_m": 0.001,
                "gripper_opening_m": 0.085,
                "gripper_closed_m": 0.058,
                "execution_ready": False,
                "candidate_id": candidate_id,
                "observation_offset_source": str(
                    candidate.get(
                        "source", record.get("source", "multi_candidate_record")
                    )
                ),
            }
        )
    return recipes


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--legacy-record", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    record = yaml.safe_load(args.record.read_text(encoding="utf-8"))
    record["source"] = str(args.record)
    if args.legacy_record is not None:
        legacy_record = yaml.safe_load(args.legacy_record.read_text(encoding="utf-8"))
        record = prepend_legacy_candidate(record, legacy_record, args.legacy_record)
    recipes = build_candidate_recipes(record)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for index, recipe in enumerate(recipes, 1):
        output = args.output_dir / f"candidate_{index:02d}.yaml"
        output.write_text(yaml.safe_dump(recipe, sort_keys=False), encoding="utf-8")
        print(f"saved: {output} ({recipe['candidate_id']})")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (KeyError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=__import__("sys").stderr)
        raise SystemExit(1)
