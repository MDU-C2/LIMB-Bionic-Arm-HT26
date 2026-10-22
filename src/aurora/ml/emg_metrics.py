# Majd
"""Scores kept per participant, including absent and missed phases."""

from __future__ import annotations

import numpy as np


def classification_scores(actual, predicted, classes: int) -> dict:
    actual, predicted = np.asarray(actual), np.asarray(predicted)
    matrix = np.zeros((classes, classes), dtype=int)
    np.add.at(matrix, (actual, predicted), 1)
    true_positive = np.diag(matrix)
    support, count = matrix.sum(axis=1), matrix.sum(axis=0)
    f1 = np.divide(
        2 * true_positive,
        support + count,
        out=np.zeros(classes, dtype=float),
        where=(support + count) > 0,
    )
    return {
        "confusion_matrix": matrix.tolist(),
        "support": support.tolist(),
        "f1": f1.tolist(),
        "macro_f1": float(f1.mean()),
        "accuracy": float(np.mean(actual == predicted)),
    }


def grip_scores(actual, predicted, records: list[dict], *, binary: bool = False) -> dict:
    expected = np.isin(actual, (1, 2))
    closed = np.asarray(predicted, dtype=bool) if binary else np.isin(predicted, (1, 2))
    result = classification_scores(expected.astype(int), closed.astype(int), 2)
    result["false_closed_windows"] = int(np.sum(closed & ~expected))
    result["false_open_windows"] = int(np.sum(~closed & expected))
    # A new trial/gap is not a state transition.
    false_closures = false_releases = 0
    for index in range(1, len(records)):
        current, previous = records[index], records[index - 1]
        if (current["subject_id"], current["trial_id"]) != (
            previous["subject_id"],
            previous["trial_id"],
        ):
            continue
        if current["end_s"] - previous["end_s"] > 0.2:
            continue
        false_closures += int(closed[index] and not closed[index - 1] and not expected[index])
        false_releases += int(not closed[index] and closed[index - 1] and expected[index])
    result.update(false_closures=false_closures, false_releases=false_releases)
    delays = {"grip": [], "release": []}
    misses = dict.fromkeys(delays, 0)
    groups = {}
    for index, record in enumerate(records):
        if actual[index] not in (1, 3):
            continue
        key = (record["subject_id"], record["trial_id"], record["segment_start_s"], actual[index])
        groups.setdefault(key, []).append(index)
    for key, indices in groups.items():
        phase = "grip" if key[-1] == 1 else "release"
        found = next((i for i in indices if closed[i] == (key[-1] == 1)), None)
        if found is None:
            misses[phase] += 1
        else:
            delays[phase].append(records[found]["end_s"] - key[-2])
    result["response_delay_s"] = delays
    result["missed_transitions"] = misses
    return result
