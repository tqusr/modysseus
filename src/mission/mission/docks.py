"""Dock definitions loaded from the docks YAML."""
from dataclasses import dataclass

import yaml


@dataclass(frozen=True)
class Dock:
    """A charging dock.

    Attributes:
        letter: Dock identifier, e.g. "A".
        tag_id: AprilTag id on the dock.
        dock_pose: Pose dict with x, y and theta in the map frame.
        staging_pose: Pre-dock pose dict with x, y and theta.
    """

    letter: str
    tag_id: int
    dock_pose: dict
    staging_pose: dict


def load_docks(path: str) -> dict[str, Dock]:
    """Loads the docks from a YAML file.

    Args:
        path: Path to a YAML file with a top-level `docks` mapping.

    Returns:
        Dock letter to `Dock`.
    """
    with open(path) as handle:
        raw = yaml.safe_load(handle)
    docks = {}
    for letter, entry in raw["docks"].items():
        docks[letter] = Dock(
            letter=letter,
            tag_id=entry["tag_id"],
            dock_pose=entry["dock_pose"],
            staging_pose=entry["staging_pose"],
        )
    return docks
