from dataclasses import dataclass

import yaml


@dataclass(frozen=True)
class Dock:
    letter: str
    tag_id: int
    dock_pose: dict
    staging_pose: dict


def load_docks(path: str) -> dict[str, Dock]:
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
