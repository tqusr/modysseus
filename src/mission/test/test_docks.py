import textwrap

from mission.docks import load_docks


def test_mission_package_importable():
    import mission  # noqa: F401


def test_load_docks_parses_all_fields(tmp_path):
    yaml_path = tmp_path / "docks.yaml"
    yaml_path.write_text(textwrap.dedent("""\
        docks:
          A:
            tag_id: 0
            dock_pose: {x: 1.0, y: 2.0, theta: 0.0}
            staging_pose: {x: 0.5, y: 2.0, theta: 0.0}
          B:
            tag_id: 1
            dock_pose: {x: 3.0, y: 2.0, theta: 1.57}
            staging_pose: {x: 2.5, y: 2.0, theta: 1.57}
        """))

    docks = load_docks(str(yaml_path))

    assert set(docks.keys()) == {"A", "B"}
    assert docks["A"].letter == "A"
    assert docks["A"].tag_id == 0
    assert docks["A"].dock_pose == {"x": 1.0, "y": 2.0, "theta": 0.0}
    assert docks["B"].staging_pose == {"x": 2.5, "y": 2.0, "theta": 1.57}
