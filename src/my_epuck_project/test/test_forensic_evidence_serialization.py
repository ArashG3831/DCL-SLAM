import csv
from types import SimpleNamespace

from my_epuck_project.forensic_evidence import ForensicEvidenceWriter


def _stamp(sec, nanosec):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def _odom():
    return SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(12, 345000000), frame_id="robot1/odom"),
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.25, w=0.9682458),
        )),
        twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.1, y=0.2, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.3),
        )),
    )


def _tf_message():
    item = SimpleNamespace(
        header=SimpleNamespace(
            stamp=_stamp(13, 456000000), frame_id="robot1/map"),
        child_frame_id="robot1/odom",
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=3.0, y=4.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.1, w=0.995),
        ),
    )
    return SimpleNamespace(transforms=[item])


def test_high_rate_forensic_csv_rows_keep_legacy_schema_and_values(tmp_path):
    writer = ForensicEvidenceWriter(tmp_path, ("robot1",), interval_s=15.0)
    writer.record_odom("robot1", _odom(), 12.5, 1.25)
    writer.record_raw_tf("/tf", _tf_message(), 13.5, 1.5)
    writer.close()

    with (tmp_path / "forensic" / "robot1_odom.csv").open(
            newline="", encoding="utf-8") as stream:
        odom_rows = list(csv.reader(stream))
    with (tmp_path / "forensic" / "raw_tf.csv").open(
            newline="", encoding="utf-8") as stream:
        tf_rows = list(csv.reader(stream))

    assert odom_rows[0] == [
        "robot_id", "received_ros_time_s", "received_wall_elapsed_s",
        "header_stamp", "frame_id", "pose_x", "pose_y", "pose_z",
        "orientation_x", "orientation_y", "orientation_z", "orientation_w",
        "twist_linear_x", "twist_linear_y", "twist_linear_z",
        "twist_angular_x", "twist_angular_y", "twist_angular_z",
    ]
    assert odom_rows[1] == [
        "robot1", "12.5", "1.25", "12.345000000", "robot1/odom",
        "1.0", "2.0", "0.0", "0.0", "0.0", "0.25", "0.9682458",
        "0.1", "0.2", "0.0", "0.0", "0.0", "0.3",
    ]
    assert tf_rows[0] == [
        "topic", "static", "received_ros_time_s", "received_wall_elapsed_s",
        "transform_stamp", "parent_frame", "child_frame", "translation_x",
        "translation_y", "translation_z", "rotation_x", "rotation_y",
        "rotation_z", "rotation_w",
    ]
    assert tf_rows[1] == [
        "/tf", "False", "13.5", "1.5", "13.456000000", "robot1/map",
        "robot1/odom", "3.0", "4.0", "0.0", "0.0", "0.0", "0.1", "0.995",
    ]
