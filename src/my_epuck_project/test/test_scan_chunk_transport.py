from types import SimpleNamespace

import pytest

from my_epuck_project.d500_scan_fix import ChunkAssembler, chunk_checksum


def _chunk(seq, index, total, values, total_beams=8, stamp=(3, 4)):
    return SimpleNamespace(
        source_robot_id='robot1', scan_sequence=seq, total_chunks=total,
        chunk_index=index, total_beams=total_beams, ranges=list(values),
        checksum=chunk_checksum(values, seq, index),
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=stamp[0], nanosec=stamp[1]),
            frame_id='robot1/d500_lidar'),
        angle_min=-3.14, angle_max=3.14, angle_increment=0.897,
        time_increment=0.01, scan_time=0.02, range_min=0.03,
        range_max=12.0)


def test_reassembles_720_style_scan_atomically_and_preserves_metadata():
    assembler = ChunkAssembler('robot1', max_pending=4, timeout_s=0.25)
    first = _chunk(7, 0, 2, range(4))
    second = _chunk(7, 1, 2, range(4, 8))
    assert assembler.add(first, now=1.0) is None
    scan = assembler.add(second, now=1.01)
    assert scan is not None
    assert list(scan.ranges) == [float(item) for item in range(8)]
    assert scan.header.frame_id == 'robot1/d500_lidar'
    assert (scan.header.stamp.sec, scan.header.stamp.nanosec) == (3, 4)
    assert scan.range_max == 12.0
    assert assembler.completed == 1


def test_missing_duplicate_bad_and_stale_chunks_never_publish_partial_scan():
    assembler = ChunkAssembler('robot1', max_pending=1, timeout_s=0.1)
    first = _chunk(1, 0, 2, [1, 2, 3, 4])
    assert assembler.add(first, now=0.0) is None
    assert assembler.add(first, now=0.01) is None
    bad = _chunk(1, 1, 2, [5, 6, 7, 8])
    bad.checksum ^= 1
    assert assembler.add(bad, now=0.02) is None
    assert assembler.pending == {}
    assert assembler.add(_chunk(2, 0, 2, [1, 2, 3, 4]), now=0.0) is None
    assert assembler.add(_chunk(3, 0, 2, [9, 9, 9, 9]), now=0.2) is None
    assert len(assembler.pending) == 1
    assert assembler.dropped >= 2


def test_wrong_robot_and_metadata_mismatch_are_rejected():
    assembler = ChunkAssembler('robot1')
    wrong = _chunk(4, 0, 2, [1, 2, 3, 4])
    wrong.source_robot_id = 'robot2'
    assert assembler.add(wrong, now=0.0) is None
    first = _chunk(5, 0, 2, [1, 2, 3, 4])
    assert assembler.add(first, now=0.0) is None
    second = _chunk(5, 1, 2, [5, 6, 7, 8])
    second.angle_max = 3.0
    second.checksum = chunk_checksum(second.ranges, 5, 1)
    assert assembler.add(second, now=0.01) is None
    assert assembler.pending == {}


@pytest.mark.parametrize('beam_count', [1, 180, 360, 720])
def test_supported_probe_sizes_reconstruct_without_partial_publication(beam_count):
    chunk_size = 180
    values = [float(index) for index in range(beam_count)]
    total = max(1, (beam_count + chunk_size - 1) // chunk_size)
    assembler = ChunkAssembler('robot1', max_pending=4, timeout_s=0.25,
                               max_chunks=8)
    if total == 1:
        result = assembler.add(
            _chunk(99, 0, total, values, beam_count), now=1.0)
    else:
        assert assembler.add(
            _chunk(99, 0, total, values[:chunk_size], beam_count),
            now=1.0) is None
        result = None
        for index in range(1, total):
            begin = index * chunk_size
            result = assembler.add(
                _chunk(99, index, total, values[begin:begin + chunk_size],
                       beam_count), now=1.01 + index * 0.001)
    assert result is not None
    assert assembler.completed == 1
