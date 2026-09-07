import csv
import io

from my_epuck_project.thin_supervisor_capture import _BufferedCsvRows


def test_thin_capture_serializes_rows_in_bounded_chunks():
    stream = io.StringIO()
    sink = _BufferedCsvRows(stream, ('time', 'robot', 'value'),
                            max_buffer_rows=2)

    sink.writerow((0.02, 'robot1', 1.25))
    assert stream.getvalue() == 'time,robot,value\n'
    sink.writerow((0.04, 'robot1', 2.5))
    sink.flush()

    rows = list(csv.reader(io.StringIO(stream.getvalue())))
    assert rows == [
        ['time', 'robot', 'value'],
        ['0.02', 'robot1', '1.25'],
        ['0.04', 'robot1', '2.5'],
    ]
    assert sink.flushes == 2
    assert sink.rows == 2
    assert sink.max_buffer_bytes_seen > 0


def test_thin_capture_keeps_numeric_values_until_flush():
    stream = io.StringIO()
    sink = _BufferedCsvRows(stream, ('value',), max_buffer_rows=3)
    value = 1.0 / 3.0
    sink.writerow((value,))

    assert sink.rows_buffer == [(value,)]
    assert stream.getvalue() == 'value\n'

    sink.flush()
    assert sink.rows_buffer == []
    assert stream.getvalue().splitlines() == ['value', str(value)]
