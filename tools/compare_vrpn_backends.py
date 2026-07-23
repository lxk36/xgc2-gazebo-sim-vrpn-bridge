#!/usr/bin/env python3
"""Report legacy/process and in-process VRPN stream differences.

This is deliberately a diagnostic, not a regression gate: it has no acceptance
thresholds and does not return failure merely because one backend is faster,
less jittery, or otherwise different.
"""

import argparse
import bisect
import json
import math
import statistics
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def stream_metrics(records):
    receive_times = [record["receive_time"] for record in records]
    periods_ms = [
        (right - left) * 1000.0
        for left, right in zip(receive_times, receive_times[1:])
    ]
    stamps = [record["stamp"] for record in records]
    duration = receive_times[-1] - receive_times[0] if len(receive_times) > 1 else 0.0
    return {
        "count": len(records),
        "receive_rate_hz": (len(records) - 1) / duration if duration > 0.0 else None,
        "period_ms": {
            "p50": percentile(periods_ms, 0.50),
            "p95": percentile(periods_ms, 0.95),
            "max": max(periods_ms) if periods_ms else None,
            "stddev": statistics.pstdev(periods_ms) if len(periods_ms) > 1 else None,
        },
        "stamp_monotonic_violations": sum(
            right < left for left, right in zip(stamps, stamps[1:])
        ),
        "nonfinite_value_records": sum(
            not all(math.isfinite(value) for value in record["values"])
            for record in records
        ),
    }


def pose_difference(left, right):
    position_error = math.sqrt(
        sum((left[index] - right[index]) ** 2 for index in range(3))
    )
    quaternion_dot = abs(sum(left[index] * right[index] for index in range(3, 7)))
    quaternion_dot = max(-1.0, min(1.0, quaternion_dot))
    return position_error, 2.0 * math.acos(quaternion_dot)


def vector_difference(left, right):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def paired_metrics(left_records, right_records, kind, pair_window_s):
    if not left_records or not right_records:
        return {"pair_count": 0}

    left_origin = left_records[0]["receive_time"]
    right_origin = right_records[0]["receive_time"]
    right_elapsed = [
        record["receive_time"] - right_origin for record in right_records
    ]
    arrival_offsets_ms = []
    primary_errors = []
    secondary_errors = []

    for left in left_records:
        elapsed = left["receive_time"] - left_origin
        index = bisect.bisect_left(right_elapsed, elapsed)
        candidates = [
            candidate
            for candidate in (index - 1, index)
            if 0 <= candidate < len(right_records)
        ]
        if not candidates:
            continue
        nearest = min(candidates, key=lambda candidate: abs(right_elapsed[candidate] - elapsed))
        offset_s = right_elapsed[nearest] - elapsed
        if abs(offset_s) > pair_window_s:
            continue
        arrival_offsets_ms.append(offset_s * 1000.0)
        right = right_records[nearest]
        if kind == "pose":
            position_error, orientation_error = pose_difference(
                left["values"], right["values"]
            )
            primary_errors.append(position_error)
            secondary_errors.append(orientation_error)
        else:
            primary_errors.append(
                vector_difference(left["values"], right["values"])
            )

    result = {
        "pair_count": len(primary_errors),
        "arrival_offset_ms": {
            "p50": percentile(arrival_offsets_ms, 0.50),
            "p95_absolute": percentile(
                [abs(value) for value in arrival_offsets_ms], 0.95
            ),
            "max_absolute": max(
                [abs(value) for value in arrival_offsets_ms], default=None
            ),
        },
    }
    if primary_errors:
        if kind == "pose":
            result["position_error_m"] = {
                "rms": math.sqrt(
                    sum(value * value for value in primary_errors)
                    / len(primary_errors)
                ),
                "p95": percentile(primary_errors, 0.95),
                "max": max(primary_errors),
            }
            result["orientation_error_rad"] = {
                "rms": math.sqrt(
                    sum(value * value for value in secondary_errors)
                    / len(secondary_errors)
                ),
                "p95": percentile(secondary_errors, 0.95),
                "max": max(secondary_errors),
            }
        else:
            result["vector_error_norm"] = {
                "rms": math.sqrt(
                    sum(value * value for value in primary_errors)
                    / len(primary_errors)
                ),
                "p95": percentile(primary_errors, 0.95),
                "max": max(primary_errors),
            }
    return result


def pose_values(message):
    return (
        message.pose.position.x,
        message.pose.position.y,
        message.pose.position.z,
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z,
        message.pose.orientation.w,
    )


def twist_values(message):
    return (
        message.twist.linear.x,
        message.twist.linear.y,
        message.twist.linear.z,
        message.twist.angular.x,
        message.twist.angular.y,
        message.twist.angular.z,
    )


def normalized_namespace(value):
    return "/" + value.strip("/")


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Observe two vrpn_client_ros namespaces and emit descriptive JSON. "
            "No reported difference is treated as a pass/fail gate."
        )
    )
    parser.add_argument("--legacy-namespace", default="/legacy_vrpn_client")
    parser.add_argument("--plugin-namespace", default="/plugin_vrpn_client")
    parser.add_argument("--trackers", required=True, help="Comma-separated tracker names")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--pair-window-ms", type=float, default=30.0)
    parser.add_argument("--output", help="Optional JSON report path; stdout is always used")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.duration <= 0.0 or args.pair_window_ms <= 0.0:
        raise SystemExit("--duration and --pair-window-ms must be greater than zero")
    trackers = [value.strip("/") for value in args.trackers.split(",") if value.strip("/")]
    if not trackers:
        raise SystemExit("--trackers must contain at least one tracker")

    namespaces = {
        "legacy": normalized_namespace(args.legacy_namespace),
        "plugin": normalized_namespace(args.plugin_namespace),
    }
    records = {}
    subscribers = []
    stream_types = {
        "pose": (PoseStamped, pose_values),
        "twist": (TwistStamped, twist_values),
        "accel": (TwistStamped, twist_values),
    }

    rospy.init_node("compare_vrpn_backends", anonymous=True)

    def callback(message, callback_data):
        key, extractor = callback_data
        records[key].append(
            {
                "receive_time": time.monotonic(),
                "stamp": message.header.stamp.to_sec(),
                "values": extractor(message),
            }
        )

    for backend, namespace in namespaces.items():
        for tracker in trackers:
            for kind, (message_type, extractor) in stream_types.items():
                key = (backend, tracker, kind)
                records[key] = []
                topic = f"{namespace}/{tracker}/{kind}"
                subscribers.append(
                    rospy.Subscriber(
                        topic,
                        message_type,
                        callback,
                        callback_args=(key, extractor),
                        queue_size=2000,
                    )
                )

    rospy.sleep(args.duration)
    report = {
        "purpose": "descriptive comparison only; no acceptance thresholds",
        "duration_s": args.duration,
        "pair_window_ms": args.pair_window_ms,
        "namespaces": namespaces,
        "trackers": {},
    }
    missing_streams = []
    for tracker in trackers:
        tracker_report = {}
        for kind in stream_types:
            left = records[("legacy", tracker, kind)]
            right = records[("plugin", tracker, kind)]
            if not left:
                missing_streams.append(f"legacy/{tracker}/{kind}")
            if not right:
                missing_streams.append(f"plugin/{tracker}/{kind}")
            tracker_report[kind] = {
                "legacy": stream_metrics(left) if left else {"count": 0},
                "plugin": stream_metrics(right) if right else {"count": 0},
                "comparison": paired_metrics(
                    left, right, kind, args.pair_window_ms / 1000.0
                ),
            }
        report["trackers"][tracker] = tracker_report
    report["diagnostic_status"] = "incomplete" if missing_streams else "complete"
    report["missing_streams"] = missing_streams

    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as destination:
            destination.write(rendered + "\n")

    # Differences and missing streams remain report data. This tool is not
    # registered as a test and intentionally has no comparison failure exit.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
