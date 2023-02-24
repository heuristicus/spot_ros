#!/usr/bin/env python3
import rosunit
import typing

test_cases: typing.List[typing.Tuple[str, str, str]] = [
    (
        "graph_nav_util",
        "graph_nav_util_test",
        "graph_nav_util_test.TestSuiteGraphNavUtil",
    ),
    ("ros_helpers", "ros_helpers_test", "ros_helpers_test.TestSuiteROSHelpers"),
]

# rosunit
for test_case in test_cases:
    rosunit.unitrun(test_case[0], test_case[1], test_case[2])
